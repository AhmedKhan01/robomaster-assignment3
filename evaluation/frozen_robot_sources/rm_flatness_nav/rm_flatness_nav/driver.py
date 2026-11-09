# VISION_STREAM_V3
"""SDK owner. Independent command timeout and ToF stop guard."""
import math
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import json
from .odometry_support import OdometryBuffer
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray, String
from robomaster import robot


class Driver(Node):
    def __init__(self):
        super().__init__('rm_driver')
        cv2.setNumThreads(1)
        self.declare_parameter('sdk_yaw_sign', -1.0)
        self.declare_parameter('initialize_arm', True)
        self.declare_parameter('tof_index', 0)
        self.declare_parameter('odom_y_sign', -1.0)
        self.declare_parameter('odom_yaw_sign', -1.0)
        self.odom_buffer = OdometryBuffer(
            float(self.get_parameter('odom_y_sign').value),
            float(self.get_parameter('odom_yaw_sign').value))
        self.last_odom_seq = 0
        self.position_sub = self.attitude_sub = self.velocity_sub = False
        self.yaw_sign = float(self.get_parameter('sdk_yaw_sign').value)
        if self.yaw_sign not in (-1, 1):
            raise ValueError('sdk_yaw_sign must be -1 or +1')
        self.tof_index = int(self.get_parameter('tof_index').value)
        if not 0 <= self.tof_index <= 3:
            raise ValueError('tof_index must be 0..3')
        self.cmd = (0., 0., -math.inf)
        self.tof = (math.nan, -math.inf)
        self.esc = (None, -math.inf)
        self.stop_latched = False
        self.running = True
        self.frame = None
        self.frame_id = 0
        self.sent_frame = -1
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.thread = None
        self.ep = robot.Robot()
        self.started = self.dist_sub = self.esc_sub = False
        image_qos = qos_profile_sensor_data.__class__(depth=1, reliability=qos_profile_sensor_data.reliability)
        self.image_pub = self.create_publisher(CompressedImage, '/rm/image/compressed', image_qos)
        self.odom_pub = self.create_publisher(Odometry, '/rm/odom', 10)
        self.odom_raw_pub = self.create_publisher(String, '/rm/odom_raw', 10)
        self.range_pub = self.create_publisher(Float64MultiArray, '/rm/range', 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/rm/wheels', 10)
        self.applied_pub = self.create_publisher(Twist, '/rm/applied_cmd', 10)
        self.health_pub = self.create_publisher(String, '/rm/driver_status', 10)
        self.create_subscription(Twist, '/rm/cmd_vel', self.on_cmd, 1)
        try:
            self.ep.initialize(conn_type='ap')
            self.ep.chassis.drive_speed(x=0, y=0, z=0, timeout=0.3)
            if self.get_parameter('initialize_arm').value:
                self.get_logger().info('Moving arm to (105,35) mm; mounting MUST be clear.')
                if not self.ep.robotic_arm.moveto(x=105, y=35).wait_for_completed(timeout=15):
                    raise RuntimeError('Arm startup action failed')
                time.sleep(1)
            self.dist_sub = self.ep.sensor.sub_distance(freq=20, callback=self.on_tof)
            self.esc_sub = self.ep.chassis.sub_esc(freq=20, callback=self.on_esc)
            if not self.dist_sub or not self.esc_sub:
                raise RuntimeError('ToF or ESC subscription rejected')
            self.position_sub = self.ep.chassis.sub_position(cs=1, freq=20, callback=self.on_position)
            self.attitude_sub = self.ep.chassis.sub_attitude(freq=20, callback=self.on_attitude)
            self.velocity_sub = self.ep.chassis.sub_velocity(freq=20, callback=self.on_velocity)
            if not all((self.position_sub, self.attitude_sub, self.velocity_sub)):
                raise RuntimeError('Odometry subscription rejected; driver stopped')
            self.started = self.ep.camera.start_video_stream(display=False, resolution='720p')
            if not self.started:
                raise RuntimeError('Camera startup failed')
            self.thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.thread.start()
            self.create_timer(0.05, self.tick)
            self.create_timer(0.10, self.publish_image, callback_group=self.image_group)
            self.get_logger().info('Driver ready; no motion until planner commands arrive.')
        except BaseException:
            self.close()
            raise

    def on_cmd(self, msg):
        self.cmd = (float(msg.linear.x), float(msg.angular.z), time.monotonic())

    def on_tof(self, data):
        d = float(data[self.tof_index])/1000.
        self.tof = (d, time.monotonic())
        if math.isfinite(d) and 0.10 <= d <= 0.20:
            self.stop_latched = True

    def on_esc(self, data):
        self.esc = (list(data[0]), time.monotonic())

    def receive_odom(self, channel, data):
        ros_s = self.get_clock().now().nanoseconds*1e-9
        try:
            valid = self.odom_buffer.put(channel, data, ros_s)
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            self.get_logger().warning('Rejected malformed SDK '+channel+' observation')

    def on_position(self, data):
        self.receive_odom('position', data)

    def on_attitude(self, data):
        self.receive_odom('attitude', data)

    def on_velocity(self, data):
        self.receive_odom('velocity', data)

    def publish_odometry(self, now):
        samples, _ = self.odom_buffer.snapshot(now)
        self.odom_raw_pub.publish(String(data=json.dumps({
            'channels': samples,
            'position_cs': 1,
            'position_fields': ['sdk_x_m', 'sdk_y_m', 'sdk_third_position_field'],
            'attitude_fields': ['yaw_deg', 'pitch_deg', 'roll_deg'],
            'velocity_fields': ['vgx', 'vgy', 'vgz', 'vbx', 'vby', 'vbz'],
            'odom_y_sign': self.odom_buffer.y_sign,
            'odom_yaw_sign': self.odom_buffer.yaw_sign,
            'timestamp_semantics': 'SDK callback receipt; not hardware acquisition time',
        }, allow_nan=False)))
        pose = self.odom_buffer.pose(now)
        if pose is None or pose['position_seq'] == self.last_odom_seq:
            return
        # Publish a position sample once. Stale samples are never stamped as new.
        self.last_odom_seq = pose['position_seq']
        msg = Odometry()
        stamp_ns = round(pose['stamp_ros_s']*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
        msg.header.frame_id, msg.child_frame_id = 'rm_sdk_odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose['x'], pose['y']
        msg.pose.pose.orientation.z = math.sin(pose['yaw']/2)
        msg.pose.pose.orientation.w = math.cos(pose['yaw']/2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = pose['vx'], pose['vy']
        msg.twist.twist.angular.z = pose['yaw_rate']
        # Uncalibrated placeholder uncertainties, NOT experimentally estimated R.
        for index in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[index] = 1e6
            msg.twist.covariance[index] = 1e6
        self.odom_pub.publish(msg)

    def camera_loop(self):
        while self.running:
            try:
                frame = self.ep.camera.read_cv2_image(strategy='newest', timeout=1)
                if frame is not None:
                    self.frame_id += 1
                    self.frame = (frame, self.get_clock().now().to_msg(), self.frame_id)
            except queue.Empty:
                pass
            except Exception as exc:
                self.get_logger().error('Camera read failed: '+str(exc))
                break
            time.sleep(0.010)

    def tick(self):
        now = time.monotonic()
        d, td = self.tof
        rpm, te = self.esc
        v, w, tc = self.cmd
        status = 'READY'
        odom_pose = self.odom_buffer.pose(now)
        if self.stop_latched:
            v = w = 0.
            status = 'TOF_STOP_LATCHED'
        elif not math.isfinite(d) or not 0.10 <= d <= 10 or now-td > .40:
            v = w = 0.
            status = 'TOF_INVALID_OR_STALE'
        elif now-te > .50:
            v = w = 0.
            status = 'ESC_STALE'
        elif odom_pose is None or not odom_pose['velocity_valid']:
            v = w = 0.
            status = 'ODOMETRY_INVALID_OR_STALE'
        elif now-tc > .25 or not math.isfinite(v+w):
            v = w = 0.
            status = 'COMMAND_TIMEOUT'
        v, w = float(np.clip(v, 0, .16)), float(np.clip(w, -.30, .30))
        self.ep.chassis.drive_speed(x=v, y=0, z=self.yaw_sign*math.degrees(w), timeout=.30)
        applied = Twist()
        applied.linear.x, applied.angular.z = v, w
        self.applied_pub.publish(applied)
        self.health_pub.publish(String(data=status))
        self.publish_odometry(now)
        self.range_pub.publish(Float64MultiArray(data=[d, now-td]))
        if rpm is not None:
            self.wheel_pub.publish(Float64MultiArray(data=[*map(float, rpm), now-te]))

    def publish_image(self):
        item = self.frame
        if item is None or item[2] == self.sent_frame:
            return
        frame, stamp, seq = item
        # Full-resolution grayscale JPEG; original frame receipt stamp retained.
        age = self.get_clock().now().nanoseconds*1e-9 - (stamp.sec+stamp.nanosec*1e-9)
        if age > .30:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, encoded = cv2.imencode('.jpg', gray, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp, msg.header.frame_id = stamp, 'camera'
        msg.format = 'mono8; jpeg compressed'
        msg.data = encoded.tobytes()
        self.image_pub.publish(msg)
        self.sent_frame = seq

    def close(self):
        self.running = False
        try:
            self.ep.chassis.drive_speed(x=0, y=0, z=0, timeout=.3)
        except Exception:
            pass
        for enabled, fn in [(self.position_sub, self.ep.chassis.unsub_position),
                            (self.attitude_sub, self.ep.chassis.unsub_attitude),
                            (self.velocity_sub, self.ep.chassis.unsub_velocity),
                            (self.dist_sub, self.ep.sensor.unsub_distance),
                            (self.esc_sub, self.ep.chassis.unsub_esc),
                            (self.started, self.ep.camera.stop_video_stream)]:
            if enabled:
                try:
                    fn()
                except Exception:
                    pass
        if self.thread:
            self.thread.join(timeout=2)
        try:
            self.ep.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = Driver()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
