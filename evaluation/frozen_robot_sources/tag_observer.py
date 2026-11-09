#!/usr/bin/env python3
"""Observe AprilTag readability/centering. No SDK connection or motion publishers.

Run with the existing driver running and the old planner stopped.
Image error is in pixels/normalized image coordinates, NOT radians.
"""
import argparse
import json
import math
import time


def marker_center(corners):
    """Projective square centre: intersection of its corner diagonals."""
    import numpy as np
    points = np.asarray(corners, dtype=float).reshape(4, 2)
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite corners')
    p = np.column_stack((points, np.ones(4)))
    center = np.cross(np.cross(p[0], p[2]), np.cross(p[1], p[3]))
    if abs(center[2]) < 1e-9:
        raise ValueError('Degenerate marker geometry')
    u, v = center[:2]/center[2]
    return float(u), float(v)


def centering_error(u, width):
    if width <= 0 or not math.isfinite(u):
        raise ValueError('Invalid image centre or width')
    # Image midpoint reference, not a calibrated optical principal point.
    image_midpoint = (width-1)/2
    pixels_left = image_midpoint-u
    return pixels_left, pixels_left/(width/2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-id', type=int, default=3)
    parser.add_argument('--max-age', type=float, default=0.30)
    parser.add_argument('--topic', default='/rm/image/compressed')
    args = parser.parse_args()
    if not math.isfinite(args.max_age) or args.max_age <= 0:
        parser.error('max-age must be positive and finite')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    cv2.setNumThreads(1)

    class Observer(Node):
        def __init__(self):
            super().__init__('assessment3_tag_observer')
            self.detector = cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
            self.frames = self.detections = self.errors = 0
            self.state = 'WAITING_FOR_IMAGE'
            self.ids = []
            self.last = None
            self.last_frame_receipt = None
            self.processing_s = None
            self.last_print = -math.inf
            self.last_published_key = None
            self.pub = self.create_publisher(String, '/rm/tag_observation', 10)
            self.sub = self.create_subscription(CompressedImage, args.topic,
                                                 self.on_image, qos_profile_sensor_data)
            self.create_timer(0.10, self.publish_status)
            self.get_logger().info('Upright/floor tag readability observer; no distance calculation or motor commands.')

        def on_image(self, msg):
            started = time.monotonic()
            self.last_frame_receipt = started
            self.frames += 1
            self.ids = []
            try:
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if age < -0.05 or age > args.max_age:
                    self.state = 'FRAME_AGE_REJECTED'
                    self.last = None
                    return
                gray = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
                if gray is None:
                    self.state = 'IMAGE_DECODE_FAILED'
                    self.last = None
                    return
                height, width = gray.shape
                corners, ids, _ = self.detector.detectMarkers(gray)
                if ids is None:
                    self.state = 'NO_MARKERS_DETECTED'
                    self.last = None
                    return
                self.ids = [int(i) for i in ids.flatten()]
                indices = [i for i,value in enumerate(self.ids) if value == args.target_id]
                if len(indices) != 1:
                    self.state = 'TARGET_NOT_FOUND' if not indices else 'AMBIGUOUS_TARGET_ID'
                    self.last = None
                    return
                u,v = marker_center(corners[indices[0]])
                error_px,error_normalized = centering_error(u,width)
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if age < -0.05 or age > args.max_age:
                    self.state = 'PROCESSING_AGE_REJECTED'
                    self.last = None
                    return
                self.detections += 1
                self.last = dict(source_stamp_ros_s=stamp,
                    received_monotonic_s=started, center_u_px=u, center_v_px=v,
                    image_width_px=width, image_height_px=height,
                    horizontal_error_px=error_px,
                    horizontal_error_normalized=error_normalized,
                    corners_px=np.asarray(corners[indices[0]], dtype=float).reshape(4,2).tolist())
                self.state = 'TARGET_OK'
            except Exception as exc:
                self.errors += 1
                self.last = None
                self.state = 'CALLBACK_ERROR_'+type(exc).__name__
                self.get_logger().error(repr(exc))
            finally:
                self.processing_s = time.monotonic()-started
                # Publish this frame immediately; a second timer must not add
                # another image period to its acquisition-to-controller age.
                self.publish_status()

        def publish_status(self):
            now = time.monotonic()
            last = self.last
            age = None if last is None else self.get_clock().now().nanoseconds*1e-9-last['source_stamp_ros_s']
            receipt_age = None if last is None else now-last['received_monotonic_s']
            visible = bool(last and -0.05 <= age <= args.max_age and 0 <= receipt_age <= args.max_age)
            status = self.state
            if status == 'TARGET_OK' and not visible:
                status = 'TARGET_STALE'
            if self.last_frame_receipt is not None and now-self.last_frame_receipt > args.max_age:
                status = 'IMAGE_STREAM_STALE'
            observation = dict(last) if visible else None
            if observation is not None:
                observation.pop('received_monotonic_s')
            details = dict(target_id=args.target_id, visible=visible, status=status,
                image_frames_received=self.frames, target_detections=self.detections,
                callback_errors=self.errors, detected_ids=self.ids,
                source_age_s=age, processing_s=self.processing_s, observation=observation,
                error_convention='positive = target left of image midpoint; pixels/normalized units, not radians',
                camera_distance_m=None)
            key = (status, visible, None if observation is None else observation['source_stamp_ros_s'])
            if key != self.last_published_key:
                self.pub.publish(String(data=json.dumps(details, allow_nan=False)))
                self.last_published_key = key
            # Timer still publishes transitions to stale/invalid state promptly;
            # repeated valid copies of the same frame are not fresh measurements.
            if now-self.last_print >= 1.0:
                self.get_logger().info(json.dumps(details, allow_nan=False))
                self.last_print = now

    rclpy.init()
    node = None
    try:
        node = Observer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
