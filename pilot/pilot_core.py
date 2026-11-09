"""Supervision for a bounded, timed characterization segment. No EKF."""
import math


def command_at(elapsed, duration, speed, tag_error, steer_gain=.4):
    if not all(math.isfinite(v) for v in (elapsed,duration,speed,tag_error,steer_gain)):
        raise ValueError('Nonfinite command input')
    if not 0 < speed <= .12 or not 0 < duration <= 30 or elapsed < 0:
        raise ValueError('Invalid segment input')
    if elapsed >= duration:
        return 0.,0.
    return speed, max(-.10,min(.10,steer_gain*tag_error))


def check_health(now, ros_now, inputs, command_publishers, execute):
    """Return a stop reason, or None. Receipt times are local monotonic seconds."""
    if execute and command_publishers != 1:
        return 'COMMAND_PUBLISHER_COUNT_'+str(command_publishers)
    if not execute and command_publishers != 0:
        return 'EXISTING_COMMAND_PUBLISHER'
    for key,limit in [('tof',.4),('odom',.4),('tag',.3),('driver',.4)]:
        item=inputs.get(key)
        if item is None:
            return key.upper()+'_MISSING'
        if not 0 <= now-item['receipt'] <= limit:
            return key.upper()+'_RECEIPT_STALE'
    t=inputs['tof']
    if not math.isfinite(t['range']) or not .10 <= t['range'] <= 10:
        return 'TOF_INVALID'
    if not math.isfinite(t['sensor_age']) or not 0 <= t['sensor_age']+now-t['receipt'] <= .4:
        return 'TOF_SOURCE_STALE'
    if t['range'] <= .30:
        return 'RAW_TOF_GUARD_0p30m'
    o=inputs['odom']
    if not all(math.isfinite(v) for v in (*o['position'],*o['quaternion'],o['source_stamp'])):
        return 'ODOM_INVALID'
    if sum(v*v for v in o['quaternion']) < .5:
        return 'ODOM_INVALID_QUATERNION'
    if not -.05 <= ros_now-o['source_stamp'] <= .4:
        return 'ODOM_SOURCE_STALE'
    t=inputs['tag']
    if not t['visible'] or t['target_id'] != 3 or t['error'] is None or not math.isfinite(t['error']):
        return 'TAG3_NOT_VISIBLE'
    if abs(t['error']) > 1:
        return 'TAG_ERROR_INVALID'
    if not math.isfinite(t['source_stamp']) or not -.05 <= ros_now-t['source_stamp'] <= .3:
        return 'TAG_SOURCE_STALE'
    if abs(t['error']) > .15:
        return 'TAG_NOT_CENTERED'
    # COMMAND_TIMEOUT is expected before this node begins supplying zero commands.
    if inputs['driver']['status'] not in ('READY','COMMAND_TIMEOUT'):
        return 'DRIVER_'+inputs['driver']['status']
    return None
