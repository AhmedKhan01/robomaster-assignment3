#!/usr/bin/env python3
"""Back up and patch the known pilot so received faults cannot clear between ticks.

This only edits the pilot file; it never starts ROS or commands the robot.
"""
import argparse
import hashlib
from pathlib import Path
import shutil
import time

OLD = '''                self.log('MALFORMED_INPUT',dict(topic=topic,error=str(exc)))

        def publish_command'''

NEW = '''                self.log('MALFORMED_INPUT',dict(topic=topic,error=str(exc)))
            # Do not let a fault message be overwritten by READY before tick().
            if topic in ('/rm/range','/rm/odom','/rm/tag_observation','/rm/driver_status'):
                ros_now = self.get_clock().now().nanoseconds*1e-9
                issue = check_health(now,ros_now,self.inputs,
                    self.count_publishers('/rm/cmd_vel'),args.execute)
                active = self.motion_start is not None and self.stop_time is None
                if active and topic == '/rm/driver_status' and msg.data == 'COMMAND_TIMEOUT':
                    issue = 'DRIVER_COMMAND_TIMEOUT'
                if issue:
                    self.healthy_since = None
                    if active:
                        self.stop(issue)

        def publish_command'''


def patched_text(text):
    if NEW in text:
        return text
    if text.count(OLD) != 1:
        raise ValueError('Expected pilot receive method not found exactly once. Nothing changed.')
    updated=text.replace(OLD,NEW)
    compile(updated,'calibration_pilot.py','exec')
    return updated


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot',type=Path,default=Path.home()/'assignment3_tools/assignment3_pilot/calibration_pilot.py')
    args=parser.parse_args()
    old=args.pilot.read_text()
    updated=patched_text(old)
    if old == updated:
        print('Pilot already has the callback fault latch. Nothing changed.')
        return
    # Require the exact original pilot supplied for this project.
    expected=(Path(__file__).parent/'original_pilot_sha256.txt').read_text().strip()
    if hashlib.sha256(args.pilot.read_bytes()).hexdigest() != expected:
        raise SystemExit('Pilot differs from the expected original. Nothing changed; share your current pilot for review.')
    backup=args.pilot.with_name(f'calibration_pilot.py.backup_{time.time_ns()}')
    shutil.copy2(args.pilot,backup)
    args.pilot.write_text(updated)
    print('Backup:',backup)
    print('Installed callback fault latch. No robot commands sent.')


if __name__=='__main__':
    main()
