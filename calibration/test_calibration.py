"""Recorded-endpoint regression and callback fault-latching checks; no ROS required."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from calibrate import analyze_run
from install_pilot_supervisor_fix import patched_text

ROOT=Path(__file__).resolve().parent


class CalibrationChecks(unittest.TestCase):
    def test_long_recorded_endpoint(self):
        physical=next(r for r in json.loads((ROOT/'physical_measurements.json').read_text())['runs'] if r['run_id']=='long01')
        result,aligned=analyze_run(ROOT/'runs/long01',physical)
        self.assertAlmostEqual(result['physical_travel_m'],.5)
        self.assertAlmostEqual(result['odometry_travel_m'],.5000443785696921,places=9)
        self.assertAlmostEqual(result['tof_drop_m'],.522,places=9)
        self.assertFalse(result['driver_faults_during_motion'])
        self.assertTrue(aligned)

    def test_bad_measurement_identity_rejected(self):
        with self.assertRaises(ValueError):
            analyze_run(ROOT/'runs/long01',{'run_id':'wrong','start_gap_m':1.2,'final_gap_m':.7,'resolution_m':.001})


class SupervisorChecks(unittest.TestCase):
    def make_pilot(self):
        # Load only the patched class with ROS-free fakes, not main().
        original=(ROOT/'pilot_reference_for_tests.txt').read_text()
        updated=patched_text(original)
        self.assertEqual(patched_text(updated),updated)
        tree=ast.parse(updated)
        cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='Pilot')
        core={}
        exec((ROOT/'pilot_core_reference_for_tests.txt').read_text(),core)
        clock=SimpleNamespace(monotonic=lambda:100.)
        env=dict(Node=object,time=clock,math=math,json=json,
                 args=SimpleNamespace(execute=True,duration=10),
                 check_health=core['check_health'],command_at=core['command_at'],
                 message_to_ordereddict=lambda msg:vars(msg))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'pilot_class','exec'),env)
        p=object.__new__(env['Pilot'])
        p.inputs=dict(tof=dict(receipt=99.99,range=1.,sensor_age=.01),
            odom=dict(receipt=99.99,position=[0.,0.],quaternion=[0.,0.,0.,1.],source_stamp=99.99),
            tag=dict(receipt=99.99,visible=True,target_id=3,error=0.,source_stamp=99.99),
            driver=dict(receipt=99.99,status='READY'))
        p.motion_start=99.;p.stop_time=None;p.reason=None;p.healthy_since=95.;p.phase='MOVING'
        p.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100000000000))
        p.count_publishers=lambda _:1
        p.events=[];p.commands=[]
        p.log=lambda topic,payload:p.events.append((topic,payload))
        p.publish_command=lambda v=0.,w=0.:p.commands.append((v,w))
        return p

    def test_transient_driver_fault_cannot_be_erased_by_ready(self):
        p=self.make_pilot()
        p.receive('/rm/driver_status',SimpleNamespace(data='ODOMETRY_INVALID_OR_STALE'))
        self.assertEqual(p.reason,'DRIVER_ODOMETRY_INVALID_OR_STALE')
        self.assertEqual(p.commands[-1],(0.,0.))
        p.receive('/rm/driver_status',SimpleNamespace(data='READY'))
        self.assertEqual(p.reason,'DRIVER_ODOMETRY_INVALID_OR_STALE')
        self.assertIsNotNone(p.stop_time)

    def test_active_command_timeout_stops(self):
        p=self.make_pilot()
        p.receive('/rm/driver_status',SimpleNamespace(data='COMMAND_TIMEOUT'))
        self.assertEqual(p.reason,'DRIVER_COMMAND_TIMEOUT')

    def test_prestart_timeout_allowed_but_fault_resets_countdown(self):
        p=self.make_pilot();p.motion_start=None
        p.receive('/rm/driver_status',SimpleNamespace(data='COMMAND_TIMEOUT'))
        self.assertEqual(p.healthy_since,95.)
        p.receive('/rm/driver_status',SimpleNamespace(data='ESC_STALE'))
        self.assertIsNone(p.healthy_since)
        self.assertIsNone(p.stop_time)

    def test_lost_tag_stops_in_callback(self):
        p=self.make_pilot()
        p.receive('/rm/tag_observation',SimpleNamespace(data=json.dumps(dict(visible=False,target_id=3,observation=None))))
        self.assertEqual(p.reason,'TAG3_NOT_VISIBLE')
        self.assertEqual(p.commands[-1],(0.,0.))


if __name__=='__main__':
    unittest.main()
