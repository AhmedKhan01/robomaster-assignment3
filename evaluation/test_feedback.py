import json
from pathlib import Path
import unittest
from feedback_core import Session,reference,feedback,health

ROOT=Path(__file__).resolve().parent
P=json.loads((ROOT/'protocol.json').read_text())
C=json.loads((ROOT/'ekf_config.json').read_text())


def sensors(t,x=0.,range_m=None,v=0.):
    return dict(tof=dict(receipt=t,range=1.2-x if range_m is None else range_m,sensor_age=.01),
        odom=dict(receipt=t,position=[x,0.],quaternion=[0.,0.,0.,1.],source_stamp=1000+t-.01),
        tag=dict(receipt=t,visible=True,target_id=3,error=.02,source_stamp=1000+t-.02),
        driver=dict(receipt=t,status='READY'),applied=dict(receipt=t,v=v,w=0.))


class Checks(unittest.TestCase):
    def send(self,s,t,x=0.,range_m=None,v=0.,publishers=1):
        for key,item in sensors(t,x,range_m,v).items():
            s.receive(key,item,t,1000+t,publishers)

    def running(self,method='unfiltered'):
        events=[];zeros=[]
        s=Session(method,1.2,True,P,C,0.,emit=lambda *a:events.append(a),on_stop=lambda:zeros.append(True))
        for k in range(111):
            t=k*.05;self.send(s,t);s.tick(t,1000+t,1)
        self.assertIsNotNone(s.motion_start);self.assertIsNone(s.stop_time)
        return s,events,zeros

    def test_reference_derivatives_and_hold(self):
        d0=1.2;r=reference(0.,d0);self.assertAlmostEqual(r['L'],1.);self.assertAlmostEqual(r['T'],10.)
        for t in [1.,3.,5.,8.]:
            h=1e-5;r=reference(t,d0)
            self.assertAlmostEqual((reference(t+h,d0)['x']-reference(t-h,d0)['x'])/(2*h),r['v'],places=8)
            self.assertAlmostEqual((reference(t+h,d0)['v']-reference(t-h,d0)['v'])/(2*h),r['a'],places=8)
        self.assertEqual(reference(20.,d0)['v'],0.)
        self.assertEqual(reference(20.,d0)['a'],0.)
        self.assertAlmostEqual(reference(20.,d0)['x'],1.)

    def test_speed_cap_no_reverse_and_steering_limit(self):
        r=reference(5.,1.2)
        self.assertEqual(feedback(r,-100.,1.,.4,P),(.06,.10))
        self.assertEqual(feedback(r,100.,-1.,1.,P),(0.,-.10))
        self.assertEqual(feedback(r,-100.,0.,1.,P)[0],.16)

    def test_guard_uses_raw_range(self):
        self.assertEqual(health(0.,1000.,sensors(0.,range_m=.249),1,True,True,P),'RAW_TOF_GUARD')

    def test_preview_has_no_motion_output(self):
        s=Session('filtered',1.2,False,P,C,0.)
        for k in range(210):
            t=k*.05;self.send(s,t,publishers=0);r=s.tick(t,1000+t,0)
            self.assertEqual((r['requested_v_m_s'],r['requested_omega_rad_s']),(0.,0.))
        self.assertTrue(s.done);self.assertIsNone(s.motion_start)

    def test_fault_then_ready_cannot_resume(self):
        s,events,zeros=self.running();t=5.51
        s.receive('driver',dict(receipt=t,status='ODOMETRY_INVALID_OR_STALE'),t,1000+t,1)
        self.assertEqual(s.stop_reason,'DRIVER_ODOMETRY_INVALID_OR_STALE');self.assertTrue(zeros)
        self.send(s,5.52);r=s.tick(5.52,1005.52,1)
        self.assertEqual((r['requested_v_m_s'],r['requested_omega_rad_s']),(0.,0.))
        self.assertEqual(s.stop_reason,'DRIVER_ODOMETRY_INVALID_OR_STALE')
        self.assertEqual(sum(e[0]=='STOP' for e in events),1)

    def test_raw_guard_stops_in_callback(self):
        s,_,zeros=self.running()
        s.receive('tof',dict(receipt=5.51,range=.249,sensor_age=.01),5.51,1005.51,1)
        self.assertEqual(s.stop_reason,'RAW_TOF_GUARD');self.assertTrue(zeros)

    def test_clock_jump_stops(self):
        s,_,zeros=self.running()
        s.receive('driver',dict(receipt=5.51,status='READY'),5.51,1006.31,1)
        self.assertEqual(s.stop_reason,'CLOCK_JUMP');self.assertTrue(zeros)

    def test_extra_publisher_stops(self):
        s,_,_=self.running();r=s.tick(5.51,1005.51,2)
        self.assertEqual(s.stop_reason,'COMMAND_PUBLISHER_COUNT_2')
        self.assertEqual(r['requested_v_m_s'],0.)

    def test_malformed_sensor_latches(self):
        s,_,_=self.running();s.invalidate('odom',5.51)
        self.assertEqual(s.stop_reason,'ODOM_MALFORMED')

    def test_countdown_fault_resets(self):
        s=Session('unfiltered',1.2,True,P,C,0.)
        for k in range(61):
            t=.05*k;self.send(s,t);s.tick(t,1000+t,1)
        s.receive('tag',dict(receipt=3.01,visible=False,target_id=3,error=None,source_stamp=1003.01),3.01,1003.01,1)
        self.assertIsNone(s.healthy_since);self.assertIsNone(s.motion_start)
        self.send(s,3.02);s.tick(3.02,1003.02,1)
        self.assertEqual(s.healthy_since,3.02)

    def test_methods_change_only_feedback_estimate(self):
        a,_,_=self.running('unfiltered');b,_,_=self.running('filtered')
        t=a.motion_start+5.
        for s in (a,b):
            s.raw_x=.6;s.ekf.state[:]=[.7,0.,0.,0.,0.];s.inputs=sensors(t)
        ra=a.tick(t,1000+t,1);rb=b.tick(t,1000+t,1)
        self.assertEqual(ra['reference_x_m'],rb['reference_x_m'])
        self.assertEqual(ra['requested_omega_rad_s'],rb['requested_omega_rad_s'])
        self.assertEqual(a.protocol,b.protocol)
        self.assertAlmostEqual(ra['requested_v_m_s'],.10)
        self.assertAlmostEqual(rb['requested_v_m_s'],.05)

    def test_ideal_commissioning_approach_stops_at_common_guard(self):
        for method in ('unfiltered','filtered'):
            s=Session(method,1.2,True,P,C,0.)
            x=v=0.
            for k in range(1000):
                t=k*.05;x+=v*.05
                self.send(s,t,x=x,v=v)
                r=s.tick(t,1000+t,1);v=r['requested_v_m_s']
                if s.done:break
            self.assertTrue(s.done)
            self.assertEqual(s.stop_reason,'RAW_TOF_GUARD')
            self.assertEqual(v,0.)

    def test_unsafe_protocol_rejected(self):
        with self.assertRaises(ValueError):Session('filtered',1.2,False,dict(P,max_speed_m_s=.5),C,0.)
        with self.assertRaises(ValueError):Session('filtered',1.2,False,dict(P,raw_tof_stop_m=.1),C,0.)

    def test_stalled_progress_hits_runtime_bound(self):
        s,_,zeros=self.running()
        t=s.motion_start+reference(0.,1.2)['T']+P['post_reference_timeout_s']+.01
        s.inputs=sensors(t);r=s.tick(t,1000+t,1)
        self.assertEqual(s.stop_reason,'BOUNDED_RUNTIME');self.assertTrue(zeros)
        self.assertEqual(r['requested_v_m_s'],0.)


if __name__=='__main__':unittest.main()
