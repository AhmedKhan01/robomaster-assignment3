import json
import math
from pathlib import Path
import unittest
import numpy as np
from ekf_core import EKF,dynamics,relative_pose,wrap

CFG=json.loads(Path(__file__).with_name('ekf_config.json').read_text())


class Checks(unittest.TestCase):
    def test_jacobian_against_central_differences(self):
        state=np.array([.3,-.2,.4,.08,-.03]);control=[.12,.05];dt=.02
        _,F,_=dynamics(state,control,dt,CFG)
        numerical=np.empty((5,5));eps=1e-6
        for k in range(5):
            delta=np.zeros(5);delta[k]=eps
            numerical[:,k]=(dynamics(state+delta,control,dt,CFG)[0]-dynamics(state-delta,control,dt,CFG)[0])/(2*eps)
        np.testing.assert_allclose(F,numerical,rtol=1e-6,atol=1e-8)

    def test_rotated_origin(self):
        pos=relative_pose([2.,4.],[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)],[2.,3.],math.pi/2)
        np.testing.assert_allclose(pos,[1.,0.,0.],atol=1e-12)

    def test_covariance_remains_symmetric_positive(self):
        e=EKF(CFG)
        for k in range(1,501):
            t=k*.02;e.predict_to(t,[.08,.03])
            if k%15==0:
                e.observe_odom(.08*t,.001,t-.02,.02)
                e.observe_tof(.08*t,t-.01,.01)
            np.testing.assert_allclose(e.P,e.P.T,atol=1e-12)
            self.assertGreaterEqual(np.linalg.eigvalsh(e.P).min(),-1e-12)
        self.assertTrue(np.isfinite(e.state).all())

    def test_stale_and_repeated_source_rejected(self):
        e=EKF(CFG);e.predict_to(1.,[0.,0.])
        self.assertFalse(e.observe_tof(.1,.5,.5))
        self.assertTrue(e.observe_tof(.1,.99,.01))
        e.predict_to(1.5,[0.,0.])
        self.assertFalse(e.observe_tof(.1,.99,.01))

    def test_larger_R_reduces_measurement_influence(self):
        a=EKF(CFG);b=EKF(dict(CFG,tof_variance_m2=CFG['tof_variance_m2']*100))
        a.observe_tof(1.,0.,0.);b.observe_tof(1.,0.,0.)
        self.assertGreater(a.state[0],b.state[0])

    def test_time_reversal_and_nonfinite_control_rejected(self):
        e=EKF(CFG);e.predict_to(1.,[0.,0.])
        with self.assertRaises(ValueError):e.predict_to(.9,[0.,0.])
        with self.assertRaises(ValueError):e.predict_to(1.1,[math.nan,0.])

    def test_straight_motion_with_position_observations(self):
        cfg=dict(CFG,odometry_bias_m=0.)
        e=EKF(cfg)
        for k in range(1,401):
            t=.02*k;e.predict_to(t,[.08,0.])
            if k%15==0:
                e.observe_odom(.08*t,0.,t,0.)
                e.observe_tof(.08*t,t,0.)
        self.assertLess(abs(e.state[0]-.64),.015)
        self.assertLess(abs(e.state[1]),1e-10)


if __name__=='__main__':unittest.main()
