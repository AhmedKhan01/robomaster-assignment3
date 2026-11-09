"""Five-state EKF. No ROS or motor interface. SI units throughout."""
import math
import numpy as np


def wrap(a):
    return (a+math.pi) % (2*math.pi)-math.pi


def dynamics(state, control, dt, cfg):
    """Euler pose integration and first-order response of body speed/yaw rate.

    State [x,y,theta,v,omega]; input [commanded body-v, commanded yaw-rate].
    G maps continuous velocity/yaw-rate diffusion to discrete state noise.
    """
    x,y,theta,v,w=np.asarray(state,dtype=float)
    uv,uw=np.asarray(control,dtype=float)
    av=1-math.exp(-dt/cfg['velocity_time_constant_s'])
    aw=1-math.exp(-dt/cfg['yaw_rate_time_constant_s'])
    c,s=math.cos(theta),math.sin(theta)
    nxt=np.array([x+v*c*dt,y+v*s*dt,wrap(theta+w*dt),
                  v+av*(uv-v),w+aw*(uw-w)])
    F=np.eye(5)
    F[0,2]=-v*s*dt;F[0,3]=c*dt
    F[1,2]=v*c*dt;F[1,3]=s*dt
    F[2,4]=dt;F[3,3]=1-av;F[4,4]=1-aw
    G=np.zeros((5,2));G[3,0]=math.sqrt(dt);G[4,1]=math.sqrt(dt)
    return nxt,F,G


def relative_pose(position, quaternion, origin_position, origin_yaw):
    dx,dy=np.asarray(position,dtype=float)-origin_position
    c,s=math.cos(origin_yaw),math.sin(origin_yaw)
    qx,qy,qz,qw=quaternion
    yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    return np.array([dx*c+dy*s,-dx*s+dy*c,wrap(yaw-origin_yaw)])


class EKF:
    def __init__(self,config):
        self.cfg=dict(config)
        positive=['velocity_time_constant_s','yaw_rate_time_constant_s',
                  'velocity_diffusion','yaw_rate_diffusion','odom_x_variance_m2',
                  'odom_y_variance_m2','tof_variance_m2','minimum_update_interval_s',
                  'bias_ramp_s']
        if any(not math.isfinite(self.cfg[k]) or self.cfg[k]<=0 for k in positive):
            raise ValueError('Noise, timing and response parameters must be positive and finite.')
        if not math.isfinite(self.cfg['odometry_bias_m']) or not 0<self.cfg['tof_travel_scale']<2:
            raise ValueError('Invalid calibration.')
        initial=np.asarray(self.cfg['initial_state_sd'],float)
        if initial.shape!=(5,) or not np.isfinite(initial).all() or np.any(initial<=0):
            raise ValueError('Require five positive finite initial standard deviations.')
        self.state=np.zeros(5)
        self.P=np.diag(initial**2)
        self.Q=np.diag([self.cfg['velocity_diffusion']**2,self.cfg['yaw_rate_diffusion']**2])
        self.time=0.
        self.last_update={'odom':-math.inf,'tof':-math.inf}
        self.last_source={'odom':-math.inf,'tof':-math.inf}
        self.counts={'odom_accepted':0,'tof_accepted':0,'rejected':0}

    def predict_to(self,elapsed,control):
        if not math.isfinite(elapsed) or elapsed<self.time-1e-9:
            raise ValueError('Prediction time must be finite and monotonic.')
        if len(control)!=2 or not np.isfinite(control).all():
            raise ValueError('Invalid control.')
        while elapsed-self.time>1e-10:
            dt=min(.02,elapsed-self.time)
            self.state,F,G=dynamics(self.state,control,dt,self.cfg)
            self.P=F@self.P@F.T+G@self.Q@G.T
            self.P=(self.P+self.P.T)/2
            self.time+=dt

    def _eligible(self,kind,source_elapsed,age):
        valid=(math.isfinite(source_elapsed) and math.isfinite(age)
               and -.05<=age<=.4 and source_elapsed>=-.4
               and source_elapsed<=self.time+.05
               and source_elapsed>self.last_source[kind]+1e-6
               and self.time-self.last_update[kind]>=self.cfg['minimum_update_interval_s']-1e-9)
        if not valid:
            self.counts['rejected']+=1
        return valid

    def _update(self,z,H,R,kind,source):
        z=np.asarray(z,float)
        if not np.isfinite(z).all():
            raise ValueError('Nonfinite measurement.')
        innovation=z-H@self.state
        S=H@self.P@H.T+R
        K=np.linalg.solve(S,H@self.P).T
        self.state=self.state+K@innovation
        self.state[2]=wrap(self.state[2])
        A=np.eye(5)-K@H
        self.P=A@self.P@A.T+K@R@K.T  # Joseph covariance update
        self.P=(self.P+self.P.T)/2
        self.last_update[kind]=self.time
        self.last_source[kind]=source
        self.counts[kind+'_accepted']+=1
        return True

    def observe_odom(self,x,y,source_elapsed,age):
        if not self._eligible('odom',source_elapsed,age):
            return False
        # Endpoint-fitted bias is introduced smoothly during initial 2 s.
        # This ramp is an explicit modeling choice, not an identified drift law.
        bias=self.cfg['odometry_bias_m']*min(1.,max(0.,self.time/self.cfg['bias_ramp_s']))
        H=np.zeros((2,5));H[0,0]=H[1,1]=1.
        R=np.diag([self.cfg['odom_x_variance_m2'],self.cfg['odom_y_variance_m2']])
        return self._update([x-bias,y],H,R,'odom',source_elapsed)

    def observe_tof(self,travel,source_elapsed,age,starting_range_variance=0.):
        if not math.isfinite(starting_range_variance) or starting_range_variance<0:
            raise ValueError('Invalid starting range variance.')
        if not self._eligible('tof',source_elapsed,age):
            return False
        H=np.zeros((1,5));H[0,0]=1.
        # Shared r0 error remains temporally correlated; inflating R cannot
        # turn repeated measurements into independent physical evidence.
        variance=self.cfg['tof_variance_m2']+self.cfg['tof_travel_scale']**2*starting_range_variance
        return self._update([travel],H,np.array([[variance]]),'tof',source_elapsed)
