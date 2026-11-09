"""Shared guarded feedback for commissioning; ROS-free logic, SI units."""
import math
from ekf_core import EKF,relative_pose


def reference(t,start_gap):
    if not math.isfinite(t+start_gap) or start_gap<=.20:
        raise ValueError('Invalid reference inputs.')
    L=start_gap-.20;T=max(4.,1.6*L/.16)
    if t>=T:return dict(x=L,v=0.,a=0.,L=L,T=T)
    tau=max(0.,t/T)
    return dict(x=L*(3*tau*tau-2*tau**3),v=6*L/T*tau*(1-tau),
                a=6*L/T**2*(1-2*tau),L=L,T=T)


def feedback(ref,progress,tag_error,raw_tof,protocol):
    if not all(math.isfinite(x) for x in (progress,tag_error,raw_tof,ref['x'],ref['v'])):
        raise ValueError('Nonfinite feedback input.')
    v=max(0.,min(protocol['max_speed_m_s'],ref['v']+protocol['kp_per_s']*(ref['x']-progress)))
    if raw_tof<=protocol['near_tof_m']:v=min(v,protocol['near_speed_cap_m_s'])
    w=max(-protocol['max_yaw_rate_rad_s'],min(protocol['max_yaw_rate_rad_s'],
          protocol['steering_gain_rad_s_per_normalized_error']*tag_error))
    return v,w


def health(now,ros,inputs,publishers,execute,active,protocol):
    if publishers!=(1 if execute else 0):return 'COMMAND_PUBLISHER_COUNT_'+str(publishers)
    for key,limit in [('tof',.4),('odom',.4),('tag',.3),('driver',.4),('applied',.4)]:
        item=inputs.get(key)
        if item is None:return key.upper()+'_MISSING'
        if not 0<=now-item['receipt']<=limit:return key.upper()+'_RECEIPT_STALE'
    t=inputs['tof'];age=t['sensor_age']+now-t['receipt']
    if not math.isfinite(t['range']) or not .10<=t['range']<=10:return 'TOF_INVALID'
    if not math.isfinite(age) or not 0<=age<=.4:return 'TOF_SOURCE_STALE'
    if t['range']<=protocol['raw_tof_stop_m']:return 'RAW_TOF_GUARD'
    o=inputs['odom']
    if not all(math.isfinite(v) for v in (*o['position'],*o['quaternion'],o['source_stamp'])):return 'ODOM_INVALID'
    if abs(sum(v*v for v in o['quaternion'])-1)>.1:return 'ODOM_INVALID_QUATERNION'
    if not -.05<=ros-o['source_stamp']<=.4:return 'ODOM_SOURCE_STALE'
    t=inputs['tag']
    if t['visible'] is not True or t['target_id']!=3:return 'TAG3_NOT_VISIBLE'
    if t['error'] is None or not math.isfinite(t['error']) or abs(t['error'])>.15:return 'TAG_NOT_CENTERED'
    if not math.isfinite(t['source_stamp']) or not -.05<=ros-t['source_stamp']<=.3:return 'TAG_SOURCE_STALE'
    status=inputs['driver']['status']
    if status!='READY' and (active or status!='COMMAND_TIMEOUT'):return 'DRIVER_'+status
    a=inputs['applied']
    if not math.isfinite(a['v']+a['w']):return 'APPLIED_COMMAND_INVALID'
    if not execute and (abs(a['v'])>1e-6 or abs(a['w'])>1e-6):return 'NONZERO_APPLIED_COMMAND'
    return None


class Session:
    def __init__(self,method,start_gap,execute,protocol,ekf_config,now,emit=None,on_stop=None):
        if method not in ('unfiltered','filtered'):raise ValueError('Unknown method.')
        if not .60<=start_gap<=2.5:raise ValueError('Measured start gap must be 0.60 to 2.50 m.')
        bounded={'max_speed_m_s':(0.,.16),'near_speed_cap_m_s':(0.,.06),
                 'max_yaw_rate_rad_s':(0.,.10),'kp_per_s':(0.,1.)}
        for key,(lo,hi) in bounded.items():
            if not math.isfinite(protocol[key]) or not lo<protocol[key]<=hi:
                raise ValueError('Unsafe or invalid protocol value: '+key)
        if protocol['target_physical_gap_m']!=.20 or not .23<=protocol['raw_tof_stop_m']<=.35:
            raise ValueError('Require 0.20 m physical target and conservative 0.23–0.35 m raw guard.')
        if not protocol['raw_tof_stop_m']<protocol['near_tof_m']<=.8:
            raise ValueError('Invalid near-box range.')
        for key in ['countdown_s','precheck_timeout_s','preview_duration_s','post_reference_timeout_s','settle_s','progress_tolerance_m']:
            if not math.isfinite(protocol[key]) or protocol[key]<=0:raise ValueError('Invalid timing/tolerance.')
        if protocol['countdown_s']<5 or protocol['precheck_timeout_s']>30:
            raise ValueError('Require five-second countdown and bounded precheck.')
        self.method=method;self.start_gap=start_gap;self.execute=execute
        self.protocol=dict(protocol);self.ekf_config=dict(ekf_config)
        self.started=now;self.inputs={};self.healthy_since=None
        self.motion_start=None;self.epoch=None;self.stop_time=None;self.stop_reason=None
        self.phase='WAITING' if execute else 'PREVIEW';self.done=False
        self.ekf=None;self.origin=None;self.r0=None;self.r0_variance=.002**2
        self.range_history=[];self.raw_x=self.raw_y=0.;self.scaled_tof=0.
        self.last_clock=None;self.clock_fault=False;self.preview_issues={}
        self.emit=emit or (lambda *args:None);self.on_stop=on_stop or (lambda:None)

    def active(self):return self.motion_start is not None and self.stop_time is None

    def check_clock(self,now,ros):
        if self.last_clock:
            pm,pr=self.last_clock;step=(ros-pr)-(now-pm)
            if abs(step)>.05:
                self.clock_fault=True;self.emit('CLOCK_JUMP',dict(step_s=step))
                if self.active():self.stop('CLOCK_JUMP',now)
        self.last_clock=(now,ros)

    def issue(self,now,ros,publishers):
        if self.clock_fault:return 'CLOCK_JUMP'
        return health(now,ros,self.inputs,publishers,self.execute,self.active(),self.protocol)

    def stop(self,reason,now):
        if self.stop_time is None:
            self.stop_time=now;self.stop_reason=reason;self.phase='SETTLING'
            self.emit('STOP',dict(reason=reason,motion_elapsed_s=None if self.motion_start is None else now-self.motion_start))
            self.on_stop()  # immediate zero, before any more filter processing

    def invalidate(self,key,now):
        self.inputs.pop(key,None);self.healthy_since=None
        if self.active():self.stop(key.upper()+'_MALFORMED',now)

    def predict(self,now):
        if self.ekf is not None and not self.clock_fault:
            a=self.inputs.get('applied') or dict(v=0.,w=0.)
            u=(a['v'],a['w']) if math.isfinite(a['v']+a['w']) else (0.,0.)
            self.ekf.predict_to(now-self.epoch,u)

    def initialize(self,now):
        o=self.inputs['odom'];q=o['quaternion']
        yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
        self.origin=(list(o['position']),yaw);self.r0=self.inputs['tof']['range']
        values=[r for t,r in self.range_history if now-t<=.8]
        if len(values)>1:
            import statistics
            self.r0_variance=max(.002**2,statistics.variance(values))
        self.epoch=now;self.ekf=EKF(self.ekf_config)
        self.raw_x=self.raw_y=self.scaled_tof=0.
        self.emit('MOTION_START' if self.execute else 'PREVIEW_ORIGIN',dict(
            physical_start_gap_m=self.start_gap,initial_odom=o,
            initial_tof=self.inputs['tof'],r0_variance_m2=self.r0_variance))

    def receive(self,key,item,now,ros,publishers):
        self.check_clock(now,ros)
        self.predict(now)  # propagate with previous applied command
        self.inputs[key]=item
        if key=='tof':
            self.range_history.append((now,item['range']))
            self.range_history=[p for p in self.range_history if now-p[0]<=1.]
        issue=self.issue(now,ros,publishers)
        if issue:
            self.healthy_since=None
            if self.active():self.stop(issue,now)
        if self.ekf is None or self.clock_fault:return
        t=now-self.epoch
        if key=='odom':
            pos=relative_pose(item['position'],item['quaternion'],*self.origin)
            self.raw_x,self.raw_y=float(pos[0]),float(pos[1])
            age=ros-item['source_stamp']
            self.ekf.observe_odom(pos[0],pos[1],t-age,age)
        elif key=='tof':
            age=item['sensor_age'];self.scaled_tof=self.ekf_config['tof_travel_scale']*(self.r0-item['range'])
            if .10<=item['range']<=10:self.ekf.observe_tof(self.scaled_tof,t-age,age,self.r0_variance)

    def tick(self,now,ros,publishers):
        self.check_clock(now,ros);self.predict(now)
        issue=self.issue(now,ros,publishers)
        status=issue or 'HEALTHY';v=w=0.
        if self.stop_time is not None:
            if now-self.stop_time>=self.protocol['settle_s']:self.done=True
            return self.details(now,status,v,w)
        if not self.execute:
            if issue:
                self.healthy_since=None;self.preview_issues[issue]=self.preview_issues.get(issue,0)+1
            elif self.healthy_since is None:self.healthy_since=now
            if self.ekf is None and self.healthy_since is not None and now-self.healthy_since>=1.:self.initialize(now)
            if now-self.started>=self.protocol['preview_duration_s']:
                self.done=True;self.stop_reason='PREVIEW_'+status
            return self.details(now,status,v,w)
        if self.motion_start is None:
            if issue:self.healthy_since=None
            elif self.healthy_since is None:self.healthy_since=now
            if now-self.started>=self.protocol['precheck_timeout_s']:
                self.stop('PRECHECK_'+(issue or 'NO_CONTINUOUS_HEALTHY_WINDOW'),now)
                return self.details(now,status,v,w)
            if self.healthy_since is not None:
                self.phase='COUNTDOWN'
                if now-self.healthy_since>=self.protocol['countdown_s']:
                    self.motion_start=now;self.phase='MOVING';self.initialize(now)
            if self.motion_start is None:return self.details(now,status,v,w)
        if issue:self.stop(issue,now);return self.details(now,status,v,w)
        elapsed=now-self.motion_start;ref=reference(elapsed,self.start_gap)
        progress=self.raw_x if self.method=='unfiltered' else float(self.ekf.state[0])
        if elapsed>=ref['T']+self.protocol['post_reference_timeout_s']:
            self.stop('BOUNDED_RUNTIME',now)
        elif elapsed>=ref['T'] and progress>=ref['L']-self.protocol['progress_tolerance_m']:
            self.stop('REFERENCE_REACHED',now)
        else:
            v,w=feedback(ref,progress,self.inputs['tag']['error'],self.inputs['tof']['range'],self.protocol)
        return self.details(now,status,v,w)

    def details(self,now,status,v,w):
        elapsed=0. if self.motion_start is None else now-self.motion_start
        ref=reference(elapsed,self.start_gap)
        data=dict(phase=self.phase,status=status,method=self.method,
            motion_elapsed_s=None if self.motion_start is None else elapsed,
            reference_x_m=ref['x'],reference_v_m_s=ref['v'],reference_a_m_s2=ref['a'],
            reference_T_s=ref['T'],reference_L_m=ref['L'],
            requested_v_m_s=v,requested_omega_rad_s=w,stop_reason=self.stop_reason,
            raw_odom_progress_m=None if self.ekf is None else self.raw_x,
            scaled_tof_travel_m=None if self.ekf is None else self.scaled_tof)
        if self.ekf is not None:
            ramp=min(1.,max(0.,(now-self.epoch)/self.ekf_config['bias_ramp_s']))
            data.update(corrected_odom_progress_m=self.raw_x-self.ekf_config['odometry_bias_m']*ramp,
                filtered_progress_m=float(self.ekf.state[0]),ekf_state=self.ekf.state.tolist(),
                covariance=self.ekf.P.tolist(),ekf_counts=self.ekf.counts.copy())
        if self.execute and self.motion_start is None:
            data['countdown_remaining_s']=None if self.healthy_since is None else max(0.,self.protocol['countdown_s']-(now-self.healthy_since))
        return data
