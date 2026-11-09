# Five-state model and observation equations

All positions are in metres in the run-start frame; heading is radians, forward body speed is m/s, yaw rate is rad/s. The origin is the initial odometry position and heading. Project global odometry displacement onto that heading, making forward travel +x and lateral displacement +y.

State `s=[x,y,theta,v,omega]^T`; input `u=[u_v,u_omega]^T` is the driver's applied command, not the requested command if a guard intervenes. Raw SDK twist is logged for comparison but not fused as independent evidence.

For timestep dt, `a_v=1-exp(-dt/tau_v)` and `a_w=1-exp(-dt/tau_w)`:

```text
x_next     = x + v*cos(theta)*dt
y_next     = y + v*sin(theta)*dt
theta_next = wrap(theta + omega*dt)
v_next     = (1-a_v)*v + a_v*u_v
omega_next = (1-a_w)*omega + a_w*u_omega
```

The 0.25 s response constants are assumed model priors; the pilots did not identify them from a physical continuous trajectory. Predictions are subdivided into steps no longer than 0.02 s.

The nonzero F entries are:

| Entry | Value |
|---|---|
| F00, F11, F22 | 1 |
| F02 | -v sin(theta) dt |
| F03 | cos(theta) dt |
| F12 | v cos(theta) dt |
| F13 | sin(theta) dt |
| F24 | dt |
| F33 | 1-a_v |
| F44 | 1-a_w |

The deterministic input Jacobian B has B30=a_v and B41=a_w. G maps independent white diffusion in speed and yaw rate, not an extra independent odometry sensor: G30=sqrt(dt), G41=sqrt(dt), all other entries zero. Q is diagonal in velocity/yaw-rate diffusion spectral amplitudes squared. Therefore `P_minus=F P F^T + G Q G^T`; direct speed and turn uncertainty propagates into pose through later predictions. The continuous-diffusion choice scales uncertainty with elapsed time rather than counting callbacks as independent time steps.

The velocity diffusion amplitude is `max(0.01, RMS(odom_middle_slope - requested_speed))/sqrt(0.25)` from the five pilots. The floor is an explicit conservative tuning choice. Yaw-rate diffusion is 0.06 rad/s/sqrt(s), a declared prior. Initial standard deviations are `[0.01 m,0.01 m,5 degrees,0.02 m/s,0.05 rad/s]`.

Odometry observation: `z_o=[x_raw - b_o min(1,max(0,t/2)), y_raw]^T`, `h_o(s)=[x,y]^T`. H_o selects x and y. Bias `b_o=median(x_odom_settled - physical_travel)`; the startup ramp is a modeling assumption. Odometry R_x is four times the across-pilot sample variance of corrected endpoint errors, floored using a 2 mm SD before inflation. R_y=R_x is an assumption, not lateral ground-truth calibration. The two components originate from one odometry system, not two independent sensors.

ToF observation: `z_T=s_T*(r0-r)` with `s_T=median(physical_travel/(r0-rf))`; `h_T(s)=x` and H_T selects x. R_T is four times the across-pilot corrected ToF endpoint-error sample variance, with the same floor, plus `s_T^2 Var(r0)` estimated from prestart range spread (floor 2 mm SD). The additional term is conservative bookkeeping; it does not model the off-diagonal temporal covariance caused by common r0. Sequential odometry/ToF corrections assume independent contemporaneous observation noise; correlated errors may invalidate that approximation.

The gain is `K=P_minus H^T (H P_minus H^T+R)^-1`. State correction is `s_plus=s_minus+K(z-h(s_minus))`, with heading wrapped. Covariance uses the numerically stable Joseph form `(I-KH)P_minus(I-KH)^T+K R K^T`, followed by symmetry enforcement. No artificial clipping of x or v is used to make replay errors look smaller.

Each sensor's corrections are separated by at least 0.25 s; duplicate source times and source ages outside [-0.05,0.40] s are rejected by the core. Live tag age is limited to 0.30 s and driver/odometry/ToF receipt age to 0.40 s. The live preview checks a clock-difference discontinuity and blocks after a step larger than 0.05 s. These are preview supervision rules, not permission to disable independent motor guards.

The prototype is not a validated uncertainty model. Physical position is known only at the measured endpoints. Pilot-derived noise proxies, assumed response constants, short paths, unmodeled ToF projection during turns, observation latency, and temporal/cross-sensor correlation must be disclosed. Evaluate whether the EKF beats bias-corrected odometry on the same independent filtered evaluation runs only after calibration and controller freeze.
