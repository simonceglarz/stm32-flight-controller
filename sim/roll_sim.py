"""Single-axis (roll) software-in-the-loop simulation.

Plant: seesaw roll axis, two motors at +/-L from the pivot, first-order motor lag.
Controller: cascaded PID using the firmware's own core/pid.c (via ctypes).
Sensor noise and loop rate are the values measured on the real hardware.

PLANT PARAMETERS ARE PLACEHOLDER ESTIMATES (no motors/frame yet). Results show the
controller and harness work; they are NOT a prediction of the real drone.
"""

import math
import os
import random
import sys

import matplotlib.pyplot as plt

from core import Pid

#Plant aparemeteres (estimates, will be replace when real hardware is measured)
J = 1.5e-4  #roll moment of inertia, kg*m^2 (estimate: ~150g, 3-inch class frame)
L = 0.05    #distance-to-motor distance, m (estimate: ~100mm motor spacing)
DT_MAX = 0.5  #max time step, s 
TAU_M = 0.03   #motor/prop thrust time cosnstant. s (estimate)

# Values MEASURED on the real hardware (2026/10/24)
DT = 1.0 / 1608.0       #control period, s measured form hardware analyzer: 622.04us)
ANGLE_NOISE_DEG = 0.015 #fitered angle noise, deg RMS (tools/rms.py)
GYRO_NOISE_DPS = 0.25   #gyro noise, deg/s RMS (estimated from calibration data)

SUBSTEPS = 10  #physics staps per control step (smaller h = more accurate integration)

#Test definition (REQ-06)
STEP_DEG = 10.0     # step size
STEP_TIME = 0.1     # s, when the step is applied
SIM_TIME = 1.5      # s, total
DIST_TIME = 0.8     #s, when the disturbance starts
DIST_TORQUE = 0.01  #N*m, constant (2-% of max torque 2*L*DT_MAX = 0.05N*m)
SETTLE_BAND = 1.0   # deg, REQ-06 band
MAX_OVERSHOOT = 20.0 #%, REQ-06 limit

def simulate(kp_angle, kp_rate, ki_rate, kd_rate, d_alpha=0.0, seed=1):
    """Run one 10-degree step. Returns lists of time, setpoint, angle, command."""
    random.seed(seed)    # fixed seed = repeatable noise, so gain changes are compared fairly
    
    # Outer loop: angle error (deg) -> rate setpoint (deg/s). P only.
    angle_pid = Pid(kp_angle, 0.0, 0.0, out_min=-500.0, out_max=500.0)
    # Inner loop: rate error (deg/s) -> motor command (-1..1). Integrator limited to +/-0.3.
    rate_pid = Pid(kp_rate, ki_rate, kd_rate, out_min=-1.0, out_max=1.0,
                   integ_min=-0.3, integ_max=0.3, d_filter_alpha=d_alpha)

    theta = 0.0      # true angle, rad
    omega = 0.0      # true rate, rad/s
    u_actual = 0.0   # motor command after lag (what the props are really doing)

    t_log, sp_log, ang_log, u_log = [], [], [], []
    h = DT / SUBSTEPS

    for k in range(int(SIM_TIME / DT)):
        t = k * DT
        setpoint = STEP_DEG if t >= STEP_TIME else 0.0

        # Sensors: true state + measured noise levels
        angle_meas = math.degrees(theta) + random.gauss(0.0, ANGLE_NOISE_DEG)
        rate_meas = math.degrees(omega) + random.gauss(0.0, GYRO_NOISE_DPS)

         # Controller: the firmware's C code
        rate_sp = angle_pid.update(setpoint, angle_meas, DT)
        u = rate_pid.update(rate_sp, rate_meas, DT)

        for _ in range(SUBSTEPS):
            u_actual += (u - u_actual) * h / TAU_M     # first-order motor lag
            torque = 2.0 * L * DT_MAX * u_actual       # T1 = +u*DT_MAX, T2 = -u*DT_MAX
            if t >= DIST_TIME:
                torque += DIST_TORQUE                  # e.g. off-centre battery
            
            omega += torque / J * h                    # angular acceleration
            theta += omega * h

        t_log.append(t)
        sp_log.append(setpoint)
        ang_log.append(math.degrees(theta))
        u_log.append(u)

    return t_log, sp_log, ang_log, u_log

def step_metrics(t_log, ang_log):
    """Overshoot (%) and settling time (s) into +/-SETTLE_BAND after the step."""
    after = [(t, a) for t, a in zip(t_log, ang_log) if STEP_TIME <= t < DIST_TIME]
    peak = max(a for _, a in after)
    overshoot = max(0.0, (peak - STEP_DEG) / STEP_DEG * 100.0)
    settle = None
    for i in range(len(after)):
        if all(abs(a - STEP_DEG) <= SETTLE_BAND for _, a in after[i:]):
            settle = after[i][0] - STEP_TIME
            break
    return overshoot, settle

if __name__ == "__main__":
    # Gains: python3 roll_sim.py [kp_angle kp_rate ki_rate kd_rate]
    args = [float(x) for x in sys.argv[1:]]
    gains = args[:4] if len(args) >= 4 else [8.0, 0.004, 0.0, 0.0]
    d_alpha = args[4] if len(args) >= 5 else 0.0
    t, sp, ang, u = simulate(*gains, d_alpha=d_alpha)
    os_pct, settle = step_metrics(t, ang)

    # Motor command noise once settled (last 0.3 s): what the motors would "feel"
    tail = [x for ti, x in zip(t, u) if ti >= SIM_TIME - 0.3]
    mean_u = sum(tail) / len(tail)
    u_noise = (sum((x - mean_u) ** 2 for x in tail) / len(tail)) ** 0.5

    end = [a for ti, a in zip(t, ang) if ti >= SIM_TIME - 0.2]
    ss_err = sum(end) / len(end) - STEP_DEG

    settle_txt = f"{settle * 1000:.0f} ms" if settle is not None else "did not settle"
    verdict = "PASS" if settle is not None and os_pct <= MAX_OVERSHOOT else "FAIL"
    print(f"gains (kp_a, kp_r, ki_r, kd_r) = {gains}")
    print(f"overshoot {os_pct:.1f} %  |  settle: {settle_txt}  |  cmd noise {u_noise:.4f} RMS  |  {verdict}")
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    ax1.plot(t, sp, "--", label="setpoint")
    ax1.plot(t, ang, label="roll angle")
    ax1.axhspan(STEP_DEG - SETTLE_BAND, STEP_DEG + SETTLE_BAND, alpha=0.15, label="±1° band")
    ax1.set_ylabel("angle (deg)")
    ax1.set_title(f"10° roll step (SIL, estimated plant): overshoot {os_pct:.1f}%, settle {settle_txt}")
    ax1.legend()
    ax1.grid(True)
    ax2.plot(t, u)
    ax2.set_ylabel("motor cmd (-1..1)")
    ax2.set_xlabel("time (s)")
    ax2.grid(True)
    fig.tight_layout()

    os.makedirs("results", exist_ok=True)
    out = "results/step_" + "_".join(f"{g:g}" for g in gains) + ".png"
    fig.savefig(out, dpi=120)
    print(f"saved {out}")
    plt.show()