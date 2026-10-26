"""Hardware-in-the-loop run: the STM32 flies the simulated roll axis.

Each step (lock-step):
  1. physics gives the true angle/rate; add the measured sensor noise
  2. send a SENSOR message to the board, wait for its CMD reply
  3. apply the board's motor command to the physics for one step

The same test is then run in SIL (controller on the PC) with identical gains
and identical noise, and the two are compared.

Usage:  python3 hil_run.py [/dev/ttyACM1]
Board must be flashed with HIL_MODE 1. Close `screen` first.
"""
import math
import os
import random
import sys
import time

import matplotlib
matplotlib.use("Agg")              # save plots to files only, never open a window
import matplotlib.pyplot as plt
import serial

import hil_proto as hp
import roll_sim as rs              # reuse the plant constants, test settings and metrics

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"

# Must match ROLL_GAINS_DEFAULT in core/roll_ctrl.c (the board uses those).
# The SIL comparison run uses these, so if they differ the comparison is meaningless.
GAINS = [10.0, 0.005, 0.1, 0.0001]   # kp_angle, kp_rate, ki_rate, kd_rate
D_ALPHA = 0.95

REPLY_TIMEOUT_S = 0.2                # give up if the board doesn't answer within 200 ms


def exchange(port, parser, seq, setpoint, angle, rate):
    """Send one SENSOR message and wait for the matching CMD reply.
    Returns the reply dict. Raises an error on timeout or a mismatched number."""
    port.write(hp.encode_sensor(seq, setpoint, angle, rate))
    deadline = time.perf_counter() + REPLY_TIMEOUT_S
    while time.perf_counter() < deadline:
        data = port.read(port.in_waiting or 1)       # whatever has arrived (at least 1 byte)
        for byte in data:
            reply = parser.feed(byte)
            if reply and reply["type"] == hp.TYPE_CMD:
                if reply["seq"] != seq:              # lock-step broken: a message was lost
                    raise RuntimeError(f"sent #{seq}, got reply #{reply['seq']}")
                return reply
    raise TimeoutError(f"no reply to message #{seq}")


def run_hil(port):
    """Same structure as rs.simulate(), but the controller is the board.
    The physics lines are copied from roll_sim.py: keep them identical."""
    random.seed(1)                                   # same noise as SIL (seed 1)
    parser = hp.Parser()

    theta = omega = u_actual = 0.0                   # true angle (rad), rate (rad/s), lagged cmd
    h = rs.DT / rs.SUBSTEPS
    t_log, sp_log, ang_log, u_log, rtt_log = [], [], [], [], []

    for k in range(int(rs.SIM_TIME / rs.DT)):
        t = k * rs.DT
        setpoint = rs.STEP_DEG if t >= rs.STEP_TIME else 0.0

        # Sensors: true state + measured noise (same order of random calls as SIL)
        angle_meas = math.degrees(theta) + random.gauss(0.0, rs.ANGLE_NOISE_DEG)
        rate_meas = math.degrees(omega) + random.gauss(0.0, rs.GYRO_NOISE_DPS)

        # Controller: ask the board. seq = k (message #0 resets the board's controller)
        t0 = time.perf_counter()
        reply = exchange(port, parser, k & 0xFFFF, setpoint, angle_meas, rate_meas)
        rtt_log.append((time.perf_counter() - t0) * 1000.0)
        u = reply["cmd"]

        # Plant: identical to roll_sim.simulate()
        for _ in range(rs.SUBSTEPS):
            u_actual += (u - u_actual) * h / rs.TAU_M
            torque = 2.0 * rs.L * rs.DT_MAX * u_actual
            if t >= rs.DIST_TIME:
                torque += rs.DIST_TORQUE
            omega += torque / rs.J * h
            theta += omega * h

        t_log.append(t)
        sp_log.append(setpoint)
        ang_log.append(math.degrees(theta))
        u_log.append(u)

    return t_log, sp_log, ang_log, u_log, rtt_log, parser.crc_errors


if __name__ == "__main__":
    # ---- HIL: the board flies ----
    with serial.Serial(PORT, 115200, timeout=REPLY_TIMEOUT_S) as port:
        time.sleep(0.2)
        port.reset_input_buffer()
        wall0 = time.time()
        t, sp, ang_hil, u_hil, rtt, crc_err = run_hil(port)
        wall = time.time() - wall0

    # ---- SIL: same test, controller on the PC ----
    _, _, ang_sil, u_sil = rs.simulate(*GAINS, d_alpha=D_ALPHA)

    # ---- Compare ----
    os_hil, settle_hil = rs.step_metrics(t, ang_hil)
    os_sil, settle_sil = rs.step_metrics(t, ang_sil)
    max_ang_diff = max(abs(a - b) for a, b in zip(ang_hil, ang_sil))
    max_cmd_diff = max(abs(a - b) for a, b in zip(u_hil, u_sil))

    def fmt(settle):
        return f"{settle * 1000:.0f} ms" if settle is not None else "did not settle"

    print(f"steps: {len(t)}   wall time: {wall:.1f} s   "
          f"round trip: mean {sum(rtt) / len(rtt):.2f} ms, max {max(rtt):.2f} ms   "
          f"CRC errors: {crc_err}")
    print(f"HIL: overshoot {os_hil:.1f} %, settle {fmt(settle_hil)}")
    print(f"SIL: overshoot {os_sil:.1f} %, settle {fmt(settle_sil)}")
    print(f"max |angle HIL - SIL| = {max_ang_diff:.2e} deg   "
          f"max |cmd HIL - SIL| = {max_cmd_diff:.2e}")

    # ---- Plot: HIL and SIL on top of each other, plus the difference ----
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    ax1.plot(t, sp, "--", label="setpoint")
    ax1.plot(t, ang_sil, label="SIL (PC)", linewidth=3, alpha=0.5)
    ax1.plot(t, ang_hil, label="HIL (STM32)", linewidth=1)
    ax1.set_ylabel("angle (deg)")
    ax1.set_title("HIL vs SIL: same controller code, PC vs STM32")
    ax1.legend()
    ax1.grid(True)
    ax2.plot(t, u_sil, label="SIL", linewidth=3, alpha=0.5)
    ax2.plot(t, u_hil, label="HIL", linewidth=1)
    ax2.set_ylabel("motor cmd")
    ax2.legend()
    ax2.grid(True)
    ax3.plot(t, [a - b for a, b in zip(ang_hil, ang_sil)])
    ax3.set_ylabel("angle diff (deg)")
    ax3.set_xlabel("time (s)")
    ax3.grid(True)
    fig.tight_layout()
    os.makedirs("results", exist_ok=True)
    fig.savefig("results/hil_vs_sil.png", dpi=120)
    plt.close(fig)
    print("saved results/hil_vs_sil.png")