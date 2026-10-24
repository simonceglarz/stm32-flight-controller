# Requirements

v0 written before any measurement. Changes since are traced below.

| ID | Requirement | Verification method | Pass criterion |
|---|---|---|---|
| REQ-01 | Control task runs once per IMU data-ready event (1.66 kHz nominal ODR) | Logic analyzer: PC7 (DRDY) vs PB10 (loop pin) | Data-ready-to-task latency jitter ≤ 50 µs, no dropped cycles. **Met (2026-10-24)**, see evidence below |
| REQ-02 | If no new IMU sample arrives for 5 consecutive cycles, motors are commanded to zero within 10 ms and state → FAULT | HIL fault injection; then IMU disconnected on hardware | Logged time ≤ 10 ms |
| REQ-03 | Motors cannot spin unless throttle is at minimum, arm button held ≥ 1 s, and no fault is active | HIL + bench test matrix | All illegal sequences rejected |
| REQ-04 | A hung task causes MCU reset within 100 ms; motors stay off through reset | Deliberate task stall | Reset observed; no motor output |
| REQ-05 | If the command link is silent > 500 ms while armed, the system disarms | Link disconnected mid-run | Disarm logged |
| REQ-06 | A 10° attitude step settles to ±1° within T ms with ≤ 20% overshoot | HIL first, then thrust rig | T fixed after first HIL run, then not changed |

## Change log

- REQ-01: rate changed from 1 kHz to 1.66 kHz (IMU offers 833 Hz or 1.66 kHz only; DS12814 Tables 51/55).
- REQ-01: worded as "once per data-ready event" because the actual rate is set by the IMU's oscillator (measured 1.608 kHz on this unit).

## REQ-01 evidence (2026-10-24)

| Measurement | Result | Method |
|---|---|---|
| Loop period | 622.04 µs (1607.6 Hz), no dropped cycles | PulseView Timing decoder on PB10, rising edges |
| IMU actual ODR vs nominal | −3.48 % measured; INTERNAL_FREQ_FINE = −24 → −3.60 % (DS12814 Table 174) | Agree within one register LSB (0.15 %) |
| Data-ready → task latency | 8.3–9.3 µs (jitter ≈ 1 µs) | PulseView Jitter decoder, PC7 → PB10 |
| Loop execution time | ≈ 44 µs (≈ 7 % of period) | PB10 high time |

Captures: `docs/captures/drdy_latency.png`, `docs/captures/loop_timing_after_wide.png`

Notes:
- The filter's dt is derived from INTERNAL_FREQ_FINE at boot.
- A 3.5 ms logging stall (newlib stdout lock held during UART output in the telemetry task) previously dropped ~5 samples per telemetry line. Fixed by formatting with snprintf and sending with HAL_UART_Transmit (`docs/captures/logging_stall_before.png`).