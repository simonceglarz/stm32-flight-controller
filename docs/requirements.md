# Requirements — v0 (written before any measurement)

| ID | Requirement | Verification method | Pass criterion |
|---|---|---|---|
| REQ-01 | Control task runs at 1.66 kHz (IMU data-ready driven); worst-case period jitter ≤ 50 µs over 10 min | GPIO toggle per cycle, logic-analyzer capture | Measured max jitter ≤ 50 µs |
| REQ-02 | If no new IMU sample arrives for 5 consecutive cycles, motors are commanded to zero within 10 ms and state → FAULT | HIL fault injection; then IMU disconnected on hardware | Logged time ≤ 10 ms |
| REQ-03 | Motors cannot spin unless throttle is at minimum, arm button held ≥ 1 s, and no fault is active | HIL + bench test matrix | All illegal sequences rejected |
| REQ-04 | A hung task causes MCU reset within 100 ms; motors stay off through reset | Deliberate task stall | Reset observed; no motor output |
| REQ-05 | If the command link is silent > 500 ms while armed, the system disarms | Link disconnected mid-run | Disarm logged |
| REQ-06 | A 10° attitude step settles to ±1° within T ms with ≤ 20% overshoot | HIL first, then thrust rig | T fixed after first HIL run, then not changed |

Numbers are initial targets. Any change will be a separate commit with the reason in the commit message.
