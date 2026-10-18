# STM32 Flight Controller
Quadrotor flight-control firmware written from scratch on an STM32F446RE under FreeRTOS
**Status:** in development - board bring-up.

## Structure
-core/` portable C control code (PID, attitude filter), unti-tests on host
- `tests/` — host-side unit tests for `core/`
- `firmware/` — STM32CubeIDE project
- `sim/` — Python plant model and hardware-in-the-loop harness
- `docs/` — requirements, test reports, logic-analyzer captures
