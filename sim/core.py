"""Python access to the firmware;s PID controller (core/pid.c via ctypes."""
import ctypes
import os

_LIB_PATH = os.path.join(os.path.dirname(__file__), "build", "libcore.so")
_lib = ctypes.CDLL(_LIB_PATH)

# Declare argument/return types. Without these, ctypes assumes int,
# and floats would be passed as garbage.
_f = ctypes.c_float
_ptr = ctypes.c_void_p                       # opaque pointer to pid_ctrl_t

_lib.pid_shim_new.argtypes = [_f] * 8
_lib.pid_shim_new.restype = _ptr
_lib.pid_shim_update.argtypes = [_ptr, _f, _f, _f]
_lib.pid_shim_update.restype = _f
_lib.pid_shim_reset.argtypes = [_ptr]
_lib.pid_shim_reset.restype = None
_lib.pid_shim_free.argtypes = [_ptr]
_lib.pid_shim_free.restype = None

class Pid:
    """One PID controller, running the exact C code used on the STM32."""

    def __init__(self, kp, ki, kd, out_min, out_max,
                 integ_min=None, integ_max=None, d_filter_alpha=0.0):
        # Default integrator limits to the output limits if not given
        integ_min = out_min if integ_min is None else integ_min
        integ_max = out_max if integ_max is None else integ_max
        self._p = _lib.pid_shim_new(kp, ki, kd, out_min, out_max,
                                    integ_min, integ_max, d_filter_alpha)
        if not self._p:
            raise MemoryError("pid_shim_new returned NULL")

    def update(self, setpoint, measurement, dt):
        return _lib.pid_shim_update(self._p, setpoint, measurement, dt)

    def reset(self):
        _lib.pid_shim_reset(self._p)

    def __del__(self):
        if getattr(self, "_p", None):
            _lib.pid_shim_free(self._p)
            self._p = None