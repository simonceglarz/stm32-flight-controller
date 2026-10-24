"""Check that Python gets the same answers as the C host tests."""
from core import Pid

# P only: kp=2, setpoint 10, measurement 4 -> error 6 -> output 12
p = Pid(kp=2.0, ki=0.0, kd=0.0, out_min=-100.0, out_max=100.0)
out = p.update(10.0, 4.0, 0.01)
print(f"P term: {out:.3f} (expected 12.000)")
assert abs(out - 12.0) < 1e-5

# Output clamp: huge error must saturate at out_max
out = p.update(1000.0, 0.0, 0.01)
print(f"clamp:  {out:.3f} (expected 100.000)")
assert abs(out - 100.0) < 1e-5

print("ctypes bridge OK")