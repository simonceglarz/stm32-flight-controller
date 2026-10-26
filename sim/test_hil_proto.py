"""Check the Python protocol against the C protocol in libcore.so.
If these pass, Python and the board (same C code) will understand each other."""
import ctypes
import os

import hil_proto as hp

# Load the compiled C library (built by `make sim`)
lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), "build", "libcore.so"))

# Tell ctypes the C function signatures (otherwise it guesses "int" and garbles data)
lib.hil_crc16.argtypes = [ctypes.c_char_p, ctypes.c_size_t]
lib.hil_crc16.restype = ctypes.c_uint16


# Mirror the C structs so Python can hand them to the C encoders.
# ctypes lays fields out in memory the same way the C compiler does.
class CSensor(ctypes.Structure):
    _fields_ = [("seq", ctypes.c_uint16), ("angle_sp_deg", ctypes.c_float),
                ("angle_deg", ctypes.c_float), ("rate_dps", ctypes.c_float)]


class CCmd(ctypes.Structure):
    _fields_ = [("seq", ctypes.c_uint16), ("cmd", ctypes.c_float),
                ("status", ctypes.c_uint8)]


FRAME_BUF = ctypes.c_uint8 * 24                   # a C array big enough for any frame (max 24)
lib.hil_encode_sensor.argtypes = [ctypes.POINTER(CSensor), FRAME_BUF]
lib.hil_encode_sensor.restype = ctypes.c_size_t
lib.hil_encode_cmd.argtypes = [ctypes.POINTER(CCmd), FRAME_BUF]
lib.hil_encode_cmd.restype = ctypes.c_size_t

fails = 0


def check(name, ok):
    global fails
    print(("PASS" if ok else "FAIL") + ": " + name)
    fails += 0 if ok else 1


# 1. Python CRC gives the official check value
check("py crc check value 0x29B1", hp.crc16(b"123456789") == 0x29B1)

# 2. Python CRC == C CRC on some varied data
data = bytes(range(256)) * 2
check("py crc == C crc", hp.crc16(data) == lib.hil_crc16(data, len(data)))

# 3. A SENSOR frame built by Python is byte-identical to one built by C
buf = FRAME_BUF()
n = lib.hil_encode_sensor(ctypes.byref(CSensor(513, 10.0, -3.25, 42.5)), buf)
c_bytes = bytes(buf[:n])
py_bytes = hp.encode_sensor(513, 10.0, -3.25, 42.5)
check("sensor frame identical (py vs C)", py_bytes == c_bytes)
print("   frame bytes:", py_bytes.hex(" "))           # so you can see the layout

# 4. The Python parser correctly reads a CMD frame built by C,
#    exactly what will happen with the board's replies
n = lib.hil_encode_cmd(ctypes.byref(CCmd(41, -0.55, hp.STATUS_OK)), buf)
p = hp.Parser()
result = None
for b in bytes(buf[:n]):
    r = p.feed(b)
    if r:
        result = r
check("py parses C cmd frame", result is not None and result["seq"] == 41
      and abs(result["cmd"] - (-0.55)) < 1e-6 and result["status"] == hp.STATUS_OK)

print("\nALL PASSED" if fails == 0 else f"\nFAILURES: {fails}")