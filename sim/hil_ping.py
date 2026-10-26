"""Send one SENSOR message to the board in HIL mode and check the reply.
Usage: python3 hil_ping.py [/dev/ttyACM1]"""
import sys
import time

import serial                      # pyserial

import hil_proto as hp

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"

with serial.Serial(PORT, 115200, timeout=0.5) as port:
    time.sleep(0.2)                # let the port settle
    port.reset_input_buffer()      # throw away anything left over (e.g. old boot text)

    # Message #0: setpoint 0 deg, angle 2 deg, rate 0 deg/s
    frame = hp.encode_sensor(0, 0.0, 2.0, 0.0)
    print("sent:    ", frame.hex(" "))
    t0 = time.perf_counter()
    port.write(frame)

    # Read bytes until the parser reports a complete reply (or we time out)
    parser = hp.Parser()
    reply = None
    deadline = time.time() + 1.0
    while reply is None and time.time() < deadline:
        data = port.read(1)        # waits up to 0.5 s for a byte
        if data:
            reply = parser.feed(data[0])
    rtt_ms = (time.perf_counter() - t0) * 1000

    if reply is None:
        print("NO REPLY: check HIL_MODE=1 is flashed, port name, and that screen is closed")
        sys.exit(1)

    expected = -0.1 - 0.1 * 20 / 1608
    print("reply:   ", reply)
    print(f"cmd {reply['cmd']:.6f}  expected {expected:.6f}  "
          f"{'PASS' if abs(reply['cmd'] - expected) < 1e-5 else 'FAIL'}")
    print(f"seq echoed: {'PASS' if reply['seq'] == 0 else 'FAIL'}   round trip {rtt_ms:.1f} ms")
    print(f"crc errors seen by PC: {parser.crc_errors}")