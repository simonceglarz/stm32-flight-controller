"""HIL message format, Python side. Must match core/hil_proto.c exactly.

Frame:  A5 5A | type | len | seq_lo seq_hi | payload (len bytes) | crc_lo crc_hi
CRC-16/CCITT-FALSE over type..payload. Everything little-endian.
See docs/hil-guide.md section 4 for the plain-language explanation.
"""
import struct

# ---- Constants: identical values to hil_proto.h ----
SYNC0 = 0xA5
SYNC1 = 0x5A
TYPE_SENSOR = 0x01        # PC -> board
TYPE_CMD = 0x02           # board -> PC
MAX_PAYLOAD = 16
SENSOR_LEN = 12           # 3 floats
CMD_LEN = 5               # 1 float + 1 status byte
STATUS_OK = 0
STATUS_FAULT = 1


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, the same recipe as hil_crc16() in C.
    "123456789" must give 0x29B1."""
    crc = 0xFFFF                         # standard starting value
    for byte in data:                    # for every byte...
        crc ^= byte << 8                 # mix it into the top 8 bits
        for _ in range(8):               # ...process its 8 bits
            if crc & 0x8000:             # top bit is 1:
                crc = (crc << 1) ^ 0x1021    # shift and mix in the constant
            else:
                crc <<= 1                    # just shift
            crc &= 0xFFFF                # Python ints never overflow, so trim back to 16 bits
                                         # (C's uint16_t does this automatically)
    return crc


def _frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    """Wrap a payload in the envelope: start marker, header, CRC.
    Same layout as frame() in C."""
    # Header: type (1 byte), len (1 byte), seq (2 bytes, low first): "<BBH"
    body = struct.pack("<BBH", msg_type, len(payload), seq & 0xFFFF) + payload
    crc = crc16(body)                                    # over type..payload
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc)


def encode_sensor(seq: int, angle_sp_deg: float, angle_deg: float, rate_dps: float) -> bytes:
    """Build a SENSOR frame (what the PC sends every step). 20 bytes."""
    payload = struct.pack("<fff", angle_sp_deg, angle_deg, rate_dps)   # 3 floats, 12 bytes
    return _frame(TYPE_SENSOR, seq, payload)


def encode_cmd(seq: int, cmd: float, status: int) -> bytes:
    """Build a CMD frame. The board normally does this; Python has it for tests."""
    payload = struct.pack("<fB", cmd, status)                          # float + byte, 5 bytes
    return _frame(TYPE_CMD, seq, payload)


class Parser:
    """Byte-at-a-time receiver, the same state machine as hil_parser_feed() in C.

    feed(byte) returns None until a complete, undamaged frame arrives, then
    returns a dict with the decoded values, for example:
        {"type": 2, "seq": 41, "cmd": -0.55, "status": 0}
    """

    # The five states, same order and meaning as the C enum
    SYNC0_, SYNC1_, BODY, CRC0, CRC1 = range(5)

    def __init__(self):
        self.state = self.SYNC0_
        self.body = bytearray()       # saved bytes: type, len, seq_lo, seq_hi, payload
        self.need = 4                 # body bytes expected in total
        self.crc_lo = 0
        self.crc_errors = 0           # damaged frames thrown away

    def feed(self, byte: int):
        if self.state == self.SYNC0_:                 # looking for A5
            if byte == SYNC0:
                self.state = self.SYNC1_

        elif self.state == self.SYNC1_:               # got A5, expecting 5A
            if byte == SYNC1:
                self.body = bytearray()
                self.need = 4                         # header first
                self.state = self.BODY
            elif byte != SYNC0:                       # A5 again: stay; otherwise start over
                self.state = self.SYNC0_

        elif self.state == self.BODY:                 # saving type, len, seq, payload
            self.body.append(byte)
            if len(self.body) == 4:                   # header complete: read len
                if self.body[1] > MAX_PAYLOAD:        # impossible length: not a real frame
                    self.state = self.SYNC0_
                    return None
                self.need = 4 + self.body[1]
            if len(self.body) == self.need:
                self.state = self.CRC0

        elif self.state == self.CRC0:                 # CRC low byte
            self.crc_lo = byte
            self.state = self.CRC1

        elif self.state == self.CRC1:                 # CRC high byte: decide
            self.state = self.SYNC0_
            received = self.crc_lo | (byte << 8)
            if received != crc16(bytes(self.body)):
                self.crc_errors += 1                  # damaged: count and discard
                return None
            return self._decode()                     # good frame: return its values

        return None

    def _decode(self):
        """Turn the saved body bytes into a dict of values."""
        msg_type, length, seq = struct.unpack_from("<BBH", self.body, 0)
        payload = bytes(self.body[4:])
        if msg_type == TYPE_CMD and length == CMD_LEN:
            cmd, status = struct.unpack("<fB", payload)
            return {"type": msg_type, "seq": seq, "cmd": cmd, "status": status}
        if msg_type == TYPE_SENSOR and length == SENSOR_LEN:
            sp, ang, rate = struct.unpack("<fff", payload)
            return {"type": msg_type, "seq": seq, "angle_sp_deg": sp,
                    "angle_deg": ang, "rate_dps": rate}
        return None                                   # valid CRC but unknown kind: ignore