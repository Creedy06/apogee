"""Binary telemetry packets with CRC-16 error checking.

Each packet is 18 bytes, little-endian:

    offset  size  field
    0       2     sync word 0xA55A
    2       2     sequence number
    4       4     time in milliseconds since launch
    8       4     barometric altitude, meters (float32)
    12      2     vertical acceleration, milli-g (int16)
    14      2     status bits (bit0 armed, bit1 drogue, bit2 main chute)
    16      2     CRC-16/CCITT-FALSE of bytes 0..15
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

SYNC = 0xA55A
_BODY = struct.Struct("<HHIfhH")
PACKET_SIZE = _BODY.size + 2  # 18

ARMED, DROGUE, MAIN = 1, 2, 4


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021), the checksum many radios and flight computers use."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


@dataclass
class Packet:
    seq: int
    t_ms: int
    altitude_m: float
    accel_mg: int
    status: int

    def encode(self) -> bytes:
        body = _BODY.pack(SYNC, self.seq & 0xFFFF, self.t_ms, self.altitude_m, self.accel_mg, self.status)
        return body + struct.pack("<H", crc16(body))


@dataclass
class DecodeStats:
    good: int = 0
    bad_crc: int = 0
    resyncs: int = 0
    dropped: int = 0   # gaps in sequence numbers


def decode_stream(stream: bytes, stats: DecodeStats | None = None) -> list[Packet]:
    """Pull every valid packet out of a raw byte stream.

    Handles what a real radio link throws at you: noise between packets
    (scans for the sync word), corrupted packets (CRC check), and lost
    packets (sequence-number gaps).
    """
    stats = stats if stats is not None else DecodeStats()
    out: list[Packet] = []
    sync = struct.pack("<H", SYNC)
    i = 0
    last_seq = None
    while i + PACKET_SIZE <= len(stream):
        if stream[i:i + 2] != sync:
            nxt = stream.find(sync, i + 1)
            if nxt == -1:
                break
            stats.resyncs += 1
            i = nxt
            continue
        chunk = stream[i:i + PACKET_SIZE]
        body, (crc,) = chunk[:-2], struct.unpack("<H", chunk[-2:])
        if crc16(body) != crc:
            stats.bad_crc += 1
            i += 1  # this sync word was garbage or the packet was hit; keep scanning
            continue
        _, seq, t_ms, alt, accel, status = _BODY.unpack(body)
        if last_seq is not None:
            gap = (seq - last_seq - 1) & 0xFFFF
            stats.dropped += gap
        last_seq = seq
        out.append(Packet(seq, t_ms, alt, accel, status))
        stats.good += 1
        i += PACKET_SIZE
    return out
