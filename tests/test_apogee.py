import struct

from apogee import DecodeStats, Packet, analyze, crc16, decode_stream, simulate, to_packets
from apogee.flight import rms
from apogee.packets import PACKET_SIZE


def test_crc16_known_value():
    # Standard check value for CRC-16/CCITT-FALSE
    assert crc16(b"123456789") == 0x29B1


def test_packet_roundtrip():
    pk = Packet(5, 1234, 321.5, -980, 3)
    raw = pk.encode()
    assert len(raw) == PACKET_SIZE == 18
    (out,) = decode_stream(raw)
    assert (out.seq, out.t_ms, out.accel_mg, out.status) == (5, 1234, -980, 3)
    assert abs(out.altitude_m - 321.5) < 1e-4


def test_rejects_corrupted_packet():
    raw = bytearray(Packet(1, 10, 5.0, 0, 0).encode())
    raw[9] ^= 0x10
    stats = DecodeStats()
    assert decode_stream(bytes(raw), stats) == []
    assert stats.bad_crc == 1


def test_resyncs_through_noise_and_counts_drops():
    good = [Packet(i, i * 20, float(i), 1000, 1).encode() for i in range(6)]
    stream = b"\x00\x13\x99" + good[0] + good[1] + b"\xa5\x7f" + good[3] + good[4] + good[5]
    stats = DecodeStats()
    pkts = decode_stream(stream, stats)
    assert [p.seq for p in pkts] == [0, 1, 3, 4, 5]
    assert stats.dropped == 1 and stats.resyncs >= 1


def test_false_sync_inside_noise():
    # Noise that contains the sync bytes but fails CRC must not swallow the next real packet
    fake = struct.pack("<H", 0xA55A) + b"\x00" * 10
    real = Packet(9, 0, 1.0, 0, 0).encode()
    pkts = decode_stream(fake + real)
    assert [p.seq for p in pkts] == [9]


def test_simulation_is_physical():
    pts = simulate()
    apogee = max(p.altitude for p in pts)
    assert 300 < apogee < 1500
    assert pts[-1].altitude == 0.0
    i = max(range(len(pts)), key=lambda k: pts[k].altitude)
    assert abs(pts[i].velocity) < 2.0


def test_kalman_beats_raw_barometer():
    truth = simulate()
    report = analyze(to_packets(truth, baro_std=4.0))
    true_alt = [p.altitude for p in truth]
    assert rms(report.est_alt, true_alt) < 0.6 * rms(report.raw_alt, true_alt)
    true_apogee = max(true_alt)
    assert abs(report.apogee_m - true_apogee) < 5.0


def test_status_bits_follow_flight():
    pkts = to_packets(simulate())
    assert pkts[0].status == 1               # armed only on the pad
    assert pkts[-1].status & 0b110 == 0b110  # drogue + main by landing
