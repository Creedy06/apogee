"""python -m apogee [--loss 0.05] [--corrupt 0.02] [--plot flight.png]

Simulates a flight, sends it over a noisy fake radio link, decodes the packets,
and reports apogee from the Kalman-filtered altitude.
"""

import argparse
import random

from .flight import analyze, rms, simulate, to_packets
from .packets import DecodeStats, decode_stream


def noisy_link(packets, loss: float, corrupt: float, seed: int = 1) -> bytes:
    rng = random.Random(seed)
    stream = bytearray()
    for pk in packets:
        if rng.random() < loss:
            continue                         # packet never arrives
        raw = bytearray(pk.encode())
        if rng.random() < corrupt:
            raw[rng.randrange(2, len(raw))] ^= 1 << rng.randrange(8)   # flip one bit
        if rng.random() < 0.05:
            stream += bytes(rng.randrange(256) for _ in range(rng.randrange(1, 6)))  # line noise
        stream += raw
    return bytes(stream)


def main() -> None:
    ap = argparse.ArgumentParser(prog="apogee")
    ap.add_argument("--loss", type=float, default=0.05, help="fraction of packets lost")
    ap.add_argument("--corrupt", type=float, default=0.02, help="fraction of packets with a flipped bit")
    ap.add_argument("--plot", help="save an altitude chart to this PNG")
    args = ap.parse_args()

    truth = simulate()
    sent = to_packets(truth)
    stats = DecodeStats()
    received = decode_stream(noisy_link(sent, args.loss, args.corrupt), stats)
    report = analyze(received)

    true_apogee = max(p.altitude for p in truth)
    print(f"Packets sent {len(sent)}, decoded {stats.good}, bad CRC {stats.bad_crc}, "
          f"lost {stats.dropped}, resyncs {stats.resyncs}")
    print(f"Apogee (Kalman):  {report.apogee_m:7.1f} m at t={report.apogee_t:.2f}s")
    print(f"Apogee (true):    {true_apogee:7.1f} m")
    print(f"Max velocity:     {report.max_velocity:7.1f} m/s")
    print(f"Flight time:      {report.flight_time:7.1f} s")

    truth_by_t = {round(p.t * 1000): p.altitude for p in truth}
    true_alt = [truth_by_t[round(t * 1000)] for t in report.times]
    print(f"Altitude error, RMS: raw barometer {rms(report.raw_alt, true_alt):.2f} m, "
          f"Kalman {rms(report.est_alt, true_alt):.2f} m")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(report.times[::3], report.raw_alt[::3], ".", ms=2, color="#bbbbbb", label="Barometer (raw)")
        ax.plot(report.times, true_alt, color="#333333", lw=1, label="True altitude")
        ax.plot(report.times, report.est_alt, color="#d1495b", lw=1.5, label="Kalman estimate")
        ax.axvline(report.apogee_t, ls="--", color="#2e86ab", lw=1)
        ax.annotate(f"apogee {report.apogee_m:.0f} m", (report.apogee_t, report.apogee_m),
                    xytext=(8, -4), textcoords="offset points", color="#2e86ab")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("altitude (m)")
        ax.set_title("Apogee: telemetry decode + Kalman filter")
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(frameon=False)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"Saved {args.plot}")


if __name__ == "__main__":
    main()
