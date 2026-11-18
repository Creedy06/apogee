"""Simulate a model-rocket flight and estimate its state with a Kalman filter."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .packets import ARMED, DROGUE, MAIN, Packet

G = 9.81


@dataclass
class TruePoint:
    t: float
    altitude: float
    velocity: float
    accel: float   # measured by an onboard accelerometer (excludes gravity at rest)


def simulate(burn_s: float = 1.8, thrust_g: float = 9.0, dt: float = 0.02,
             drogue_rate: float = 20.0, main_rate: float = 6.0, main_alt: float = 150.0) -> list[TruePoint]:
    """A simple 1-D flight: motor burn, coast to apogee, drogue descent, main chute."""
    t, h, v = 0.0, 0.0, 0.0
    pts = []
    phase = "boost"
    while True:
        if phase == "boost":
            a = thrust_g * G - G - 0.0015 * v * abs(v)
            if t >= burn_s:
                phase = "coast"
        elif phase == "coast":
            a = -G - 0.0015 * v * abs(v)
            if v <= 0:
                phase = "drogue"
        else:
            target = -(main_rate if (phase == "main") else drogue_rate)
            a = (target - v) * 2.0  # chute quickly settles to its terminal rate
            if phase == "drogue" and h <= main_alt:
                phase = "main"
        v += a * dt
        h += v * dt
        t += dt
        if h <= 0 and t > 1:
            pts.append(TruePoint(t, 0.0, 0.0, 0.0))
            return pts
        pts.append(TruePoint(t, h, v, a + G))


class AltitudeKalman:
    """2-state Kalman filter (altitude, vertical velocity) fusing a noisy barometer
    with the accelerometer as the control input."""

    def __init__(self, baro_std: float = 3.0, accel_std: float = 1.0):
        self.x = [0.0, 0.0]                      # altitude, velocity
        self.P = [[10.0, 0.0], [0.0, 10.0]]      # covariance
        self.R = baro_std ** 2
        self.q = accel_std ** 2

    def predict(self, accel: float, dt: float) -> None:
        h, v = self.x
        self.x = [h + v * dt + 0.5 * accel * dt * dt, v + accel * dt]
        p00, p01, p10, p11 = self.P[0][0], self.P[0][1], self.P[1][0], self.P[1][1]
        # P = F P F^T + Q, with F = [[1, dt], [0, 1]]
        n00 = p00 + dt * (p10 + p01) + dt * dt * p11
        n01 = p01 + dt * p11
        n10 = p10 + dt * p11
        n11 = p11
        dt2 = dt * dt
        q = self.q
        self.P = [[n00 + 0.25 * dt2 * dt2 * q, n01 + 0.5 * dt2 * dt * q],
                  [n10 + 0.5 * dt2 * dt * q, n11 + dt2 * q]]

    def update(self, baro_alt: float) -> None:
        y = baro_alt - self.x[0]
        s = self.P[0][0] + self.R
        k0, k1 = self.P[0][0] / s, self.P[1][0] / s
        self.x = [self.x[0] + k0 * y, self.x[1] + k1 * y]
        p00, p01, p10, p11 = self.P[0][0], self.P[0][1], self.P[1][0], self.P[1][1]
        self.P = [[(1 - k0) * p00, (1 - k0) * p01],
                  [p10 - k1 * p00, p11 - k1 * p01]]


def to_packets(points: list[TruePoint], baro_std: float = 3.0, accel_std_mg: float = 50.0,
               seed: int = 7, main_alt: float = 150.0) -> list[Packet]:
    """Turn the true flight into the noisy packets a flight computer would radio down."""
    rng = random.Random(seed)
    packets = []
    apogee_seen = False
    peak = 0.0
    for i, p in enumerate(points):
        peak = max(peak, p.altitude)
        if p.velocity < 0:
            apogee_seen = True
        status = ARMED
        if apogee_seen:
            status |= DROGUE
            if p.altitude <= main_alt and peak > main_alt:
                status |= MAIN
        baro = p.altitude + rng.gauss(0, baro_std)
        accel_mg = int(round(p.accel / G * 1000 + rng.gauss(0, accel_std_mg)))
        accel_mg = max(-32768, min(32767, accel_mg))
        packets.append(Packet(i, int(round(p.t * 1000)), baro, accel_mg, status))
    return packets


@dataclass
class FlightReport:
    apogee_m: float
    apogee_t: float
    max_velocity: float
    flight_time: float
    times: list[float]
    raw_alt: list[float]
    est_alt: list[float]
    est_vel: list[float]


def analyze(packets: list[Packet]) -> FlightReport:
    """Run the Kalman filter over received packets and find apogee."""
    kf = AltitudeKalman()
    times, raw, est, vel = [], [], [], []
    prev_t = None
    for pk in packets:
        t = pk.t_ms / 1000
        dt = 0.02 if prev_t is None else max(t - prev_t, 1e-3)
        prev_t = t
        accel = pk.accel_mg / 1000 * G - G   # remove gravity to get net vertical acceleration
        kf.predict(accel, dt)
        kf.update(pk.altitude_m)
        times.append(t)
        raw.append(pk.altitude_m)
        est.append(kf.x[0])
        vel.append(kf.x[1])
    i_max = max(range(len(est)), key=est.__getitem__)
    return FlightReport(
        apogee_m=est[i_max], apogee_t=times[i_max], max_velocity=max(vel),
        flight_time=times[-1], times=times, raw_alt=raw, est_alt=est, est_vel=vel,
    )


def rms(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / len(a))
