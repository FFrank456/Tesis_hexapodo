"""Réplica en Python del control en cascada del firmware pata_MOD_DRV.

Se traducen línea a línea (mismas variables, mismo anti-windup, mismas máquinas de
estado) las funciones del firmware:
  - pi_update_sample      (main/current_pi.h)   lazo de corriente, 1 kHz, salida % PWM
  - speed_pi_update       (main/speed_pi.h)     lazo de velocidad, 100 Hz, salida A
  - position_pi_update    (main/position_pi.h)  lazo de posición, 50 Hz, salida rad/s
  - friction_feedforward, cascade_current, dither_term y la parte de cascada de
    motor_control_step  (main/main.c:1048-1339)

Simplificaciones (documentadas en el README): sin FMAP, sin FRIC (PWM, se deja en 0),
sin fallos de sensado/nFAULT y sin ride-through (bloques de corriente siempre válidos).
"""
import math
from dataclasses import dataclass, field

import numpy as np

from motor_model import DRIVE_BRAKE, DRIVE_STOP, Encoder, JointPlant

# Constantes del firmware
POSITION_PERIOD_US = 20000
POSITION_QUIET_US = 100000
POSITION_REVERSE_QUIET_US = 20000
POSITION_STOP_RAD_S = 0.3
POSITION_FOLLOW_STILL_RAD_S = 0.02
POSITION_FOLLOW_DEADBAND_RAD_S = 0.05
SPEED_PERIOD_US = 10000
OVERSPEED_BRAKE_RAD_S = 0.5
FRIC_MOVING_RAD_S = 0.15
FRIC_LOOSE_RAD = 0.01
FRIC_FAST_WREF_RAD_S = 0.2
FRIC_BREAKAWAY_MAX_FACTOR = 2.0
GRAVITY_HOLD_MIN_A = 0.01
FOLLOW_TIMEOUT_US = 100000
FOLLOW_AHEAD_US = 40000

TRACK, REVERSING, HOLD, ARRIVING = 0, 1, 2, 3
STATE_NAMES = {TRACK: "TRACK", REVERSING: "REVERSING", HOLD: "HOLD", ARRIVING: "ARRIVING"}


def clamp(x, lo, hi):
    return min(hi, max(lo, x))


# --------------------------------------------------------------------- current_pi.h
@dataclass
class CurrentPI:
    kp: float
    ki: float
    out_min: float = 0.0
    out_max: float = 100.0
    ref: float = 0.0
    integrator: float = 0.0
    output: float = 0.0
    error: float = 0.0
    last_us: int = 0

    def reset(self):
        self.integrator = self.output = self.error = 0.0
        self.last_us = 0

    def update(self, measured, t_us):
        dt = (t_us - self.last_us) * 1e-6 if self.last_us else 0.0
        e = self.ref - measured
        cand = self.integrator + self.ki * e * dt
        effort = self.kp * e + cand
        high, low = effort > self.out_max, effort < self.out_min
        if not ((high and e > 0) or (low and e < 0)):
            self.integrator = cand
        self.output = min(self.out_max, max(self.out_min, effort))
        self.error = e
        self.last_us = t_us


# ----------------------------------------------------------------------- speed_pi.h
@dataclass
class SpeedPI:
    kp: float
    ki: float
    limit: float
    ref: float = 0.0
    integral: float = 0.0
    output: float = 0.0
    error: float = 0.0
    last_us: int = 0
    limited: bool = False

    def reset(self):
        self.integral = self.output = self.error = 0.0
        self.last_us = 0
        self.limited = False

    def update(self, projected, t_us, inner_high):
        dt = (t_us - self.last_us) * 1e-6 if self.last_us else 0.0
        e = self.ref - projected
        cand = self.integral + self.ki * e * dt
        req = self.kp * e + cand
        high, low = req > self.limit, req < 0
        block = (high and e > 0) or (low and e < 0) or (inner_high and e > 0)
        if not block:
            self.integral = cand
        if inner_high and e > 0:
            req = self.kp * e + self.integral
        self.output = min(self.limit, max(0.0, req))
        self.error = e
        self.last_us = t_us
        self.limited = high or low or (inner_high and e > 0)


# -------------------------------------------------------------------- position_pi.h
@dataclass
class PositionPI:
    kp: float
    ki: float
    vmax: float
    acc: float
    tol: float
    ref: float = 0.0
    error: float = 0.0
    integral: float = 0.0
    output: float = 0.0
    direction: int = 1
    last_us: int = 0
    quiet_since: int = 0
    state: int = TRACK
    limited: bool = False
    following: bool = False
    ff: float = 0.0
    follow_ref: float = 0.0
    follow_time: int = 0

    def update(self, measured, speed, now, downstream):
        dt = (now - self.last_us) * 1e-6 if self.last_us else 0.0
        e = self.ref - measured
        self.error = e
        self.last_us = now
        ff = self.ff if self.following else 0.0
        moving = abs(ff) > POSITION_FOLLOW_STILL_RAD_S
        band = 2 * self.tol if self.state in (HOLD, ARRIVING) else self.tol
        if not moving and abs(e) <= band:
            self.output = self.integral = 0.0
            self.limited = False
            if abs(speed) <= POSITION_STOP_RAD_S:
                if not self.quiet_since:
                    self.quiet_since = now
                self.state = HOLD if now - self.quiet_since >= POSITION_QUIET_US else ARRIVING
            else:
                self.quiet_since = 0
                self.state = ARRIVING
            return
        if self.state in (HOLD, ARRIVING):
            self.state = TRACK
            self.quiet_since = 0
        cand = self.integral + self.ki * e * dt
        req = self.kp * e + cand
        if req * e < 0:
            self.integral = 0.0
            cand = self.ki * e * dt
            req = self.kp * e + cand
        if self.following:
            req += ff
            if req * self.direction < 0 and abs(req) < POSITION_FOLLOW_DEADBAND_RAD_S:
                req = 0.0
        wanted = 1 if req > 0 else -1 if req < 0 else self.direction
        if wanted != self.direction or wanted * speed < -POSITION_STOP_RAD_S or self.state == REVERSING:
            self.state = REVERSING
            self.output = self.integral = 0.0
            self.limited = True
            if abs(speed) <= POSITION_STOP_RAD_S:
                if not self.quiet_since:
                    self.quiet_since = now
                if now - self.quiet_since >= POSITION_REVERSE_QUIET_US:
                    self.direction = wanted
                    self.state = TRACK
                    self.quiet_since = 0
            else:
                self.quiet_since = 0
            return
        self.quiet_since = 0
        blocked = downstream and e * self.direction > 0
        if blocked:
            req = self.kp * e + self.integral
        dist = max(0.0, abs(e) - self.tol)
        latency = POSITION_PERIOD_US * 1e-6 + 0.020
        al = self.acc * latency
        approach = 2 * self.acc * dist / (math.sqrt(al * al + 2 * self.acc * dist) + al)
        lim = self.vmax if moving else min(self.vmax, approach)
        bounded = min(lim, max(-lim, req))
        delta = self.acc * dt
        ramped = min(self.output + delta, max(self.output - delta, bounded))
        ramped = min(lim, max(-lim, ramped))
        saturating = (req - ramped) * e > 0
        if not saturating and not blocked:
            self.integral = cand
        self.output = ramped
        self.limited = saturating or blocked
        self.state = TRACK


# ------------------------------------------------------------- motor_control_step
class FirmwareJoint:
    """Estado y paso de 1 ms de una articulación, como motor_control_step()."""

    def __init__(self, cfg, j, mode="position"):
        c = cfg["control"]
        self.j = j
        self.mode = mode
        self.pid = CurrentPI(c["pid_kp"][j], c["pid_ki"][j], c["pwm_min_pct"][j], c["pwm_max_pct"][j])
        self.speed = SpeedPI(c["spi_kp"][j], c["spi_ki"][j], c["ilimit_a"][j])
        self.pos = PositionPI(c["ppi_kp"][j], c["ppi_ki"][j], c["vmax_rad_s"][j], c["acel_rad_s2"][j],
                              c["tolerancia_rad"][j], state=HOLD)
        self.ic, self.is_ = c["frici_ic_a"][j], c["frici_is_a"][j]
        self.fricb_start = c["fricb_inicio"][j]
        self.fricb_ramp_us = int(c["fricb_rampa_s"][j] * 1e6)
        self.dither_pct, self.dither_hz = c["dither_pct"][j], c["dither_hz"][j]
        self.grav = c["gravedad"][j]
        self.hold_brake = c["freno_en_hold"]
        self.control_dir = 1
        self.drive = 1
        self.direction_out = DRIVE_STOP       # lo que se aplicó en el ciclo anterior
        self.speed_current = 0.0
        self.friction_term = 0.0
        self.breakaway = False
        self.stuck_since = 0
        self.stuck_pos = 0.0
        self.gravity_term = 0.0
        self.current_ref_cmd = 0.0            # modo corriente

    # ---------------------------------------------------------------- compensaciones
    def has_gravity(self):
        return bool(self.grav)

    def gravity_hold(self, pos, positions):
        tot = 0.0
        for t in self.grav:
            arg = pos + t["theta0"]
            if t.get("motor") is not None:
                arg += t["c"] * positions[t["motor"]]
            tot += t["g"] * math.cos(arg)
        return tot

    def friction_ff(self, pos, speed, speed_valid, now):
        ic, is_ = self.ic, max(self.is_, self.ic)
        if self.speed.ref == 0 or is_ <= 0 or not speed_valid:
            self.stuck_since = 0
            self.breakaway = False
            return 0.0
        d = -1.0 if self.control_dir < 0 else 1.0
        projected = d * speed
        moved = d * (pos - self.stuck_pos) if self.stuck_since else (FRIC_LOOSE_RAD if projected > 0 else -FRIC_LOOSE_RAD)
        if projected > FRIC_MOVING_RAD_S and moved >= FRIC_LOOSE_RAD:
            self.stuck_since = 0
            self.breakaway = False
            return ic
        if projected < -FRIC_MOVING_RAD_S and moved <= -FRIC_LOOSE_RAD:
            self.stuck_since = 0
            self.breakaway = False
            return 0.0
        if not self.stuck_since:
            self.stuck_since = now
            self.stuck_pos = pos
        self.breakaway = True
        sf = self.fricb_start if abs(self.speed.ref) >= FRIC_FAST_WREF_RAD_S else 0.0
        start = ic + (is_ - ic) * sf
        cap = max(min(FRIC_BREAKAWAY_MAX_FACTOR * is_, self.speed.limit), ic)
        ramp = (now - self.stuck_since) / (self.fricb_ramp_us if self.fricb_ramp_us > 0 else 1)
        return min(start + (cap - start) * clamp(ramp, 0.0, 1.0), cap)

    def dither(self, now):
        if self.dither_pct <= 0 or self.dither_hz <= 0 or self.speed.ref == 0:
            return 0.0
        half = int(500000.0 / self.dither_hz)
        return -self.dither_pct if (now // half) & 1 else self.dither_pct

    def cascade_current(self, speed_part, pos, positions):
        limit = self.speed.limit
        hold = self.gravity_hold(pos, positions)
        self.gravity_term = hold
        if hold == 0.0:
            return self.control_dir, clamp(speed_part, 0.0, limit), False
        net = (-speed_part if self.control_dir < 0 else speed_part) + hold
        drive = 1 if net >= 0 else -1
        mag = abs(net)
        return drive, (0.0 if mag < GRAVITY_HOLD_MIN_A else min(mag, limit)), mag >= limit

    # ---------------------------------------------------------------- paso de 1 ms
    def step(self, now, pos, speed, speed_valid, i_block, positions):
        """Devuelve (drive, pwm_pct, current_sat)."""
        current_sat = False
        if self.mode == "current":
            self.pid.ref = self.current_ref_cmd
            self.pid.update(i_block, now)
            self.drive = 1
            return 1, self.pid.output, False

        if self.mode == "position" and now - self.pos.last_us >= POSITION_PERIOD_US:
            downstream = self.speed.limited or (self.pid.output >= self.pid.out_max - 0.01 and self.pid.error > 0)
            prev_dir = self.pos.direction
            if self.pos.following:
                age = now - self.pos.follow_time
                if age > FOLLOW_TIMEOUT_US:
                    self.pos.following = False
                    self.pos.ff = 0.0
                    self.pos.ref = self.pos.follow_ref
                else:
                    ahead = min(max(age, 0), FOLLOW_AHEAD_US) * 1e-6
                    self.pos.ref = self.pos.follow_ref + self.pos.ff * ahead
            self.pos.update(pos, speed, now, downstream)
            if prev_dir != self.pos.direction or self.pos.output == 0:
                self.speed.reset()
                if not self.has_gravity():
                    self.pid.reset()
                    self.pid.ref = 0.0
                self.speed_current = 0.0
                self.speed.last_us = now
            self.control_dir = 1 if self.pos.direction > 0 else -1
            self.speed.ref = abs(self.pos.output)

        # Lazo de velocidad
        if self.speed.ref == 0:
            self.speed.reset()
            self.speed.last_us = now
            self.speed_current = 0.0
        elif now - self.speed.last_us >= SPEED_PERIOD_US:
            projected = self.control_dir * speed
            inner_high = (self.drive == self.control_dir and self.pid.output >= self.pid.out_max - 0.01
                          and self.pid.error > 0) or self.breakaway
            if projected > self.speed.ref + OVERSPEED_BRAKE_RAD_S:
                self.speed.integral = 0.0
            self.speed.update(projected, now, inner_high)
            self.speed_current = self.speed.output

        # Cascada -> corriente
        direction_change_brake = False
        self.friction_term = self.friction_ff(pos, speed, speed_valid, now)
        drive, iref, current_sat = self.cascade_current(self.speed_current + self.friction_term, pos, positions)
        self.pid.ref = iref
        if self.speed_current + self.friction_term >= self.speed.limit and not self.grav:
            current_sat = True
        if drive != self.drive:
            self.pid.reset()
            self.drive = drive
        if self.direction_out in (1, -1) and self.drive != self.direction_out:
            direction_change_brake = True
            self.pid.reset()

        brake_hold = self.hold_brake and self.mode == "position" and self.pos.state != TRACK
        overspeed_brake = False
        if self.hold_brake and speed_valid:
            if self.control_dir * speed > self.speed.ref + OVERSPEED_BRAKE_RAD_S:
                brake_hold = True
                overspeed_brake = self.pid.ref == 0 or self.drive == self.control_dir
        if overspeed_brake:
            self.pid.reset()
            self.pid.ref = 0.0
            self.pid.last_us = now
            self.speed.integral = self.speed.output = self.speed_current = 0.0
            self.friction_term = 0.0
            self.breakaway = False
            self.stuck_since = 0
            out = (DRIVE_BRAKE, 0.0)
        elif direction_change_brake:
            self.pid.last_us = now
            out = (DRIVE_BRAKE, 0.0)
        elif self.pid.ref == 0:
            self.pid.reset()
            self.pid.last_us = now
            out = (DRIVE_BRAKE if brake_hold else DRIVE_STOP, 0.0)
        else:
            self.pid.update(i_block, now)
            u = clamp(self.pid.output + self.dither(now), 0.0, self.pid.out_max)
            out = (self.drive, u)
        self.direction_out = out[0]
        return out[0], out[1], current_sat


# ------------------------------------------------------------------------ simulador
@dataclass
class SimResult:
    t: np.ndarray
    data: dict = field(default_factory=dict)

    def __getitem__(self, k):
        return self.data[k]


def simulate(cfg, duration, q0_enc, mode="position", ref=None, follow=None, wref=None, iref=None,
             locked=False, joints=(0, 1, 2)):
    """Simula las 3 articulaciones (marco del encoder).

    mode="position": ref(t) -> objetivos (3,) como IR_A (escalones), o follow(t) -> trayectoria
                     que la maestra manda a 50 Hz (SEGUIR, con feed-forward).
    mode="speed":    wref(t) -> (3,) rad/s.
    mode="current":  iref(t) -> (3,) A; con locked=True el rotor está bloqueado.
    """
    dt = cfg["simulacion"]["dt_planta_s"]
    sub = int(round(1e-3 / dt))
    n = int(round(duration * 1000))
    q0_enc = np.asarray(q0_enc, float)
    plants = [JointPlant(cfg, j, locked=locked) for j in range(3)]
    fws = [FirmwareJoint(cfg, j, mode) for j in range(3)]
    encs = [Encoder(cfg, q0_enc[j]) for j in range(3)]
    for j in range(3):
        plants[j].theta = q0_enc[j]
        fws[j].pos.ref = fws[j].pos.follow_ref = q0_enc[j]
    keys = ["q", "q_meas", "w", "w_meas", "w_ref", "q_ref", "i_meas", "i_ref", "pwm", "state",
            "pwm_sat", "i_sat", "grav", "fric"]
    log = {k: np.zeros((n, 3)) for k in keys}
    i_block = [0.0, 0.0, 0.0]
    drive = [DRIVE_STOP] * 3
    u = [0.0] * 3
    last_follow = None
    for k in range(n):
        now = (k + 1) * 1000           # µs (los temporizadores del firmware empiezan en > 0)
        t = k * 1e-3
        thetas = [p.theta for p in plants]
        meas = [encs[j].sample(thetas[j]) for j in range(3)]
        positions = [m[0] for m in meas]
        if mode == "position":
            if follow is not None and k % 20 == 0:
                qf = np.asarray(follow(t), float)
                for j in joints:
                    fw = fws[j]
                    if last_follow is not None:
                        vmax = fw.pos.vmax
                        fw.pos.ff = clamp((qf[j] - last_follow[j]) / 0.020, -vmax, vmax)
                    fw.pos.follow_ref = qf[j]
                    fw.pos.follow_time = now
                    fw.pos.following = True
                last_follow = qf
            elif ref is not None:
                r = ref(t)
                for j in joints:
                    fws[j].pos.ref = r[j]
        elif mode == "speed":
            w = wref(t)
            for j in joints:
                fws[j].control_dir = 1 if w[j] >= 0 else -1
                fws[j].speed.ref = abs(w[j])
        elif mode == "current":
            r = iref(t)
            for j in joints:
                fws[j].current_ref_cmd = r[j]
        for j in joints:
            pos, spd, valid = meas[j]
            drive[j], u[j], isat = fws[j].step(now, pos, spd, valid, i_block[j], positions)
            log["i_sat"][k, j] = isat
        # Planta durante el ms siguiente; la corriente medida es la media del bloque
        acc = [0.0, 0.0, 0.0]
        for _ in range(sub):
            th = [p.theta for p in plants]
            for j in joints:
                plants[j].step(dt, drive[j], u[j], th)
                acc[j] += plants[j].i_sense
        for j in joints:
            fw, pl = fws[j], plants[j]
            i_block[j] = acc[j] / sub
            s = drive[j] if drive[j] in (1, -1) else 0
            log["q"][k, j] = pl.theta
            log["w"][k, j] = pl.omega
            log["q_meas"][k, j] = meas[j][0]
            log["w_meas"][k, j] = meas[j][1]
            log["w_ref"][k, j] = fw.control_dir * fw.speed.ref
            log["q_ref"][k, j] = fw.pos.ref if mode == "position" else np.nan
            log["i_meas"][k, j] = s * i_block[j]
            log["i_ref"][k, j] = (s if s else fw.drive) * fw.pid.ref
            log["pwm"][k, j] = s * u[j]
            log["state"][k, j] = fw.pos.state
            log["pwm_sat"][k, j] = u[j] >= fw.pid.out_max - 0.01
            log["grav"][k, j] = fw.gravity_term
            log["fric"][k, j] = fw.friction_term
    return SimResult(np.arange(n) * 1e-3, log)


# ------------------------------------------------------------------- métricas
def step_metrics(t, y, y0, yf, t0=0.0, band=0.02, abs_band=None):
    """Tiempo de subida 10–90 %, sobreimpulso, establecimiento y error estacionario.

    band: banda relativa al escalón (2 %) salvo que se dé abs_band (unidades de y).
    """
    m = t >= t0
    t, y = t[m] - t0, y[m]
    step = yf - y0
    if abs(step) < 1e-12:
        return {}
    yn = (y - y0) / step
    def first(cond):
        idx = np.flatnonzero(cond)
        return t[idx[0]] if len(idx) else np.nan
    tr = first(yn >= 0.9) - first(yn >= 0.1)
    mp = max(0.0, (np.max(yn) - 1.0) * 100)
    tol = abs_band if abs_band is not None else band * abs(step)
    out = np.abs(y - yf) > tol
    idx = np.flatnonzero(out)
    # Sin establecer si sigue fuera de la banda en el último 2 % del registro
    ts = 0.0 if not len(idx) else (t[idx[-1] + 1] if idx[-1] < len(t) - max(2, len(t) // 50) else np.nan)
    n_tail = max(1, len(y) // 10)
    ess = abs(yf - np.mean(y[-n_tail:]))
    return {"tr": tr, "Mp": mp, "ts": ts, "ess": ess}
