"""Planta de una articulación: motor N20 + reductora 100:1 + carga identificada.

Convención (la misma de la identificación, reporte_control_cascada.md:31):
el par se expresa en amperes equivalentes del sensado (corriente real × ciclo de
trabajo, que es lo que mide el DRV8876 por IPROPI).

    Jeq·α = T_motor − Ic·sign(ω) − b·ω − gravedad(θ)          [A]

Dos modelos eléctricos:
  - "identificado": I_sensada = a·(PWM − zona muerta), primer orden con tau_e.
    Es el modelo con el que se diseñaron las ganancias del firmware.
  - "fisico": L di/dt = d·V − R·i − Ke·ω  (R, Ke de catálogo, L TODO),
    I_sensada = i·d. Sirve para estudiar el efecto de la fuerza contraelectromotriz.
"""
import math

import numpy as np

DRIVE_STOP, DRIVE_BRAKE = 0, 2   # además de +1 / -1 (sentido de giro)


def gravity_current(terms, theta, thetas, factor=1.0):
    """Σ G·cos(θ + c·θ[motor] + θ0) (misma forma que gravity_hold_a en main.c)."""
    total = 0.0
    for t in terms:
        arg = theta + t["theta0"]
        if t.get("motor") is not None:
            arg += t["c"] * thetas[t["motor"]]
        total += t["g"] * math.cos(arg)
    return factor * total


class JointPlant:
    def __init__(self, cfg, j, locked=False):
        p, m = cfg["planta"], cfg["motor"]
        self.model = p["modelo"]
        self.a = p["a_a_por_pct"][j]
        self.tau_e = p["tau_e_s"]
        self.dz = p["zona_muerta_pct"]
        self.J = p["jeq_a_s2_rad"][j]
        self.ic = p["ic_a"][j]
        self.b = p["b_a_s_rad"][j]
        self.is_ = max(p["is_a"][j], self.ic)
        self.b_brake = p["b_freno_a_s_rad"]
        self.grav = cfg["control"]["gravedad"][j]
        self.grav_factor = p["factor_gravedad_planta"]
        self.V, self.R, self.L, self.Ke = m["tension_v"], m["r_ohm"], m["l_h"], m["ke_v_s_rad"]
        self.locked = locked
        self.theta = 0.0
        self.omega = 0.0
        self.i_sense = 0.0     # corriente sensada (≥ 0, en el sentido de mando)
        self.i_real = 0.0      # solo modelo físico (con signo)
        self.torque = 0.0

    def gravity(self, thetas):
        return gravity_current(self.grav, self.theta, thetas, self.grav_factor)

    def step(self, dt, drive, u_pct, thetas):
        """Avanza dt. drive ∈ {+1, -1, STOP, BRAKE}; u_pct = PWM 0…100 %."""
        s = drive if drive in (1, -1) else 0
        d = u_pct / 100.0 if s else 0.0
        if self.model == "identificado":
            target = self.a * max(u_pct - self.dz, 0.0) if s else 0.0
            self.i_sense += (target - self.i_sense) * (1 - math.exp(-dt / self.tau_e))
            torque = s * self.i_sense
            if drive == DRIVE_BRAKE:
                torque = -self.b_brake * self.omega
        else:
            if drive == DRIVE_STOP:
                self.i_real = 0.0                      # sin conducción (coast)
            else:
                v = s * d * self.V                       # freno: v = 0 (bobinado en corto)
                i_ss = (v - self.Ke * self.omega) / self.R
                self.i_real = i_ss + (self.i_real - i_ss) * math.exp(-dt * self.R / max(self.L, 1e-9))
            d_eff = 1.0 if drive == DRIVE_BRAKE else d
            torque = self.i_real * d_eff
            self.i_sense = max(s * self.i_real, 0.0) * d if s else 0.0
        self.torque = torque
        if self.locked:
            self.omega = 0.0
            return
        # Karnopp: pegada mientras |par externo| ≤ Is
        f_ext = torque - self.gravity(thetas)
        eps = 1e-3
        if abs(self.omega) < eps:
            if abs(f_ext) <= self.is_:
                self.omega = 0.0
                return
            acc = (f_ext - math.copysign(self.ic, f_ext)) / self.J
        else:
            acc = (f_ext - math.copysign(self.ic, self.omega) - self.b * self.omega) / self.J
        w_new = self.omega + acc * dt
        if self.omega != 0.0 and w_new * self.omega < 0:   # cruza cero: se pega
            w_new = 0.0
        self.theta += 0.5 * (self.omega + w_new) * dt
        self.omega = w_new


class Encoder:
    """PCNT x4 + velocidad por diferencia en ventana (encoder.h)."""

    def __init__(self, cfg, theta0=0.0):
        m, c = cfg["motor"], cfg["control"]
        self.cpr = m["pulsos_por_vuelta_motor"] * m["factor_cuadratura"] * m["reduccion"]
        self.k = 2 * math.pi / self.cpr
        self.window_ms = int(round(c["ventana_velocidad_s"] * 1000))
        self.hist = []
        self.c0 = math.floor(theta0 / self.k)

    def sample(self, theta):
        cnt = math.floor(theta / self.k) - self.c0
        self.hist.append(cnt)
        if len(self.hist) > self.window_ms + 1:
            self.hist.pop(0)
        pos = cnt * self.k + self.c0 * self.k
        valid = len(self.hist) > self.window_ms
        speed = (self.hist[-1] - self.hist[0]) * self.k / (self.window_ms / 1000) if valid else 0.0
        return pos, speed, valid


# ------------------------------------------------------------------ modelos lineales
def current_plant_gain(cfg, j):
    """Ganancia del PWM a la corriente sensada (A/%) en la zona lineal."""
    p = cfg["planta"]
    if p["modelo"] == "identificado":
        return p["a_a_por_pct"][j]
    m = cfg["motor"]
    # Linealización alrededor de 50 % con el rotor bloqueado: i·d = d²·V/R
    return 2 * 0.5 * m["tension_v"] / m["r_ohm"] / 100
