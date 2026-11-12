#!/usr/bin/env python3
"""Genera CSV con el formato del dashboard (TEL,MULTI) a partir de una articulación
simulada con parámetros conocidos, para comprobar identificar_articulacion.m.

Modelo (unidades del firmware, A equivalentes):
    Jeq*alfa = I - Ic*sign(w) - b*w - G*cos(theta+theta0)     (pegado si |w|~0 y |I-grav| < Is)
    I = (u/100*V - Ke*w)/R          (motor, con u en %)
Encoder de 2800 cuentas/vuelta, velocidad por diferencias en ventana de 20 ms,
ruido en la corriente, muestreo de 100 Hz como el dashboard.

Uso: generar_csv_sintetico.py <carpeta_salida>
"""
import math, os, random, sys

TRUE = dict(Jeq=0.004, Ic=0.05, b=0.002, G=0.12, theta0=0.1, Is=0.07)
V, R, KE = 12.0, 16.0, 0.31
CPR = 2800

HEADER = ["time_ms", "selected"] + [f"m{i}_{n}" for i in (1, 2, 3) for n in (
    "mode", "duty_percent", "current_a", "current_ref_a", "position_rad", "speed_rad_s",
    "speed_ref_rad_s", "position_ref_rad", "position_state", "current_status", "flags")]


class Joint:
    def __init__(self, p, theta=0.0):
        self.p, self.th, self.w = p, theta, 0.0

    def step(self, u, dt):
        p = self.p
        I = (u / 100 * V - KE * self.w) / R
        if u > 0: I = max(0.0, I)
        elif u < 0: I = min(0.0, I)
        else: I = 0.0
        grav = p["G"] * math.cos(self.th + p["theta0"])
        net = I - grav
        if abs(self.w) < 1e-3 and abs(net) < p["Is"]:
            self.w = 0.0
        else:
            sgn = math.copysign(1, self.w) if abs(self.w) >= 1e-3 else math.copysign(1, net)
            acc = (net - p["Ic"] * sgn - p["b"] * self.w) / p["Jeq"]
            w_new = self.w + acc * dt
            if self.w != 0 and w_new * self.w < 0 and abs(net) < p["Is"]:
                w_new = 0.0
            self.w = w_new
        self.th += self.w * dt
        return I


def run(controller, seconds, motor_col, p, theta0=0.0, gravity=True, seed=1):
    rnd = random.Random(seed)
    pp = dict(p)
    if not gravity: pp["G"] = 0.0
    j = Joint(pp, theta0)
    dt = 0.001
    hist = []
    rows = []
    state = {"gff": 1 if gravity else 0}
    for k in range(int(seconds / dt)):
        t = k * dt
        counts = round(j.th / (2 * math.pi) * CPR)
        pos_meas = counts * 2 * math.pi / CPR
        hist.append((t, counts))
        # 20 ms window speed
        while len(hist) > 1 and t - hist[0][0] > 0.020:
            hist.pop(0)
        w_meas = (hist[-1][1] - hist[0][1]) * 2 * math.pi / CPR / max(t - hist[0][0], 1e-6) if t > 0.02 else float("nan")
        mode, u, wref, pref = controller(t, pos_meas, w_meas, state)
        I = j.step(u, dt)
        if k % 10 == 0:
            cur = abs(I) + rnd.gauss(0, 0.002)
            cur = max(0.0, cur)
            row = [0] * len(HEADER)
            row[0] = round(t * 1000)
            row[1] = motor_col
            base = 2 + (motor_col - 1) * 11
            row[base:base + 11] = [mode, round(u, 2), round(cur, 4), 0, round(pos_meas, 5),
                                   "nan" if math.isnan(w_meas) else round(w_meas, 4),
                                   round(wref, 4), round(pref, 5), 0, 9, 3]
            rows.append(row)
    return rows


def open_ramp(t, pos, w, s):
    """Ensayo 1: rampa de PWM por escalones (como Mx,SEQ), primero +, luego -."""
    period = 0.4
    n = int(t / period)
    if n < 15: return 0, 2 * (n + 1), 0, 0
    if n < 18: return 0, 0, 0, 0
    if n < 33: return 0, -2 * (n - 17), 0, 0
    return 0, 0, 0, 0


def back_and_forth(t, pos, w, s):
    """Ensayo 2: vaivén en POSICIÓN con varias velocidades y aceleraciones."""
    plan = [(0.8, 1, 5), (-0.8, 1, 5), (0.8, 2, 20), (-0.8, 2, 20), (0.8, 4, 40), (-0.8, 4, 40),
            (0.4, 3, 10), (-0.6, 3, 10), (0.8, 5, 60), (-0.8, 5, 60)]
    seg = min(int(t / 2.0), len(plan) - 1)
    pref, vmax, acc = plan[seg]
    if math.isnan(w): w = 0.0
    e = pref - pos
    vdes = max(-vmax, min(vmax, 4.0 * e))
    vr = s.get("vr", 0.0)
    vr += max(-acc * 0.001, min(acc * 0.001, vdes - vr))
    s["vr"] = vr
    # PI de velocidad -> corriente ideal -> PWM por el modelo eléctrico
    ew = vr - w
    s["iw"] = s.get("iw", 0.0) + 0.3 * ew * 0.001
    I = 0.08 * ew + s["iw"] + TRUE["G"] * math.cos(pos + TRUE["theta0"]) * s.get("gff", 1)
    I = max(-0.6, min(0.6, I))
    u = (I * R + KE * w) / V * 100
    u = max(-100, min(100, u))
    return 3, u, vr, pref


def open_steps(t, pos, w, s):
    """Ensayo 3: escalones de PWM alternados (como Mx,SEQ,40:300;-40:300;...)."""
    seq = [40, -40, 60, -60, 25, -25, 80, -80]
    n = int(t / 0.3)
    return 0, (seq[n % len(seq)] if n < 32 else 0), 0, 0


def write(path, rows):
    with open(path, "w") as f:
        f.write(",".join(HEADER) + "\n")
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    # Fémur (columna m2) con gravedad; el ensayo 1 con el eslabón vertical (theta = pi/2 - theta0 => cos = 0).
    write(os.path.join(out, "femur_rampa.csv"), run(open_ramp, 14, 2, TRUE, theta0=math.pi / 2 - TRUE["theta0"], seed=1))
    write(os.path.join(out, "femur_vaiven.csv"), run(back_and_forth, 20, 2, TRUE, seed=2))
    write(os.path.join(out, "femur_escalones.csv"), run(open_steps, 10, 2, TRUE, seed=3))
    # Coxa (columna m1) en un plano horizontal: sin gravedad.
    write(os.path.join(out, "coxa_vaiven.csv"), run(back_and_forth, 20, 1, TRUE, gravity=False, seed=4))
    write(os.path.join(out, "coxa_rampa.csv"), run(open_ramp, 14, 1, TRUE, gravity=False, seed=5))
    print("Parámetros verdaderos:", TRUE)
