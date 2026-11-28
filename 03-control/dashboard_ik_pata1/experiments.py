"""Ensayos estándar sobre la réplica de la cascada (los usa export_figures.py y app.py)."""
import numpy as np

from cascade_control import simulate, step_metrics

STEP_CURRENT = (0.03, 0.06, 0.10, 0.30)   # A inicial, A final, t del escalón, duración (s)
STEP_SPEED = (0.0, 1.0, 0.20, 2.0)        # rad/s
STEP_POSITION = (0.0, 0.5, 0.20, 4.0)     # rad (mismo escalón de las pruebas del 12-nov)
SPEED_BAND = 0.05                          # banda de establecimiento del lazo de velocidad (5 %)


def current_step(cfg, j):
    i0, i1, ts, T = STEP_CURRENT
    r = simulate(cfg, T, [0, 0, 0], mode="current", locked=True, joints=(j,),
                 iref=lambda t: [i1 if t >= ts else i0] * 3)
    m = step_metrics(r.t, r["i_meas"][:, j], i0, i1, t0=ts)
    return r, m


def speed_step(cfg, j):
    w0, w1, ts, T = STEP_SPEED
    r = simulate(cfg, T, [0, 0, 0], mode="speed", joints=(j,), wref=lambda t: [w1 if t >= ts else w0] * 3)
    m = step_metrics(r.t, r["w"][:, j], w0, w1, t0=ts, band=SPEED_BAND)
    return r, m


def position_step(cfg, j):
    q0, q1, ts, T = STEP_POSITION
    r = simulate(cfg, T, [0, 0, 0], joints=(j,), ref=lambda t: [q1 if t >= ts else q0] * 3)
    m = step_metrics(r.t, r["q"][:, j], q0, q1, t0=ts, abs_band=cfg["control"]["tolerancia_rad"][j])
    return r, m


def all_steps(cfg):
    out = {}
    for j in range(3):
        out[j] = {"corriente": current_step(cfg, j), "velocidad": speed_step(cfg, j),
                  "posicion": position_step(cfg, j)}
    return out


def ik_verification(kin, n=31, rng_pts=20000, seed=0):
    """FK(IK(p)) en todo el espacio de trabajo + clasificación de puntos cartesianos."""
    Q, P, near = kin.workspace(n)
    best = np.full(len(P), np.inf)
    for c in (1, -1):
        for b in (-1, 1):
            q, s = kin.ik(P, b, c)
            e = np.linalg.norm(kin.fk(q) - P, axis=1)
            best = np.minimum(best, np.where(s <= 1, e, np.inf))
    # Caja alrededor del espacio de trabajo: ¿qué puntos no tienen solución?
    rng = np.random.default_rng(seed)
    lo, hi = P.min(0) - 0.02, P.max(0) + 0.02
    X = rng.uniform(lo, hi, (rng_pts, 3))
    status = np.full(len(X), 9)
    for c in (1, -1):
        for b in (-1, 1):
            _, s = kin.ik(X, b, c)
            status = np.minimum(status, s)
    return {"n_muestras": len(P), "error_max_m": float(best.max()), "error_medio_m": float(best.mean()),
            "cerca_de_singular": int(near.sum()), "caja_puntos": len(X),
            "caja_dentro_limites": int((status == 0).sum()), "caja_fuera_limites": int((status == 1).sum()),
            "caja_fuera_alcance": int((status == 2).sum()), "caja_cilindro_coxa": int((status == 3).sum()),
            "_Q": Q, "_P": P, "_near": near, "_err": best}
