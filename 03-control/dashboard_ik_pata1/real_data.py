"""Lectura del CSV que guarda el dashboard de la pata (pata_dashboard.html, TEL,MULTI a 100 Hz)
y comparación con la simulación de la cascada.

Formato (una fila por muestra, ya lo genera el dashboard actual; no hace falta cambiar el firmware):
  time_ms, selected,
  m{k}_mode, m{k}_duty_percent, m{k}_current_a, m{k}_current_ref_a,
  m{k}_position_rad, m{k}_speed_rad_s, m{k}_speed_ref_rad_s, m{k}_position_ref_rad,
  m{k}_position_state, m{k}_current_status, m{k}_flags          (k = 1 coxa, 2 fémur, 3 tibia)
Unidades: ms, %, A equivalentes del sensado, rad, rad/s. Ángulos en el marco del encoder.
"""
import csv

import numpy as np

from cascade_control import simulate

FIELDS = ["mode", "duty_percent", "current_a", "current_ref_a", "position_rad", "speed_rad_s",
          "speed_ref_rad_s", "position_ref_rad", "position_state"]


def load_csv(path_or_text):
    """Acepta una ruta o el texto del CSV (para la carga desde el dashboard web)."""
    if isinstance(path_or_text, str) and "\n" in path_or_text:
        lines = path_or_text.splitlines()
    else:
        lines = open(path_or_text, encoding="utf-8-sig").read().splitlines()
    rows = list(csv.reader(lines))
    head = [h.strip() for h in rows[0]]
    missing = [f"m1_{f}" for f in FIELDS if f"m1_{f}" not in head]
    if "time_ms" not in head or missing:
        raise ValueError(f"El CSV no tiene el formato del dashboard (faltan: time_ms o {missing})")
    col = {h: i for i, h in enumerate(head)}
    need = ["time_ms"] + [f"m{k}_{f}" for f in FIELDS for k in (1, 2, 3)]
    arr = []
    for r in rows[1:]:
        try:
            arr.append([float(r[col[n]]) for n in need])
        except (ValueError, IndexError):
            continue                       # filas incompletas o comentarios
    A = np.array(arr)
    A = A[np.argsort(A[:, 0], kind="stable")]
    out = {"t": (A[:, 0] - A[0, 0]) / 1000.0}
    for i, f in enumerate(FIELDS):
        out[f] = A[:, 1 + 3 * i: 4 + 3 * i]
    return out


def active_window(data, motor_idx, pad_s=1.0):
    """Recorta al intervalo en que la articulación está en modo posición (mode = 3)."""
    on = np.flatnonzero(data["mode"][:, motor_idx] == 3)
    if not len(on):
        return data
    t0, t1 = data["t"][on[0]] - pad_s, data["t"][on[-1]]
    m = (data["t"] >= max(t0, 0)) & (data["t"] <= t1)
    out = {k: v[m] for k, v in data.items()}
    out["t"] = out["t"] - out["t"][0]
    return out


def simulate_like(cfg, data, motor_idx):
    """Simula la misma secuencia de objetivos (position_ref) que tuvo la pata real."""
    t, ref = data["t"], data["position_ref_rad"]
    q0 = data["position_rad"][0].copy()
    def target(tt):
        i = min(np.searchsorted(t, tt, side="right") - 1, len(t) - 1)
        r = q0.copy()
        r[motor_idx] = ref[max(i, 0), motor_idx] if data["mode"][max(i, 0), motor_idx] == 3 else q0[motor_idx]
        return r
    return simulate(cfg, float(t[-1]), q0, ref=target, joints=(motor_idx,))


def compare(cfg, data, motor_idx):
    sim = simulate_like(cfg, data, motor_idx)
    q_sim = np.interp(data["t"], sim.t, sim["q"][:, motor_idx])
    e = q_sim - data["position_rad"][:, motor_idx]
    return sim, {"rmse_sim_real_rad": float(np.sqrt(np.mean(e**2))), "max_sim_real_rad": float(np.max(np.abs(e)))}
