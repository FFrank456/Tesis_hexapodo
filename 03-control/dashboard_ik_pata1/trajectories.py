"""Trayectorias cartesianas de prueba del pie y su simulación en lazo cerrado.

Ejes locales de la pata (en el marco raíz de Fusion):
  avance  = eje de J1 girado con la coxa en la postura central (horizontal)
  lateral = radial hacia afuera en la postura central
  vertical= +z
"""
import numpy as np

from cascade_control import SimResult, simulate
from kinematics import LegKinematics, rot


def local_axes(kin, q_center):
    R0 = rot(kin.a0, q_center[0])[0]
    fwd = R0 @ kin.u
    lat = R0 @ kin.r0
    return np.stack([fwd, lat, np.array([0.0, 0.0, 1.0])])   # filas


def _to_world(kin, q_center, d_mm):
    E = local_axes(kin, q_center)
    return kin.fk(np.asarray(q_center)) + (np.asarray(d_mm) / 1000) @ E


def _smooth(s):
    s = np.clip(s, 0, 1)
    return 10 * s**3 - 15 * s**4 + 6 * s**5          # quíntica (vel. y acel. nulas en los extremos)


def build(cfg, kin, name):
    """Devuelve dict con t (s), p (N,3) m, tipo ('escalones' | 'seguir') y duración."""
    tc = cfg["trayectorias"]
    qc = np.array(tc["postura_central_cad_rad"], float)
    hold = tc["espera_final_s"]
    dt = 0.02
    if name == "punto_a_punto":
        c = tc["punto_a_punto"]
        pts = np.array([_to_world(kin, qc, d) for d in c["desplazamientos_mm"]])
        T = c["permanencia_s"] * len(pts)
        t = np.arange(0, T + 1e-9, dt)
        idx = np.minimum((t // c["permanencia_s"]).astype(int), len(pts) - 1)
        return {"t": t, "p": pts[idx], "tipo": "escalones", "T": T, "puntos": pts}
    if name == "linea":
        c = tc["linea"]
        A = _to_world(kin, qc, -0.5 * np.array(c["desplazamiento_mm"]))
        B = _to_world(kin, qc, 0.5 * np.array(c["desplazamiento_mm"]))
        T = c["duracion_s"] * 2 * c["idas_y_vueltas"]
        t = np.arange(0, T + hold + 1e-9, dt)
        ph = np.minimum(t, T) / c["duracion_s"]
        k = np.floor(ph).astype(int)
        s = _smooth(ph - k)
        s = np.where(k % 2 == 0, s, 1 - s)
        s[t >= T] = 0.0
        return {"t": t, "p": A + s[:, None] * (B - A), "tipo": "seguir", "T": T}
    if name == "circulo":
        c = tc["circulo"]
        E = local_axes(kin, qc)
        center = kin.fk(qc)
        T = c["periodo_s"] * c["vueltas"]
        t = np.arange(0, T + hold + 1e-9, dt)
        # Arranque y parada suaves en la fase para no pedir un salto de velocidad
        ramp = min(1.0, 0.25 * c["periodo_s"])
        th = 2 * np.pi * c["vueltas"] * _time_warp(np.minimum(t, T), T, ramp)
        r = c["radio_mm"] / 1000
        p = center + r * (np.cos(th)[:, None] * E[0] + np.sin(th)[:, None] * E[2])
        return {"t": t, "p": p, "tipo": "seguir", "T": T}
    if name == "ciclo_paso":
        c = tc["ciclo_paso"]
        E = local_axes(kin, qc)
        center = kin.fk(qc)
        L, H, P, beta = c["largo_paso_mm"] / 1000, c["altura_mm"] / 1000, c["periodo_s"], c["factor_apoyo"]
        T = P * c["ciclos"]
        t = np.arange(0, T + hold + 1e-9, dt)
        ph = (np.minimum(t, T) % P) / P
        x = np.zeros_like(ph)
        z = np.zeros_like(ph)
        st = ph < beta
        x[st] = L / 2 - L * ph[st] / beta                       # apoyo: el pie va hacia atrás
        s = (ph[~st] - beta) / (1 - beta)                        # vuelo: cicloide
        x[~st] = -L / 2 + L * (s - np.sin(2 * np.pi * s) / (2 * np.pi))
        z[~st] = H * 0.5 * (1 - np.cos(2 * np.pi * s))
        x[t >= T] = L / 2
        p = center + x[:, None] * E[0] + z[:, None] * E[2]
        return {"t": t, "p": p, "tipo": "seguir", "T": T, "apoyo": st & (t < T)}
    raise ValueError(name)


def _time_warp(t, T, ramp):
    """Fase normalizada 0…1 con rampas de velocidad de duración 'ramp' al inicio y al final."""
    v = 1.0 / (T - ramp)                   # velocidad de crucero en fase/s
    s = np.where(t < ramp, 0.5 * v * t**2 / ramp,
                 np.where(t > T - ramp, 1 - 0.5 * v * (T - t) ** 2 / ramp, 0.5 * v * ramp + v * (t - ramp)))
    return s


NAMES = {"punto_a_punto": "Punto a punto", "linea": "Línea recta", "circulo": "Círculo", "ciclo_paso": "Ciclo de paso"}


def joint_reference(kin, traj):
    """IK de cada punto -> q_cad (N,3). Falla si algún punto no tiene solución."""
    q = np.zeros_like(traj["p"])
    prev = None
    for i, p in enumerate(traj["p"]):
        qi, st = kin.ik_best(p, prev)
        if st != 0:
            raise ValueError(f"Punto {i} de la trayectoria sin solución dentro de límites (estado {st}): {p*1000} mm")
        q[i] = prev = qi
    return q


def run(cfg, kin, name):
    """Simula la trayectoria con la cascada. Devuelve dict con todo lo necesario para graficar."""
    traj = build(cfg, kin, name)
    q_cad = joint_reference(kin, traj)
    q_enc = kin.cad_to_enc(q_cad)
    q_start_enc = kin.cad_to_enc(cfg["simulacion"]["postura_inicial_cad_rad"])
    t_ref = traj["t"]
    dur = t_ref[-1] + (cfg["trayectorias"]["espera_final_s"] if traj["tipo"] == "escalones" else 0.0)
    # Aproximación: desde la postura inicial, la pata va primero (IR_A) al primer punto de la
    # trayectoria. Esa fase no cuenta en las métricas: el error mide el seguimiento.
    t_ap = float(cfg["trayectorias"].get("aproximacion_s", 0.0))

    def sample(t):
        if t < t_ap:
            return q_enc[0]
        i = min(int((t - t_ap) / 0.02 + 1e-9), len(t_ref) - 1)
        return q_enc[i]

    if traj["tipo"] == "escalones":
        full = simulate(cfg, dur + t_ap, q_start_enc, ref=sample)
    else:
        full = simulate(cfg, dur + t_ap, q_start_enc, follow=sample)
    keep = full.t >= t_ap - 1e-9
    res = SimResult(full.t[keep] - full.t[keep][0], {k: v[keep] for k, v in full.data.items()})
    t = res.t
    idx = np.minimum(np.floor(t / 0.02 + 1e-9).astype(int), len(t_ref) - 1)
    p_des = traj["p"][idx]
    q_des_cad = q_cad[idx]
    q_sim_cad = kin.enc_to_cad(res["q"])
    p_sim = kin.fk(q_sim_cad)
    err = p_sim - p_des
    return {"name": name, "traj": traj, "t": t, "p_des": p_des, "p_sim": p_sim, "err": err,
            "q_des_cad": q_des_cad, "q_sim_cad": q_sim_cad, "res": res, "T": traj["T"],
            "aproximacion": {"t_s": t_ap, "error_inicial_mm": float(np.linalg.norm(err[0]) * 1000),
                             "t_completo": full.t, "q_completo_cad": kin.enc_to_cad(full["q"])}}


def cartesian_metrics(out, cfg):
    """RMSE, máximo y error en régimen permanente (mm)."""
    e = np.linalg.norm(out["err"], axis=1) * 1000
    t = out["t"]
    if out["traj"]["tipo"] == "escalones":
        dwell = cfg["trayectorias"]["punto_a_punto"]["permanencia_s"]
        npts = len(out["traj"]["puntos"])
        ss = []
        for k in range(npts):
            m = (t >= (k + 1) * dwell - 0.5) & (t < (k + 1) * dwell)
            if m.any():
                ss.append(e[m].mean())
        e_ss = float(np.max(ss))
        # RMSE y máximo después del transitorio de cada escalón (último segundo de cada permanencia)
        m = (t % dwell) >= dwell - 1.0
        return {"RMSE_mm": float(np.sqrt(np.mean(e[m] ** 2))), "max_mm": float(e[m].max()), "ss_mm": e_ss,
                "nota": "RMSE y máximo en el último segundo de cada permanencia"}
    T = out["T"]
    m = t <= T
    tail = t >= t[-1] - 0.5
    return {"RMSE_mm": float(np.sqrt(np.mean(e[m] ** 2))), "max_mm": float(e[m].max()),
            "ss_mm": float(e[tail].mean()), "nota": "permanente = media de los últimos 0.5 s, con la referencia quieta"}
