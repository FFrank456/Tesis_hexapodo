"""Análisis lineal de los tres lazos: respuesta en frecuencia de lazo cerrado, ancho de
banda, cruce y márgenes. Mismos modelos del diseño por asignación de polos
(reporte_control_cascada.md:44-58):

  Corriente (discreto, T = 1 ms):  P(z) = a(1-p)/(z-p) · z^-1,   p = e^{-T/tau_e}
                                    C(z) = Kp + Ki·T·z/(z-1)
  Velocidad (T = 10 ms):           P(s) = 1/(Jeq s + b) · Tcl_corr · e^{-s·tau_v}, tau_v = 19 ms
                                    C(s) = Kp + Ki/s
  Posición (T = 20 ms):            P(s) = Tcl_vel / s · e^{-s·tau_p},  tau_p = 10 ms; C = Kp

No incluye la zona muerta, la fricción ni la envolvente de llegada: son los modelos
lineales con los que se verifica la separación de frecuencias.
"""
import numpy as np

from motor_model import current_plant_gain


def _w(n=2000, lo=-2, hi=3.4):
    return 2 * np.pi * np.logspace(lo, hi, n)


def loops(cfg, j, w=None):
    c, p = cfg["control"], cfg["planta"]
    w = _w() if w is None else w
    s = 1j * w
    T1 = c["periodo_corriente_s"]
    a = current_plant_gain(cfg, j)
    pz = np.exp(-T1 / p["tau_e_s"])
    z = np.exp(s * T1)
    Pi = a * (1 - pz) / (z - pz) / z
    Ci = c["pid_kp"][j] + c["pid_ki"][j] * T1 * z / (z - 1)
    Li = Ci * Pi
    Ti = Li / (1 + Li)

    J, b = p["jeq_a_s2_rad"][j], p["b_a_s_rad"][j]
    tau_v = c["retardo_lazo_velocidad_s"]
    Pv = Ti / (J * s + b) * np.exp(-s * tau_v)
    Cv = c["spi_kp"][j] + c["spi_ki"][j] / s
    Lv = Cv * Pv
    Tv = Lv / (1 + Lv)

    tau_p = c["retardo_lazo_posicion_s"]
    Lp = c["ppi_kp"][j] * Tv / s * np.exp(-s * tau_p)
    if c["ppi_ki"][j]:
        Lp = Lp * (1 + c["ppi_ki"][j] / (c["ppi_kp"][j] * s))
    Tp = Lp / (1 + Lp)
    nyq = {"corriente": 0.5 / T1, "velocidad": 0.5 / c["periodo_velocidad_s"], "posicion": 0.5 / c["periodo_posicion_s"]}
    return w, {"corriente": (Li, Ti), "velocidad": (Lv, Tv), "posicion": (Lp, Tp)}, nyq


def margins(w, L):
    mag = np.abs(L)
    ph = np.unwrap(np.angle(L))
    f = w / (2 * np.pi)
    idx = np.flatnonzero((mag[:-1] >= 1) & (mag[1:] < 1))
    fc = pm = np.nan
    if len(idx):
        i = idx[0]
        fc = f[i]
        pm = (np.degrees(ph[i]) + 180 + 180) % 360 - 180
    # margen de ganancia en el primer cruce de -180°
    ph_deg = np.degrees(ph)
    k = np.flatnonzero(np.diff(np.floor((ph_deg + 180) / 360)) != 0)
    gm = np.inf
    if len(k):
        gm = 1 / mag[k[0]]
    return {"fc_hz": fc, "pm_deg": pm, "gm": gm}


def bandwidth(w, T):
    mag = np.abs(T)
    f = w / (2 * np.pi)
    ref = mag[0]
    idx = np.flatnonzero(mag < ref / np.sqrt(2))
    return f[idx[0]] if len(idx) else np.nan


def discrete_current_poles(cfg, j):
    """Polos exactos del lazo de corriente discreto (prueba de estabilidad)."""
    c, p = cfg["control"], cfg["planta"]
    T = c["periodo_corriente_s"]
    a = current_plant_gain(cfg, j)
    pz = np.exp(-T / p["tau_e_s"])
    kp, ki = c["pid_kp"][j], c["pid_ki"][j]
    g = a * (1 - pz)
    # 1 + C(z)·g/((z-p)·z) = 0,  C = ((kp+ki T) z - kp)/(z-1)
    # (z-1)(z-p)z + g((kp+kiT)z - kp) = 0
    poly = np.polysub(np.polymul(np.polymul([1, -1], [1, -pz]), [1, 0]), [0, 0, -g * (kp + ki * T), g * kp])
    return np.roots(poly)


def summary(cfg, j):
    w, L, nyq = loops(cfg, j)
    out = {}
    for name, (Lk, Tk) in L.items():
        m = margins(w, Lk)
        out[name] = {"bw_hz": bandwidth(w, Tk), **m, "nyquist_hz": nyq[name]}
    out["sep_corr_vel"] = out["corriente"]["bw_hz"] / out["velocidad"]["bw_hz"]
    out["sep_vel_pos"] = out["velocidad"]["bw_hz"] / out["posicion"]["bw_hz"]
    return out
