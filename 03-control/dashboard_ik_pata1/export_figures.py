"""Genera todas las figuras de la sustentación en figures/ (PNG 300 dpi + SVG)
y la tabla de métricas (CSV + LaTeX).

    python export_figures.py            # todo
    python export_figures.py --sin-real # sin la comparación con el CSV de la pata real
"""
import argparse
import json
import math
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import experiments as ex
import loop_analysis as la
import trajectories as tr
from common import ROOT, load_config, resolve
from kinematics import LegKinematics

# ---------------------------------------------------------------- estilo (paleta de referencia)
C = {"coxa": "#2a78d6", "femur": "#eb6834", "tibia": "#1baf7a", "violeta": "#6250d6",
     "ref": "#52514e", "muted": "#898781", "grid": "#e1e0d9", "axis": "#c3c2b7",
     "ink": "#0b0b0b", "crit": "#d03b3b", "warn": "#fab219", "surface": "#fcfcfb"}
JC = [C["coxa"], C["femur"], C["tibia"]]
LOOP_C = {"corriente": C["coxa"], "velocidad": C["femur"], "posicion": C["tibia"]}
LOOP_N = {"corriente": "Corriente", "velocidad": "Velocidad", "posicion": "Posición"}

plt.rcParams.update({
    "font.family": "sans-serif", "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9, "legend.fontsize": 8,
    "axes.edgecolor": C["axis"], "axes.labelcolor": C["ink"], "xtick.color": C["ref"], "ytick.color": C["ref"],
    "axes.grid": True, "grid.color": C["grid"], "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 1.6, "legend.frameon": False,
    "figure.facecolor": "white", "axes.facecolor": "white", "savefig.facecolor": "white",
    "axes.formatter.use_locale": False, "svg.fonttype": "none",
})


class Exporter:
    def __init__(self, cfg):
        self.dir = ROOT / cfg["figuras"]["carpeta"]
        self.dir.mkdir(exist_ok=True)
        self.dpi = cfg["figuras"]["dpi"]
        self.files = []

    def save(self, fig, name):
        for ext in ("png", "svg"):
            fig.savefig(self.dir / f"{name}.{ext}", dpi=self.dpi if ext == "png" else None, bbox_inches="tight")
        plt.close(fig)
        self.files.append(name)
        print(f"  figures/{name}.png / .svg")


def mm(x):
    return np.asarray(x) * 1000


def draw_leg(ax, kin, q, color=C["ink"], lw=2.2, alpha=1.0, label=None):
    P = mm(kin.points(q)[0])
    ax.plot(P[:, 0], P[:, 1], P[:, 2], "-o", color=color, lw=lw, ms=3.5, alpha=alpha, label=label)


def set_equal_3d(ax, pts):
    pts = np.asarray(pts)
    c = (pts.max(0) + pts.min(0)) / 2
    r = (pts.max(0) - pts.min(0)).max() / 2 * 1.05
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))


def style3d(ax):
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.pane.set_facecolor((1, 1, 1, 0))
        a.pane.set_edgecolor(C["grid"])
        a._axinfo["grid"]["color"] = C["grid"]
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_zlabel("z [mm]")


# ===================================================================== cinemática
def fig_workspace(E, kin, ikv):
    P, near = mm(ikv["_P"]), ikv["_near"]
    Q = ikv["_Q"]
    sub = slice(None, None, 3)
    fig = plt.figure(figsize=(6.2, 5.2))
    ax = fig.add_subplot(projection="3d")
    sc = ax.scatter(P[sub, 0], P[sub, 1], P[sub, 2], c=np.degrees(Q[sub, 2]), cmap="Blues", s=2, alpha=0.55,
                    linewidths=0)
    if near.any():
        ax.scatter(*P[near].T, s=6, color=C["crit"], label="cerca de singular (pata estirada)")
    draw_leg(ax, kin, np.zeros(3), label="pata en q = 0")
    style3d(ax)
    set_equal_3d(ax, np.vstack([P, mm(kin.points(np.zeros(3))[0])]))
    cb = fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1)
    cb.set_label("ángulo de la tibia q₂ [°]")
    ax.set_title("Espacio de trabajo alcanzable de la Pata 1 (3D)")
    ax.legend(loc="upper left")
    E.save(fig, "01_espacio_trabajo_3d")

    fig, axs = plt.subplots(1, 2, figsize=(9, 4))
    ax = axs[0]
    ax.scatter(P[:, 0], P[:, 1], s=1, color=C["coxa"], alpha=0.15, linewidths=0)
    P0 = mm(kin.points(np.zeros(3))[0])
    ax.plot(P0[:, 0], P0[:, 1], "-o", color=C["ink"], ms=3, label="q = 0")
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_title("Vista superior (plano XY)")
    ax.legend(loc="best")
    # Corte en el plano de flexión con la coxa en 0: alcance radial vs altura
    ax = axs[1]
    m = np.isclose(Q[:, 0], Q[np.argmin(np.abs(Q[:, 0])), 0])
    rad = (P[m, :2] - mm(kin.p0)[:2]) @ np.array([kin.r0[0], kin.r0[1]])
    rad = np.hypot(*(P[m, :2] - mm(kin.p0)[:2]).T)
    ax.scatter(rad, P[m, 2], s=4, c=np.degrees(Q[m, 2]), cmap="Blues", linewidths=0)
    nm = m & near
    if nm.any():
        ax.scatter(np.hypot(*(P[nm, :2] - mm(kin.p0)[:2]).T), P[nm, 2], s=8, color=C["crit"],
                   label="cerca de singular\n(pata estirada)")
        ax.legend(loc="upper left", fontsize=7)
    r0 = np.hypot(*(P0[:, :2] - mm(kin.p0)[:2]).T)
    ax.plot(r0, P0[:, 2], "-o", color=C["ink"], ms=3)
    ax.set_aspect("equal")
    ax.set_xlabel("distancia al eje de la coxa [mm]")
    ax.set_ylabel("z [mm]")
    ax.set_title(f"Plano de flexión (coxa en {math.degrees(Q[m][0, 0]):.0f}°)")
    fig.suptitle("Espacio de trabajo alcanzable (vistas 2D)", y=1.02)
    fig.tight_layout()
    E.save(fig, "02_espacio_trabajo_2d")


_MESH_CACHE = {}


def _fusion_meshes(cfg):
    if "m" not in _MESH_CACHE:
        try:
            from mesh_utils import load_fusion_meshes
            _MESH_CACHE["m"] = load_fusion_meshes(cfg) if "mallas" in cfg["geometria"] else None
        except Exception as exc:  # noqa: BLE001
            print("  (sin mallas de Fusion:", exc, ")")
            _MESH_CACHE["m"] = None
    return _MESH_CACHE["m"]


def _lambert(tris, rgb, light=(0.4, -0.5, 0.8)):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    L = np.array(light) / np.linalg.norm(light)
    d = 0.45 + 0.55 * np.abs(n @ L)
    return np.clip(np.outer(d, rgb), 0, 1)


def draw_leg_mesh(ax, cfg, kin, q, alpha=1.0, tint=None, base=True, axes=True):
    """Pata con las mallas de Fusion (mm, marco de la pata) en la pose q (CAD). Devuelve los vértices."""
    m = _fusion_meshes(cfg)
    pts = [mm(kin.points(q)[0])]
    if not m:
        return pts
    cols = {"base": (0.78, 0.78, 0.76), "coxa": (0.55, 0.62, 0.70), "femur": (0.45, 0.62, 0.86), "tibia": (0.55, 0.62, 0.70)}
    tr_ = kin.link_transforms(q)
    for k, name in enumerate(["base", "coxa", "femur", "tibia"]):
        if name not in m or (name == "base" and not base):
            continue
        V, F = m[name]
        if name != "base":
            R, piv, piv0 = tr_[k - 1]
            V = (V - piv0) @ R.T + piv
        T = mm(V)[F]
        ax.add_collection3d(Poly3DCollection(T, facecolors=_lambert(T, tint or cols[name]), edgecolor="none",
                                             alpha=alpha, linewidths=0))
        pts.append(mm(V))
    if axes:
        for v, c, lab in [((40, 0, 0), C["crit"], "X"), ((0, 40, 0), "#0ca30c", "Y"), ((0, 0, 45), C["coxa"], "Z (eje coxa)")]:
            ax.quiver(0, 0, 0, *v, color=c, lw=2, arrow_length_ratio=0.15)
            ax.text(*(np.array(v) * 1.12), lab, color=c, fontsize=8, fontweight="bold")
        ax.plot([0, 0], [0, 0], [-35, 60], ls="--", color=C["muted"], lw=0.8)
        ax.text(0, 0, -8, "O", fontsize=8)
    return pts


def fig_leg_mesh(E, cfg, kin):
    for name, q, title in [("03_pata_home", np.array(cfg["simulacion"]["postura_inicial_cad_rad"], float),
                            "Pata 1 en home (pata estirada, q = 0)"),
                           ("03b_pata_postura_central", np.array(cfg["trayectorias"]["postura_central_cad_rad"], float),
                            "Pata 1 en la postura central de las trayectorias")]:
        fig = plt.figure(figsize=(7.5, 5))
        ax = fig.add_subplot(projection="3d")
        pts = draw_leg_mesh(ax, cfg, kin, q)
        draw_leg(ax, kin, q, color=C["ink"], lw=1.6, label="modelo cinemático (J0, J1, J2, pie)")
        style3d(ax)
        set_equal_3d(ax, np.vstack(pts))
        ax.view_init(elev=22, azim=-125)
        deg = np.degrees(q)
        ax.set_title(f"{title}\nq = [{deg[0]:.0f}°, {deg[1]:.0f}°, {deg[2]:.0f}°] · origen en el eje de la coxa", fontsize=9)
        ax.legend(loc="upper left", fontsize=7)
        E.save(fig, name)

# ===================================================================== trayectorias
def fig_trajectory(E, kin, out, metrics):
    key, name = out["name"], tr.NAMES[out["name"]]
    pd_, ps = mm(out["p_des"]), mm(out["p_sim"])
    fig = plt.figure(figsize=(11, 4.0))
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    ax.plot(*pd_.T, "--", color=C["ref"], lw=1.4, label="deseada")
    ax.plot(*ps.T, color=C["coxa"], lw=1.6, label="alcanzada (simulada)")
    draw_leg(ax, kin, out["q_sim_cad"][0], color=C["muted"], lw=1.5, alpha=0.8)
    style3d(ax)
    set_equal_3d(ax, np.vstack([pd_, ps, mm(kin.points(out["q_sim_cad"][0])[0])]))
    ax.set_title("3D")
    ax.legend(loc="upper left", fontsize=7)
    for k, (i, j, lab) in enumerate([(0, 1, "XY (superior)"), (0, 2, "XZ (lateral)")]):
        a = fig.add_subplot(1, 3, k + 2)
        a.plot(pd_[:, i], pd_[:, j], "--", color=C["ref"], lw=1.4, label="deseada")
        a.plot(ps[:, i], ps[:, j], color=C["coxa"], lw=1.4, label="alcanzada")
        a.set_aspect("equal", adjustable="datalim")
        a.set_xlabel("xyz"[i] + " [mm]")
        a.set_ylabel("xyz"[j] + " [mm]")
        a.set_title(f"Proyección {lab}")
    fig.suptitle(f"Trayectoria del efector final: {name}  ·  RMSE {metrics['RMSE_mm']:.2f} mm, "
                 f"máx. {metrics['max_mm']:.2f} mm", y=1.03)
    fig.tight_layout(w_pad=3.0)
    E.save(fig, f"10_trayectoria_{key}")

    # Error cartesiano en el tiempo + histograma
    e = np.linalg.norm(out["err"], axis=1) * 1000
    t = out["t"]
    fig, axs = plt.subplots(1, 2, figsize=(9.5, 3.4), gridspec_kw={"width_ratios": [2.2, 1]})
    ax = axs[0]
    for k, (lab, col) in enumerate(zip("xyz", JC)):
        ax.plot(t, out["err"][:, k] * 1000, color=col, lw=1.0, label=f"e{lab}")
    ax.plot(t, e, color=C["ink"], lw=1.5, label="‖e‖")
    if "apoyo" in out["traj"]:
        ap = out["traj"]["apoyo"]
        tt = out["traj"]["t"]
        _shade(ax, tt, ap, color=C["grid"], label="apoyo")
    ax.axhline(metrics["RMSE_mm"], color=C["violeta"], lw=1, ls=":", label=f"RMSE {metrics['RMSE_mm']:.2f} mm")
    ax.set_xlabel("tiempo [s]")
    ax.set_ylabel("error de posición [mm]")
    ax.set_title(f"Error cartesiano del efector · {name}")
    ax.legend(ncol=3, loc="upper right", fontsize=7)
    ax = axs[1]
    # Mismo intervalo que el RMSE (sin la espera final, que va en el error permanente)
    e_h = e[t <= out["T"]] if out["traj"]["tipo"] == "seguir" else e
    ax.hist(e_h, bins=40, color=C["coxa"], edgecolor="white", linewidth=0.6)
    ax.axvline(metrics["RMSE_mm"], color=C["violeta"], ls=":", lw=1.2)
    ax.axvline(metrics["max_mm"], color=C["crit"], ls="--", lw=1.2)
    ax.text(metrics["max_mm"], ax.get_ylim()[1] * 0.9, f" máx. {metrics['max_mm']:.1f}", color=C["ink"], fontsize=7)
    ax.set_xlabel("‖e‖ [mm]")
    ax.set_ylabel("muestras")
    ax.set_title(f"Histograma · permanente {metrics['ss_mm']:.2f} mm")
    fig.tight_layout()
    E.save(fig, f"11_error_cartesiano_{key}")

    # Error angular por articulación
    ea = np.degrees(out["q_des_cad"] - out["q_sim_cad"])
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    for k, n in enumerate(["Coxa", "Fémur", "Tibia"]):
        ax.plot(t, ea[:, k], color=JC[k], lw=1.2, label=f"{n} (máx. {np.abs(ea[:, k]).max():.1f}°)")
    tol = math.degrees(kin.cfg["control"]["tolerancia_rad"][0])
    ax.axhspan(-tol, tol, color=C["grid"], alpha=0.6, lw=0, label=f"tolerancia ±{tol:.2f}°")
    ax.set_xlabel("tiempo [s]")
    ax.set_ylabel("error angular [°]")
    ax.set_title(f"Error angular por articulación · {name}")
    ax.legend(ncol=2, loc="upper right")
    fig.tight_layout()
    E.save(fig, f"12_error_angular_{key}")


def _shade(ax, t, mask, color, label=None, alpha=0.6):
    mask = np.asarray(mask, bool)
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    for k, (a, b) in enumerate(zip(edges[::2], edges[1::2])):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color=color, alpha=alpha, lw=0, label=label if k == 0 else None)


# ===================================================================== cascada
def fig_steps(E, cfg, steps):
    fig, axs = plt.subplots(3, 3, figsize=(11, 8))
    names = ["Coxa", "Fémur", "Tibia"]
    for j in range(3):
        for row, loop in enumerate(["corriente", "velocidad", "posicion"]):
            ax = axs[row, j]
            r, m = steps[j][loop]
            t = r.t
            if loop == "corriente":
                i0, i1, ts, T = ex.STEP_CURRENT
                ax.plot(t * 1000, r["i_ref"][:, j], "--", color=C["ref"], lw=1.2, label="referencia")
                ax.plot(t * 1000, r["i_meas"][:, j], color=LOOP_C[loop], label="medida (bloque 1 ms)")
                ax.set_xlim(ts * 1000 - 10, ts * 1000 + 60)
                ax.set_xlabel("tiempo [ms]")
                ax.set_ylabel("corriente [A eq.]")
                unit, scale = "ms", 1000
            elif loop == "velocidad":
                ax.plot(t, r["w_ref"][:, j], "--", color=C["ref"], lw=1.2, label="referencia")
                ax.plot(t, r["w_meas"][:, j], color=C["axis"], lw=0.8, label="medida (encoder)")
                ax.plot(t, r["w"][:, j], color=LOOP_C[loop], lw=1.4, label="real (planta)")
                ax.set_xlabel("tiempo [s]")
                ax.set_ylabel("velocidad [rad/s]")
                unit, scale = "s", 1
            else:
                ax.plot(t, r["q_ref"][:, j], "--", color=C["ref"], lw=1.2, label="referencia")
                ax.plot(t, r["q"][:, j], color=LOOP_C[loop], label="respuesta")
                tol = cfg["control"]["tolerancia_rad"][j]
                ax.axhspan(ex.STEP_POSITION[1] - tol, ex.STEP_POSITION[1] + tol, color=C["grid"], lw=0, alpha=0.7,
                           label="tolerancia")
                ax.set_xlabel("tiempo [s]")
                ax.set_ylabel("posición [rad]")
                unit, scale = "s", 1
            ts_txt = "no se establece" if not np.isfinite(m["ts"]) else f"{m['ts'] * scale:.0f} {unit}" if unit == "ms" else f"{m['ts']:.2f} s"
            tr_txt = f"{m['tr'] * scale:.1f} {unit}" if unit == "ms" else f"{m['tr']:.3f} s"
            ax.text(0.98, 0.04, f"tr {tr_txt}\nMp {m['Mp']:.1f} %\nts {ts_txt}\ness {m['ess']:.3g}",
                    transform=ax.transAxes, ha="right", va="bottom", fontsize=7, color=C["ink"],
                    bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=C["grid"]))
            if row == 0:
                ax.set_title(names[j], fontweight="bold")
            if j == 0:
                ax.annotate(LOOP_N[loop], xy=(-0.33, 0.5), xycoords="axes fraction", rotation=90, va="center",
                            fontsize=10, fontweight="bold")
            if j == 2:
                ax.legend(loc="center right", fontsize=7)
    fig.suptitle("Respuesta al escalón de cada lazo de la cascada (réplica del firmware)", y=1.0)
    fig.tight_layout()
    E.save(fig, "20_cascada_escalones")


def fig_bode(E, cfg):
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.6), sharey=True)
    summ = {}
    for j, n in enumerate(["Coxa", "Fémur", "Tibia"]):
        w, L, nyq = la.loops(cfg, j)
        s = la.summary(cfg, j)
        summ[j] = s
        f = w / (2 * np.pi)
        ax = axs[j]
        for loop, (Lk, Tk) in L.items():
            fmax = nyq[loop]
            m = f <= fmax
            ax.semilogx(f[m], 20 * np.log10(np.abs(Tk[m])), color=LOOP_C[loop],
                        label=f"{LOOP_N[loop]}: {s[loop]['bw_hz']:.2f} Hz")
            ax.plot(s[loop]["bw_hz"], -3, "o", color=LOOP_C[loop], ms=5, mec="white")
        ax.axhline(-3, color=C["muted"], lw=0.8, ls=":")
        ax.set_ylim(-40, 10)
        ax.set_xlabel("frecuencia [Hz]")
        ax.set_title(f"{n} · separación {s['sep_corr_vel']:.0f}× / {s['sep_vel_pos']:.1f}×")
        ax.legend(loc="lower left", fontsize=7, title="ancho de banda (−3 dB)", title_fontsize=7)
    axs[0].set_ylabel("|T(jω)| lazo cerrado [dB]")
    fig.suptitle("Bode de lazo cerrado: separación de frecuencias corriente > velocidad > posición", y=1.03)
    fig.tight_layout()
    E.save(fig, "21_bode_lazo_cerrado")
    return summ


def fig_tracking_loops(E, cfg, out):
    """Referencia vs respuesta de los tres lazos durante el ciclo de paso."""
    r, t = out["res"], out["t"]
    fig, axs = plt.subplots(3, 3, figsize=(11, 7.5), sharex=True)
    for j, n in enumerate(["Coxa", "Fémur", "Tibia"]):
        a = axs[0, j]
        a.plot(t, r["q_ref"][:, j], "--", color=C["ref"], lw=1.2, label="referencia")
        a.plot(t, r["q"][:, j], color=LOOP_C["posicion"], lw=1.4, label="respuesta")
        a.set_title(n, fontweight="bold")
        a = axs[1, j]
        a.plot(t, r["w_ref"][:, j], "--", color=C["ref"], lw=1.0, label="referencia")
        a.plot(t, r["w"][:, j], color=LOOP_C["velocidad"], lw=1.0, label="respuesta")
        a = axs[2, j]
        a.plot(t, r["i_ref"][:, j], "--", color=C["ref"], lw=1.0, label="referencia")
        a.plot(t, r["i_meas"][:, j], color=LOOP_C["corriente"], lw=0.8, label="medida")
        a.set_xlabel("tiempo [s]")
    for row, lab in enumerate(["posición [rad]", "velocidad [rad/s]", "corriente [A eq.]"]):
        axs[row, 0].set_ylabel(lab)
        axs[row, 2].legend(loc="upper right", fontsize=7)
    fig.suptitle("Seguimiento de los tres lazos durante el ciclo de paso", y=1.0)
    fig.tight_layout()
    E.save(fig, "22_cascada_seguimiento_ciclo_paso")


def fig_effort(E, cfg, out):
    r, t = out["res"], out["t"]
    c = cfg["control"]
    fig, axs = plt.subplots(2, 3, figsize=(11, 5.2), sharex=True)
    for j, n in enumerate(["Coxa", "Fémur", "Tibia"]):
        a = axs[0, j]
        a.plot(t, r["pwm"][:, j], color=JC[j], lw=0.6)
        _shade(a, t, r["pwm_sat"][:, j] > 0, color=C["crit"], alpha=0.25, label="PWM saturado")
        for s in (1, -1):
            a.axhline(s * c["pwm_max_pct"][j], color=C["crit"], lw=0.9, ls="--")
        a.set_title(f"{n} · PWM saturado {100 * np.mean(r['pwm_sat'][:, j]):.1f} % del tiempo", fontsize=9)
        a.set_ylim(-110, 110)
        a = axs[1, j]
        a.plot(t, r["i_meas"][:, j], color=JC[j], lw=0.8, label="medida")
        a.plot(t, r["i_ref"][:, j], color=C["ref"], lw=0.8, ls="--", label="referencia")
        _shade(a, t, r["i_sat"][:, j] > 0, color=C["warn"], alpha=0.3, label="ILIMIT alcanzado")
        for s in (1, -1):
            a.axhline(s * c["ilimit_a"][j], color=C["crit"], lw=0.9, ls="--")
        a.set_xlabel("tiempo [s]")
    axs[0, 0].set_ylabel("PWM [%]")
    axs[1, 0].set_ylabel("corriente [A eq.]")
    h, l = [], []
    for a in (axs[0, 0], axs[1, 0]):
        hh, ll = a.get_legend_handles_labels()
        h += hh
        l += ll
    for a in axs[1]:
        hh, ll = a.get_legend_handles_labels()
        for x, y in zip(hh, ll):
            if y not in l:
                h.append(x)
                l.append(y)
    fig.legend(h, l, loc="lower center", ncol=4, bbox_to_anchor=(0.5, -0.04))
    fig.suptitle("Esfuerzo de control durante el ciclo de paso (líneas rojas: límites de PWM e ILIMIT)", y=1.0)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    E.save(fig, "23_esfuerzo_control")


def fig_real(E, cfg):
    import real_data as rd
    path = resolve(cfg, cfg["datos_reales"]["csv"])
    if not path.exists():
        print("  (sin CSV real:", path, ")")
        return None
    j = cfg["datos_reales"]["motor"] - 1
    data = rd.active_window(rd.load_csv(path), j)
    sim, m = rd.compare(cfg, data, j)
    names = ["Coxa", "Fémur", "Tibia"]
    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    axs[0].plot(data["t"], data["position_ref_rad"][:, j], "--", color=C["ref"], lw=1.2, label="referencia")
    axs[0].plot(data["t"], data["position_rad"][:, j], color=C["coxa"], lw=1.4, label="pata real")
    axs[0].plot(sim.t, sim["q"][:, j], color=C["femur"], lw=1.2, label="simulación")
    axs[0].set_ylabel("posición [rad]")
    axs[1].plot(data["t"], data["speed_rad_s"][:, j], color=C["coxa"], lw=0.8, label="pata real")
    axs[1].plot(sim.t, sim["w_meas"][:, j], color=C["femur"], lw=0.8, label="simulación")
    axs[1].set_ylabel("velocidad [rad/s]")
    sgn = np.sign(data["duty_percent"][:, j])
    axs[2].plot(data["t"], sgn * data["current_a"][:, j], color=C["coxa"], lw=0.8, label="pata real")
    axs[2].plot(sim.t, sim["i_meas"][:, j], color=C["femur"], lw=0.8, label="simulación")
    axs[2].set_ylabel("corriente [A eq.]")
    axs[2].set_xlabel("tiempo [s]")
    for a in axs:
        a.legend(loc="upper right", ncol=3)
    fig.suptitle(f"{names[j]}: pata real vs simulación ({path.name}) · "
                 f"RMSE de posición {m['rmse_sim_real_rad']:.3f} rad", y=1.0)
    fig.tight_layout()
    E.save(fig, "30_real_vs_simulacion")
    return {"archivo": path.name, "motor": names[j], **m}


# ===================================================================== tablas
def write_tables(E, rows):
    import csv
    path = E.dir / "tabla_metricas.csv"
    keys = ["grupo", "elemento", "metrica", "valor", "unidad"]
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:   # con BOM: Excel lee las tildes
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows)
    # LaTeX: una tabla por grupo
    lines = ["% Generado por export_figures.py. Requiere \\usepackage{booktabs}.", ""]
    groups = dict.fromkeys(r["grupo"] for r in rows)
    for g in groups:
        rs = [r for r in rows if r["grupo"] == g]
        mets = list(dict.fromkeys(f"{r['metrica']} [{r['unidad']}]" if r["unidad"] else r["metrica"] for r in rs))
        elems = list(dict.fromkeys(r["elemento"] for r in rs))
        lines += ["\\begin{table}[htbp]", "\\centering", f"\\caption{{{_tex(g)}}}",
                  "\\begin{tabular}{l" + "r" * len(mets) + "}", "\\toprule",
                  " & ".join(["Elemento"] + [_tex(m) for m in mets]) + " \\\\", "\\midrule"]
        for e in elems:
            vals = []
            for mt in mets:
                hit = [r for r in rs if r["elemento"] == e and
                       (f"{r['metrica']} [{r['unidad']}]" if r["unidad"] else r["metrica"]) == mt]
                vals.append(_fmt(hit[0]["valor"]) if hit else "--")
            lines.append(" & ".join([_tex(e)] + vals) + " \\\\")
        lines += ["\\bottomrule", "\\end{tabular}", "\\end{table}", ""]
    (E.dir / "tabla_metricas.tex").write_text("\n".join(lines), encoding="utf-8")
    print("  figures/tabla_metricas.csv / .tex")


def _tex(s):
    return (str(s).replace("\\", "\\textbackslash{}").replace("%", "\\%").replace("_", "\\_")
            .replace("&", "\\&").replace("#", "\\#").replace("‖", "$\\|$").replace("·", "$\\cdot$"))


def _fmt(v):
    if isinstance(v, str):
        return _tex(v)
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "--"
    av = abs(v)
    return f"{v:.3g}" if av < 1e-2 and av != 0 else f"{v:.3f}" if av < 10 else f"{v:.1f}"


# ===================================================================== main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sin-real", action="store_true")
    ap.add_argument("--config")
    args = ap.parse_args()
    t0 = time.time()
    cfg = load_config(args.config)
    kin = LegKinematics(cfg)
    E = Exporter(cfg)
    rows = []
    add = lambda g, e, m, v, u="": rows.append({"grupo": g, "elemento": e, "metrica": m, "valor": v, "unidad": u})

    print("Cinemática…")
    ikv = ex.ik_verification(kin)
    fig_workspace(E, kin, ikv)
    fig_leg_mesh(E, cfg, kin)
    G = "Verificación de la cinemática inversa"
    add(G, "FK(IK(p))", "error máximo", ikv["error_max_m"], "m")
    add(G, "FK(IK(p))", "muestras", ikv["n_muestras"])
    add(G, "Caja cartesiana", "con solución dentro de límites", ikv["caja_dentro_limites"])
    add(G, "Caja cartesiana", "solución fuera de límites", ikv["caja_fuera_limites"])
    add(G, "Caja cartesiana", "fuera del alcance", ikv["caja_fuera_alcance"])

    print("Trayectorias…")
    outs = {}
    for name in tr.NAMES:
        out = tr.run(cfg, kin, name)
        mt = tr.cartesian_metrics(out, cfg)
        outs[name] = out
        fig_trajectory(E, kin, out, mt)
        G = "Error de posición del efector final"
        add(G, tr.NAMES[name], "RMSE", mt["RMSE_mm"], "mm")
        add(G, tr.NAMES[name], "error máximo", mt["max_mm"], "mm")
        add(G, tr.NAMES[name], "error permanente", mt["ss_mm"], "mm")
        ea = np.degrees(np.abs(out["q_des_cad"] - out["q_sim_cad"]))
        for k, n in enumerate(["Coxa", "Fémur", "Tibia"]):
            add("Error angular máximo por articulación", tr.NAMES[name], n, float(ea[:, k].max()), "°")

    print("Cascada…")
    steps = ex.all_steps(cfg)
    fig_steps(E, cfg, steps)
    summ = fig_bode(E, cfg)
    fig_tracking_loops(E, cfg, outs["ciclo_paso"])
    fig_effort(E, cfg, outs["ciclo_paso"])
    names = ["Coxa", "Fémur", "Tibia"]
    for j in range(3):
        for loop in ("corriente", "velocidad", "posicion"):
            _, m = steps[j][loop]
            g = f"Respuesta al escalón · lazo de {LOOP_N[loop].lower()}"
            u = {"corriente": "A", "velocidad": "rad/s", "posicion": "rad"}[loop]
            add(g, names[j], "tr", m["tr"], "s")
            add(g, names[j], "Mp", m["Mp"], "%")
            add(g, names[j], "ts", m["ts"], "s")
            add(g, names[j], "ess", m["ess"], u)
            s = summ[j][loop]
            g2 = f"Lazo de {LOOP_N[loop].lower()} · análisis lineal"
            add(g2, names[j], "ancho de banda", s["bw_hz"], "Hz")
            add(g2, names[j], "cruce", s["fc_hz"], "Hz")
            add(g2, names[j], "margen de fase", s["pm_deg"], "°")
        add("Separación de frecuencias", names[j], "corriente/velocidad", summ[j]["sep_corr_vel"], "×")
        add("Separación de frecuencias", names[j], "velocidad/posición", summ[j]["sep_vel_pos"], "×")
        r = outs["ciclo_paso"]["res"]
        add("Saturación durante el ciclo de paso", names[j], "PWM saturado", 100 * float(np.mean(r["pwm_sat"][:, j])), "%")
        add("Saturación durante el ciclo de paso", names[j], "ILIMIT alcanzado", 100 * float(np.mean(r["i_sat"][:, j])), "%")

    if not args.sin_real:
        print("Datos reales…")
        try:
            rr = fig_real(E, cfg)
            if rr:
                add("Simulación vs pata real", rr["motor"], "RMSE de posición", rr["rmse_sim_real_rad"], "rad")
                add("Simulación vs pata real", rr["motor"], "error máximo de posición", rr["max_sim_real_rad"], "rad")
        except Exception as exc:  # noqa: BLE001
            print("  (no se pudo comparar con el CSV real:", exc, ")")

    write_tables(E, rows)
    rep = {"singularidades": kin.singularity_report(), "geometria": kin.summary(),
           "limites_cad_rad": kin.lim_cad.tolist(),
           "ik": {k: v for k, v in ikv.items() if not k.startswith("_")}}
    (E.dir / "reporte_cinematica.json").write_text(json.dumps(rep, indent=2, ensure_ascii=False, default=float),
                                                   encoding="utf-8")
    print(f"Listo: {len(E.files)} figuras en {E.dir} ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
