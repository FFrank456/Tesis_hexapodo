"""Figuras de la tesis "pata real vs gemelo digital" a partir del CSV que graba web/pata_ik_3d.html.

    python figuras_gemelo.py ruta\\al\\pata1_secuencia_gait.csv  [--salida figures/gemelo]

Genera (PNG 300 dpi + SVG) en la carpeta de salida:
  g01_angulos, g02_velocidad, g03_corriente, g04_esfuerzo_control, g05_par_estimado,
  g06_pie_3d (trayectoria del pie real, gemelo y objetivos sobre la pata de Fusion),
  g07_error_pie (en el tiempo + histograma) y tabla_gemelo.csv / .tex
"""
import argparse
import csv
from pathlib import Path

import numpy as np

import export_figures as ef
from common import ROOT, load_config
from kinematics import LegKinematics

plt = ef.plt
C, JC = ef.C, ef.JC
NAMES = ["Coxa", "Fémur", "Tibia"]
REAL, TWIN, REF = C["coxa"], C["femur"], C["ref"]


def load(path):
    rows = list(csv.reader(open(path, encoding="utf-8-sig")))
    head = rows[0]
    col = {h: i for i, h in enumerate(head)}
    need = ["time_ms"] + [f"m{k}_{f}" for k in (1, 2, 3) for f in ("mode", "duty_percent", "current_a", "current_ref_a",
                                                                       "position_rad", "speed_rad_s", "speed_ref_rad_s",
                                                                       "position_ref_rad", "position_state")]
    tw = [f"tw{k}_{f}" for k in (1, 2, 3) for f in ("position_rad", "speed_rad_s", "current_a", "duty_percent")]
    if any(n not in col for n in need + tw):
        raise SystemExit("El CSV no viene de pata_ik_3d.html (faltan columnas del gemelo tw1_…)")
    num = lambda r, n: float(r[col[n]]) if r[col[n]] not in ("", "NaN", "nan") else np.nan
    data = {n: np.array([num(r, n) for r in rows[1:] if len(r) == len(head)]) for n in need + tw}
    for n in ("target_x_mm", "target_y_mm", "target_z_mm", "foot_x_mm", "foot_y_mm", "foot_z_mm",
              "twin_foot_x_mm", "twin_foot_y_mm", "twin_foot_z_mm", "foot_error_mm", "foot_twin_error_mm", "seq_index"):
        if n in col:
            data[n] = np.array([num(r, n) for r in rows[1:] if len(r) == len(head)])
    sign = offset = None
    if "q_sign" in col and len(rows) > 1:
        sign = np.array(rows[1][col["q_sign"]].split(), float)
        offset = np.array(rows[1][col["q_offset"]].split(), float)
    t = (data["time_ms"] - data["time_ms"][0]) / 1000
    return t, data, sign, offset


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--salida", default=str(ROOT / "figures" / "gemelo"))
    args = ap.parse_args()
    cfg = load_config()
    kin = LegKinematics(cfg)
    t, d, sign, offset = load(args.csv)
    sign = kin.sign if sign is None else sign
    offset = kin.offset if offset is None else offset
    out = Path(args.salida)
    out.mkdir(parents=True, exist_ok=True)
    cfg["figuras"]["carpeta"] = str(out)
    E = ef.Exporter(cfg)
    E.dir = out
    kt = cfg["motor"]["kt_nm_a"]
    pos = lambda k: d[f"m{k+1}_mode"] == 3

    def series(k):
        sg, of = sign[k], offset[k]
        s_r = np.where(d[f"m{k+1}_duty_percent"] < 0, -1, 1)
        s_t = np.where(d[f"tw{k+1}_duty_percent"] < 0, -1, 1)
        tau = lambda cur, duty: np.where(np.abs(duty) >= 25, kt * cur / (np.abs(duty) / 100) * np.sign(duty), np.nan)
        return {
            "q": np.degrees(sg * d[f"m{k+1}_position_rad"] + of), "qt": np.degrees(sg * d[f"tw{k+1}_position_rad"] + of),
            "qr": np.where(pos(k), np.degrees(sg * d[f"m{k+1}_position_ref_rad"] + of), np.nan),
            "w": sg * d[f"m{k+1}_speed_rad_s"], "wt": sg * d[f"tw{k+1}_speed_rad_s"],
            "wr": np.where(pos(k), sg * d[f"m{k+1}_speed_ref_rad_s"], np.nan),
            "i": s_r * d[f"m{k+1}_current_a"], "it": s_t * d[f"tw{k+1}_current_a"],
            "ir": np.where(pos(k), s_r * np.abs(d[f"m{k+1}_current_ref_a"]), np.nan),
            "u": d[f"m{k+1}_duty_percent"], "ut": d[f"tw{k+1}_duty_percent"],
            "tau": tau(d[f"m{k+1}_current_a"], d[f"m{k+1}_duty_percent"]),
            "taut": tau(d[f"tw{k+1}_current_a"], d[f"tw{k+1}_duty_percent"]),
        }
    S = [series(k) for k in range(3)]

    def rows3(name, key, ylabel, title, ref=True, lines=None):
        fig, axs = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True)
        for k, ax in enumerate(axs):
            s = S[k]
            if ref and f"{key}r" in s:
                ax.plot(t, s[f"{key}r"], ls=":", color=REF, lw=1.3, label="referencia")
            ax.plot(t, s[key], color=REAL, lw=1.3, label="pata real")
            ax.plot(t, s[f"{key}t"], ls="--", color=TWIN, lw=1.3, label="gemelo digital")
            for y, c, lab in (lines(k) if lines else []):
                ax.axhline(y, color=c, ls="--", lw=0.9)
                if lab:
                    ax.text(t[0], y, " " + lab, color=c, fontsize=7, va="bottom")
            ax.set_ylabel(f"{NAMES[k]}\n{ylabel}")
        axs[0].legend(loc="upper right", ncol=3)
        axs[-1].set_xlabel("tiempo [s]")
        fig.suptitle(title, y=0.995)
        fig.tight_layout()
        E.save(fig, name)

    c = cfg["control"]
    rows3("g01_angulos", "q", "ángulo [°]", "Ángulo articular: pata real vs gemelo digital (q = 0: pata estirada)")
    rows3("g02_velocidad", "w", "velocidad [rad/s]", "Velocidad articular: pata real vs gemelo digital")
    rows3("g03_corriente", "i", "corriente [A eq.]", "Corriente sensada (real × ciclo de trabajo): pata real vs gemelo",
          lines=lambda k: [(c["ilimit_a"][k], C["crit"], "ILIMIT"), (-c["ilimit_a"][k], C["crit"], None)])
    dz = cfg["planta"]["zona_muerta_pct"]
    rows3("g04_esfuerzo_control", "u", "PWM [%]", "Esfuerzo de control (PWM con signo): pata real vs gemelo", ref=False,
          lines=lambda k: [(c["pwm_max_pct"][k], C["crit"], "saturación"), (-c["pwm_max_pct"][k], C["crit"], None),
                           (dz, C["muted"], "zona muerta"), (-dz, C["muted"], None)])
    rows3("g05_par_estimado", "tau", "par [N·m]", f"Par articular estimado τ = Kt·I/d (Kt = {kt} N·m/A de catálogo; |PWM| ≥ 25 %)", ref=False)

    # --- Pie en 3D sobre la pata de Fusion -------------------------------------------
    have_foot = all(n in d for n in ("foot_x_mm", "twin_foot_x_mm"))
    if have_foot:
        F = np.c_[d["foot_x_mm"], d["foot_y_mm"], d["foot_z_mm"]]
        Ft = np.c_[d["twin_foot_x_mm"], d["twin_foot_y_mm"], d["twin_foot_z_mm"]]
        Tg = np.c_[d["target_x_mm"], d["target_y_mm"], d["target_z_mm"]] if "target_x_mm" in d else None
        q_end = np.array([sign[k] * d[f"m{k+1}_position_rad"][-1] + offset[k] for k in range(3)])
        fig = plt.figure(figsize=(12, 4.6))
        ax = fig.add_subplot(1, 3, 1, projection="3d")
        pts = ef.draw_leg_mesh(ax, cfg, kin, q_end, alpha=0.55, base=True, axes=True)
        ax.plot(*F.T, color=REAL, lw=1.6, label="pie real")
        ax.plot(*Ft.T, ls="--", color=TWIN, lw=1.4, label="pie gemelo")
        if Tg is not None:
            ok = np.isfinite(Tg).all(1)
            ax.scatter(*np.unique(Tg[ok], axis=0).T, color=C["crit"], s=14, marker="x", label="objetivos")
        ef.style3d(ax)
        ef.set_equal_3d(ax, np.vstack(pts + [F[np.isfinite(F).all(1)]]))
        ax.view_init(elev=22, azim=-125)
        ax.legend(loc="upper left", fontsize=7)
        ax.set_title("3D (pata en la pose final)")
        for j, (a, b, lab) in enumerate([(0, 1, "XY (superior)"), (0, 2, "XZ (lateral)")]):
            ax2 = fig.add_subplot(1, 3, j + 2)
            ax2.plot(F[:, a], F[:, b], color=REAL, lw=1.4, label="real")
            ax2.plot(Ft[:, a], Ft[:, b], ls="--", color=TWIN, lw=1.3, label="gemelo")
            if Tg is not None:
                ok = np.isfinite(Tg).all(1)
                ax2.scatter(Tg[ok, a], Tg[ok, b], color=C["crit"], s=14, marker="x", label="objetivos")
            ax2.set_aspect("equal", adjustable="datalim")
            ax2.set_xlabel("xyz"[a] + " [mm]")
            ax2.set_ylabel("xyz"[b] + " [mm]")
            ax2.set_title(f"Proyección {lab}")
        fig.suptitle("Trayectoria del pie: pata real vs gemelo digital (marco de la pata, origen en el eje de la coxa)", y=1.02)
        fig.tight_layout(w_pad=3)
        E.save(fig, "g06_pie_3d")

        fig, axs = plt.subplots(1, 2, figsize=(10, 3.4), gridspec_kw={"width_ratios": [2.2, 1]})
        e_ref, e_tw = d.get("foot_error_mm"), d["foot_twin_error_mm"]
        if e_ref is not None:
            axs[0].plot(t, e_ref, color=C["crit"], lw=1.2, label="real – objetivo")
        axs[0].plot(t, e_tw, ls="--", color=TWIN, lw=1.2, label="real – gemelo")
        axs[0].set_xlabel("tiempo [s]")
        axs[0].set_ylabel("error [mm]")
        axs[0].legend(loc="upper right")
        axs[0].set_title("Error de posición del pie")
        axs[1].hist(e_tw[np.isfinite(e_tw)], bins=40, color=TWIN, edgecolor="white", lw=0.5, alpha=0.9, label="real – gemelo")
        if e_ref is not None:
            axs[1].hist(e_ref[np.isfinite(e_ref)], bins=40, color=C["crit"], edgecolor="white", lw=0.5, alpha=0.6, label="real – objetivo")
        axs[1].set_xlabel("error [mm]")
        axs[1].set_ylabel("muestras")
        axs[1].legend(fontsize=7)
        axs[1].set_title("Histograma")
        fig.tight_layout()
        E.save(fig, "g07_error_pie")

    # --- Tabla de métricas -------------------------------------------------------------
    rows = []
    add = lambda g, e, m, v, u="": rows.append({"grupo": g, "elemento": e, "metrica": m, "valor": float(v), "unidad": u})
    rms = lambda x: float(np.sqrt(np.nanmean(np.asarray(x) ** 2)))
    for k in range(3):
        s, G = S[k], "Pata real vs gemelo digital"
        add(G, NAMES[k], "RMSE ángulo real–gemelo", rms(s["q"] - s["qt"]), "°")
        add(G, NAMES[k], "RMSE ángulo real–referencia", rms(s["q"] - s["qr"]), "°")
        add(G, NAMES[k], "RMSE corriente real–gemelo", rms(s["i"] - s["it"]), "A")
        add(G, NAMES[k], "corriente rms real", rms(s["i"]), "A")
        add(G, NAMES[k], "PWM saturado", 100 * np.mean(np.abs(s["u"]) >= c["pwm_max_pct"][k] - 0.5), "%")
        add(G, NAMES[k], "par máximo estimado", np.nanmax(np.abs(s["tau"])) if np.isfinite(s["tau"]).any() else np.nan, "N·m")
    if have_foot:
        add("Pie", "trayectoria", "RMSE real–gemelo", rms(d["foot_twin_error_mm"]), "mm")
        if "foot_error_mm" in d:
            add("Pie", "trayectoria", "RMSE real–objetivo", rms(d["foot_error_mm"]), "mm")
        if "seq_index" in d and np.isfinite(d["seq_index"]).any():
            si = d["seq_index"]
            fin = [np.flatnonzero(si == v)[-1] for v in np.unique(si[np.isfinite(si)])]
            e = d["foot_error_mm"][fin]
            add("Pie", "al llegar a cada punto", "RMSE", rms(e), "mm")
            add("Pie", "al llegar a cada punto", "máximo", np.nanmax(e), "mm")
    ef.write_tables(E, rows)
    for ext in ("csv", "tex"):
        (out / f"tabla_metricas.{ext}").replace(out / f"tabla_gemelo.{ext}")
    print(f"Listo: {len(E.files)} figuras y tabla_gemelo.csv/.tex en {out}")


if __name__ == "__main__":
    main()
