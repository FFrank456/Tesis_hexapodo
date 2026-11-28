"""Genera los datos del dashboard web en vivo (web/pata_ik_3d.html):

  web/geometria_pata1.js   geometría (marco de la pata), límites, trayectorias, vectores de prueba
                           FK/IK y los parámetros del gemelo digital (control + planta identificada)
  web/mallas_pata1.js      mallas de Fusion por eslabón (base, coxa, fémur, tibia) en la pose cero

Todo sale de kinematics.py + config.yaml + fusion/*, así el navegador usa el mismo modelo
que las figuras.     python export_web_geometry.py
"""
import json

import numpy as np

import trajectories as tr
from common import ROOT, load_config
from kinematics import LegKinematics


def main():
    cfg = load_config()
    k = LegKinematics(cfg)
    mm = lambda v: (np.asarray(v) * 1000).round(6).tolist()
    qc = np.array(cfg["trayectorias"]["postura_central_cad_rad"], float)
    E = tr.local_axes(k, qc)
    rng = np.random.default_rng(3)
    Qt = rng.uniform(k.lim_cad[:, 0], k.lim_cad[:, 1], (12, 3))
    Qw, Pw, _ = k.workspace(13)
    c, p, m = cfg["control"], cfg["planta"], cfg["motor"]
    geo = {
        "fuente": f"{cfg['geometria']['medicion_fusion']} + config.yaml",
        "marco": "origen en el eje de la coxa; Z = eje de la coxa (arriba); X = a lo largo de la pata estirada; Y = Z x X"
                 if k.frame == "pata" else "marco raíz de Fusion",
        "p0": mm(k.p0), "p1": mm(k.p1), "p2": mm(k.p2), "pf": mm(k.pf), "radio_pie": k.foot_radius * 1000,
        "a0": k.a0.tolist(), "a1": k.a1.tolist(), "a2": k.a2.tolist(),
        "u": k.u.tolist(), "r0": k.r0.tolist(), "s0": float(k.s0),
        "c_d": mm(k.c_d), "f_d": mm(k.f_d), "t_d": mm(k.t_d), "d_ax": float(k.d_ax * 1000),
        "L1": float(k.L1 * 1000), "L2": float(k.L2 * 1000),
        "alpha1": float(k.alpha1), "alpha2": float(k.alpha2),
        "signo": k.sign.tolist(), "offset": k.offset.tolist(),
        "lim_enc": k.lim_enc.tolist(), "rama_codo": int(k.branch),
        "tolerancia_rad": c["tolerancia_rad"],
        "q_inicial_cad": cfg["simulacion"]["postura_inicial_cad_rad"],
        "q_central_cad": qc.tolist(), "ejes_locales": E.tolist(),
        "secuencia_mm": cfg["trayectorias"]["punto_a_punto"]["desplazamientos_mm"],
        "circulo": cfg["trayectorias"]["circulo"], "ciclo_paso": cfg["trayectorias"]["ciclo_paso"],
        "workspace_mm": mm(Pw[::2]),
        "pruebas": [{"q": q.tolist(), "p": mm(k.fk(q))} for q in Qt],
        # Gemelo digital: mismas ganancias, límites y compensaciones que config.yaml (la página
        # las reemplaza por las de la placa cuando llegan las líneas CFG) y la planta identificada.
        "gemelo": {
            "control": {kk: c[kk] for kk in ("pid_kp", "pid_ki", "spi_kp", "spi_ki", "ppi_kp", "ppi_ki", "pwm_max_pct",
                                             "ilimit_a", "vmax_rad_s", "acel_rad_s2", "tolerancia_rad", "frici_ic_a",
                                             "frici_is_a", "fricb_inicio", "fricb_rampa_s", "dither_pct", "dither_hz",
                                             "freno_en_hold", "gravedad")},
            "planta": {kk: p[kk] for kk in ("tau_e_s", "zona_muerta_pct", "a_a_por_pct", "jeq_a_s2_rad", "ic_a",
                                            "b_a_s_rad", "is_a", "b_freno_a_s_rad", "factor_gravedad_planta")},
            "cpr": m["pulsos_por_vuelta_motor"] * m["factor_cuadratura"] * m["reduccion"],
            "kt_nm_a": m["kt_nm_a"],
        },
    }
    web = ROOT / "web"
    web.mkdir(exist_ok=True)
    out = web / "geometria_pata1.js"
    out.write_text("// Generado por export_web_geometry.py. No editar a mano.\nwindow.PATA1 = "
                   + json.dumps(geo, ensure_ascii=False) + ";\n", encoding="utf-8")
    print(f"{out} ({out.stat().st_size / 1024:.0f} kB)")

    if "mallas" in cfg["geometria"]:
        from mesh_utils import load_fusion_meshes
        meshes = load_fusion_meshes(cfg)
        data = {}
        for name, (V, F) in meshes.items():
            data[name] = {"v": (V * 1000).round(2).ravel().tolist(), "f": F.ravel().astype(int).tolist()}
        out = web / "mallas_pata1.js"
        out.write_text("// Mallas de Fusion (mm, marco de la pata, pose cero). Generado por export_web_geometry.py.\n"
                       "window.MALLAS = " + json.dumps(data, separators=(",", ":")) + ";\n", encoding="utf-8")
        print(f"{out} ({out.stat().st_size / 1024:.0f} kB, {sum(len(F) for _, F in meshes.values())} triángulos)")


if __name__ == "__main__":
    main()
