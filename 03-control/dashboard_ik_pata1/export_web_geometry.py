"""Genera web/geometria_pata1.js para el dashboard web en vivo (pata_ik_3d.html).

Toda la geometría sale de kinematics.py (medición de Fusion + config.yaml), así el
navegador usa exactamente el mismo modelo que las figuras. Incluye vectores de prueba
FK/IK que la página verifica al cargar.

    python export_web_geometry.py
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
    # Nube del espacio de trabajo (para dibujar de fondo)
    Qw, Pw, _ = k.workspace(13)
    geo = {
        "fuente": "medicion_fusion_u2.json (ROBOT_ESAMBLE_alfred_mejorado) + config.yaml",
        "p0": mm(k.p0), "p1": mm(k.p1), "p2": mm(k.p2), "pf": mm(k.pf),
        "a0": k.a0.tolist(), "a1": k.a1.tolist(), "a2": k.a2.tolist(),
        "u": k.u.tolist(), "r0": k.r0.tolist(), "s0": float(k.s0),
        "c_d": mm(k.c_d), "f_d": mm(k.f_d), "t_d": mm(k.t_d), "d_ax": float(k.d_ax * 1000),
        "L1": float(k.L1 * 1000), "L2": float(k.L2 * 1000),
        "alpha1": float(k.alpha1), "alpha2": float(k.alpha2),
        "signo": k.sign.tolist(), "offset": k.offset.tolist(),
        "lim_enc": k.lim_enc.tolist(), "rama_codo": int(k.branch),
        "tolerancia_rad": cfg["control"]["tolerancia_rad"],
        "q_inicial_cad": cfg["simulacion"]["postura_inicial_cad_rad"],
        "q_central_cad": qc.tolist(), "ejes_locales": E.tolist(),
        "secuencia_mm": cfg["trayectorias"]["punto_a_punto"]["desplazamientos_mm"],
        "circulo": cfg["trayectorias"]["circulo"], "ciclo_paso": cfg["trayectorias"]["ciclo_paso"],
        "workspace_mm": mm(Pw[::2]),
        "pruebas": [{"q": q.tolist(), "p": mm(k.fk(q))} for q in Qt],
    }
    out = ROOT / "web" / "geometria_pata1.js"
    out.parent.mkdir(exist_ok=True)
    out.write_text("// Generado por export_web_geometry.py. No editar a mano.\nwindow.PATA1 = "
                   + json.dumps(geo, ensure_ascii=False) + ";\n", encoding="utf-8")
    print(f"{out} ({out.stat().st_size / 1024:.0f} kB)")


if __name__ == "__main__":
    main()
