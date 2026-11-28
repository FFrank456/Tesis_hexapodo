"""Lee la Pata 1 directamente de Autodesk Fusion (por el MCP local) y genera:

  fusion/geometria_pata1_v2.json   ejes, orígenes, pie y marco de la pata (mm)
  fusion/mallas_pata1_v2.npz       mallas simplificadas por eslabón, en el marco de la pata

Marco de la pata (el que se usa en todo el dashboard):
  origen = punto del eje de la coxa (J0) en su joint de Fusion
  Z      = eje de la coxa, hacia arriba
  X      = horizontal, a lo largo de la pata estirada (de la coxa hacia el pie)
  Y      = Z × X  (paralelo a los ejes de J1 y J2)

Pose cero = pose del ensamble en Fusion: la pata estirada, todas las articulaciones en 0.
Los ejes de J1 y J2 se toman de los cilindros de los bujes (geometría física), no del
origen del joint de Fusion (el de la tibia está 2.5 mm más abajo que sus bujes).

Requisitos: Fusion abierto con el documento de la pata y el add-in MCP activo.
    python fusion/actualizar_desde_fusion.py [--url http://127.0.0.1:53157/mcp]
"""
import argparse
import datetime
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from fusion_mcp import FusionMCP  # noqa: E402
from mesh_utils import simplify  # noqa: E402

# Ocurrencia de Fusion -> eslabón. Las que no están aquí se avisan y van a "base".
ESLABONES = {
    "base": ["soporte_J0_chasis", "soporte_J0_inferior", "Soporte_ESP32S3", "MotorN20_conEncoder:1+CuerpoMotorN20"],
    "coxa": ["coxa_link_1", "brazo_J0_inferior", "brazo_J0", "Brida_acopleCoxa", "soporte_J1_buje:1",
             "sup_articulacion", "MotorN20_conEncoder:1+Eje"],
    "femur": ["femur_link_1", "Brida_acoplefemur", "femur_tapa_J1", "femur_link:1", "soporte_J1_buje (1)"],
    "tibia": ["tibia_link_1", "inf_articulacion", "Tibiaini", "Tibiamed", "TibiaPunta", "Brida_acople_tibia"],
}
PIE = "TibiaPunta"
# Cilindros que definen cada eje: (ocurrencia, radio aproximado en mm)
EJES = {"J0": ("brazo_J0_inferior", 11.0), "J1": ("soporte_J1_buje:1", 5.2), "J2": ("soporte_J1_buje (1)", 5.2)}

SCRIPT_GEOM = r'''
import adsk.core, adsk.fusion, json
def run(_context: str):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    out = {"doc": app.activeDocument.name, "joints": [], "cyl": {}}
    for j in list(root.allJoints) + list(root.allAsBuiltJoints):
        d = {"name": j.name, "asbuilt": j.objectType.endswith("AsBuiltJoint")}
        try:
            m = j.jointMotion
            if hasattr(m, "rotationValue"):
                d["angle"] = m.rotationValue
                a = m.rotationAxisVector; d["axis"] = [a.x, a.y, a.z]
            g = j.geometry if d["asbuilt"] else j.geometryOrOriginOne
            d["origin_mm"] = [g.origin.x * 10, g.origin.y * 10, g.origin.z * 10]
        except Exception as e:
            d["err"] = str(e)
        out["joints"].append(d)
    for o in root.allOccurrences:
        if not any(o.fullPathName.startswith(k) for k in __NOMBRES__):
            continue
        cs = []
        for b in o.bRepBodies:
            for f in b.faces:
                if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                    c = adsk.core.Cylinder.cast(f.geometry)
                    cs.append({"r": c.radius * 10, "o": [c.origin.x * 10, c.origin.y * 10, c.origin.z * 10],
                               "ax": [c.axis.x, c.axis.y, c.axis.z], "area": f.area * 100})
        out["cyl"][o.fullPathName] = cs
    print(json.dumps(out))
'''

SCRIPT_MESH = r'''
import adsk.core, adsk.fusion, json
OUT = r"__OUT__"
def run(_context: str):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    items = [("<root>", b) for b in root.bRepBodies]
    for o in root.allOccurrences:
        for b in o.bRepBodies:
            items.append((o.fullPathName, b))
    out = []
    for occ, b in items:
        if not b.isVisible:
            continue
        calc = b.meshManager.createMeshCalculator()
        calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh)
        m = calc.calculate()
        out.append({"occ": occ, "body": b.name, "v": [round(x * 10, 3) for x in m.nodeCoordinatesAsDouble],
                    "f": list(m.nodeIndices)})
    with open(OUT, "w") as fh:
        json.dump(out, fh)
    print(len(out))
'''


def pick_cylinder(cyl, occ_key, r):
    hits = [(k, c) for k, cs in cyl.items() if occ_key in k for c in cs if abs(c["r"] - r) < 0.3]
    if not hits:
        raise RuntimeError(f"No encontré el cilindro r≈{r} mm en {occ_key}")
    k, c = max(hits, key=lambda kc: kc[1]["area"])
    return np.array(c["o"]), np.array(c["ax"]) / np.linalg.norm(c["ax"])


def fit_sphere(P):
    A = np.c_[2 * P, np.ones(len(P))]
    b = (P ** 2).sum(1)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = x[:3]
    return c, float(np.sqrt(x[3] + c @ c))


def link_of(occ):
    for name, keys in ESLABONES.items():
        if any(occ.startswith(k) or occ == k or (k in occ and ":" in k) for k in keys):
            return name
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:53157/mcp")
    ap.add_argument("--celda", type=float, default=1.0, help="tamaño de celda para simplificar las mallas (mm)")
    args = ap.parse_args()
    f = FusionMCP(args.url)
    doc = f.active_document()
    print("Documento activo:", doc["name"] if doc else "?")
    names = repr([occ for occ, _ in EJES.values()])
    g = json.loads(f.run_readonly(SCRIPT_GEOM.replace("__NOMBRES__", names)))
    j0 = [j for j in g["joints"] if "coxa" in j["name"].lower()]
    if not j0:
        raise RuntimeError("No encontré el joint de la coxa (J0)")
    for j in g["joints"]:
        if abs(j.get("angle", 0.0)) > 1e-6:
            raise RuntimeError(f"El joint {j['name']} no está en 0 ({j['angle']:.4f} rad): pon la pata estirada en Fusion")

    # --- Ejes físicos ---------------------------------------------------------------
    o0, a0 = pick_cylinder(g["cyl"], *EJES["J0"])
    o1, a1 = pick_cylinder(g["cyl"], *EJES["J1"])
    o2, a2 = pick_cylinder(g["cyl"], *EJES["J2"])
    if a1 @ a2 < 0:
        a2 = -a2
    origin = np.array(j0[0]["origin_mm"])
    # El origen se proyecta sobre el eje físico de la coxa (misma altura del joint)
    zf = np.array([0.0, 0.0, 1.0])
    if abs(abs(a0 @ zf) - 1) > 1e-3:
        raise RuntimeError("El eje de la coxa no es vertical en Fusion")
    origin = np.array([o0[0], o0[1], origin[2]])
    Y = np.array([a1[0], a1[1], 0.0]); Y /= np.linalg.norm(Y)
    X = np.cross(Y, zf)
    # X debe apuntar hacia la pata (del eje de la coxa hacia J2)
    if (o2 - origin) @ X < 0:
        X, Y = -X, -Y
    R = np.stack([X, Y, zf])                       # filas: ejes del marco de la pata en Fusion
    to_leg = lambda p: (np.asarray(p) - origin) @ R.T

    # Punto de cada eje en el plano y = 0 del marco de la pata
    def axis_point(o, a):
        o, a = to_leg(o), a @ R.T
        return o - a * (o[1] / a[1]) if abs(a[1]) > 1e-9 else o
    p0 = np.zeros(3)
    p1, p2 = axis_point(o1, a1), axis_point(o2, a2)

    # --- Mallas ----------------------------------------------------------------------
    tmp = Path(tempfile.gettempdir()) / "pata1_mallas_fusion.json"
    f.run_readonly(SCRIPT_MESH.replace("__OUT__", str(tmp)))
    bodies = json.loads(tmp.read_text())
    tmp.unlink()
    per = {k: [] for k in ESLABONES}
    sin_asignar = set()
    foot_pts = []
    for b in bodies:
        v = to_leg(np.array(b["v"]).reshape(-1, 3))
        tri = v[np.array(b["f"]).reshape(-1, 3)]
        k = link_of(b["occ"])
        if k is None:
            sin_asignar.add(b["occ"])
            k = "base"
        per[k].append(tri)
        if b["occ"].startswith(PIE):
            foot_pts.append(v)
    if sin_asignar:
        print("AVISO: ocurrencias sin eslabón asignado (van a la base):", sorted(sin_asignar))
    # Esfera del pie: vértices de la parte baja de TibiaPunta
    P = np.vstack(foot_pts)
    low = P[P[:, 2] < P[:, 2].min() + 12.0]
    c, r = fit_sphere(low)
    res = np.abs(np.linalg.norm(low - c, axis=1) - r)
    print(f"Esfera del pie: centro {c.round(2)} mm, radio {r:.2f} mm, residuo rms {np.sqrt(np.mean(res**2)):.3f} mm")

    arrays, ntri = {}, 0
    for k, tris in per.items():
        if not tris:
            continue
        V, F = simplify(np.vstack(tris), args.celda)
        arrays[f"{k}_V"], arrays[f"{k}_F"] = V.astype(np.float32), F.astype(np.int32)
        ntri += len(F)
        print(f"  malla {k:6s}: {len(F):6d} triángulos")
    np.savez_compressed(HERE / "mallas_pata1_v2.npz", **arrays)

    geo = {
        "formato": "pata1_v2",
        "documento": doc["name"] if doc else g["doc"],
        "fecha": datetime.date.today().isoformat(),
        "marco": {"descripcion": "origen en el eje de la coxa; Z = eje de la coxa hacia arriba; "
                                 "X = a lo largo de la pata estirada; Y = Z x X (ejes de J1 y J2)",
                  "origen_en_fusion_mm": origin.round(4).tolist(), "ejes_en_fusion_filas": R.round(9).tolist()},
        "unidades": "mm",
        "pose_cero": "pata estirada tal como está el ensamble (todas las articulaciones en 0)",
        "J0": {"punto_mm": p0.round(4).tolist(), "eje": (a0 @ R.T).round(9).tolist()},
        "J1": {"punto_mm": p1.round(4).tolist(), "eje": (a1 @ R.T).round(9).tolist(),
               "nota": "eje físico (cilindro del buje)"},
        "J2": {"punto_mm": p2.round(4).tolist(), "eje": (a2 @ R.T).round(9).tolist(),
               "nota": "eje físico (cilindro del buje)"},
        "joints_fusion": [{**j, "origin_leg_mm": to_leg(j["origin_mm"]).round(4).tolist()}
                          for j in g["joints"] if "origin_mm" in j],
        "pie": {"centro_esfera_mm": c.round(4).tolist(), "radio_mm": round(r, 4),
                "contacto_mm": (c - np.array([0, 0, r])).round(4).tolist(), "pieza": PIE},
        "mallas": {"archivo": "mallas_pata1_v2.npz", "triangulos": ntri, "celda_mm": args.celda,
                   "eslabones": {k: v for k, v in ESLABONES.items()}},
    }
    (HERE / "geometria_pata1_v2.json").write_text(json.dumps(geo, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"J1 {p1.round(3)} mm · J2 {p2.round(3)} mm · pie {c.round(3)} mm")
    print("Escrito fusion/geometria_pata1_v2.json y fusion/mallas_pata1_v2.npz")


if __name__ == "__main__":
    main()
