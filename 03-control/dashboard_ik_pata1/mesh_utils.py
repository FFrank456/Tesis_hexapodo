"""Carga de las mallas STL de la pata y su ubicación con la cinemática.

Las STL de la exportación del 4-nov vienen en mm en el marco raíz de esa versión.
Cada malla se traslada para que su articulación coincida con la de la medición
actual (alfred_mejorado) y luego se mueve con la transformación de su eslabón.
Es una referencia visual: esa versión de la pata no es idéntica a la actual.
"""
import numpy as np

from common import resolve


def read_stl(path):
    raw = open(path, "rb").read()
    if raw[:5].lower() == b"solid" and b"facet" in raw[:400]:
        v = [list(map(float, l.split()[1:4])) for l in raw.decode(errors="ignore").splitlines()
             if l.strip().startswith("vertex")]
        return np.array(v, float).reshape(-1, 3, 3)
    n = int(np.frombuffer(raw[80:84], np.uint32)[0])
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(raw[84:84 + n * 50], dt)["v"].astype(float)


def simplify(tris, cell):
    """Agrupamiento de vértices en una rejilla de 'cell' (mismas unidades). Devuelve (V, F)."""
    pts = tris.reshape(-1, 3)
    key = np.floor(pts / cell).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    V = np.zeros((len(uniq), 3))
    np.add.at(V, inv, pts)
    V /= np.bincount(inv, minlength=len(uniq))[:, None]
    F = inv.reshape(-1, 3)
    ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
    return V, F[ok]


LINKS = ("coxa", "femur", "tibia")


def load_fusion_meshes(cfg):
    """Mallas leídas de Fusion (fusion/mallas_pata1_v2.npz), en el marco de la pata, en metros.
    Devuelve {"base": (V, F), "coxa": ..., "femur": ..., "tibia": ...}."""
    path = resolve(cfg, cfg["geometria"]["mallas"])
    d = np.load(path)
    return {k: (d[f"{k}_V"].astype(float) / 1000, d[f"{k}_F"]) for k in ("base",) + LINKS if f"{k}_V" in d}


def load_leg_meshes(cfg, kin):
    """Lista de (V_cero_m (N,3), F) por eslabón (coxa, fémur, tibia) en la pose cero."""
    if "mallas" in cfg["geometria"]:
        m = load_fusion_meshes(cfg)
        return [m[k] for k in LINKS]
    g = cfg["geometria"]["mallas_stl"]
    folder = resolve(cfg, g["carpeta"])
    new_piv = [kin.p0, kin.p1, kin.p2]
    out = []
    for k, name in enumerate(g["archivos"]):
        path = folder / name
        if not path.exists():
            return None
        V, F = simplify(read_stl(path), g["tamano_celda_mm"])
        V = V / 1000 - np.array(g["origen_articulacion_mm"][k]) / 1000 + new_piv[k]
        out.append((V, F))
    return out


def pose_meshes(meshes, kin, q_cad):
    posed = []
    for (V, F), (R, piv, piv0) in zip(meshes, kin.link_transforms(q_cad)):
        posed.append(((V - piv0) @ R.T + piv, F))
    return posed
