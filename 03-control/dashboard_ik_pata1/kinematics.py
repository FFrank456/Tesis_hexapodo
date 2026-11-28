"""Cinemática directa e inversa de la Pata 1 (Unidad 2).

La geometría sale de la medición de Fusion (medicion_fusion_u2.json), con la misma
convención que 01-mecanica/cinematica-directa-pata/cinematica_pata1_u2.m:

    J1   = p0 + R0·c
    J2   = J1 + R0·R1·f
    pie  = J2 + R0·R1·R2·t

con R0 = giro q0 alrededor del eje de J0, R1 = giro q1 alrededor de -eje(J1) y
R2 = giro q2 alrededor de +eje(J2). c, f y t son los vectores entre orígenes en q = 0.
Todas las magnitudes internas están en metros y radianes (ángulos de Fusion, q_cad).

Inversa analítica: el eje de J1 es horizontal y paralelo al de J2. En el marco de la
coxa, la componente del pie a lo largo de ese eje es constante (d_ax); la coxa se
resuelve en planta y fémur + tibia son un 2R plano con ángulos fijos de desplazamiento.
"""
import json

import numpy as np

from common import load_config, resolve


def rot(axis, ang):
    """Matrices de giro (Rodrigues). ang escalar o array (N,) -> (N,3,3)."""
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    ang = np.atleast_1d(np.asarray(ang, float))
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    s, c = np.sin(ang)[:, None, None], np.cos(ang)[:, None, None]
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


class LegKinematics:
    def __init__(self, cfg=None, foot_point_mm=None):
        cfg = cfg or load_config()
        self.cfg = cfg
        data = json.loads(resolve(cfg, cfg["geometria"]["medicion_fusion"]).read_text(encoding="utf-8"))
        if data.get("formato") == "pata1_v2":
            self._load_v2(data, foot_point_mm)
        else:
            self._load_v1(data, cfg, foot_point_mm)
        self._finish(cfg)

    def _load_v2(self, data, foot_point_mm):
        """Geometría leída de Fusion por fusion/actualizar_desde_fusion.py, ya en el marco de la pata:
        origen en el eje de la coxa, Z hacia arriba, X a lo largo de la pata estirada, Y = Z × X."""
        self.frame = "pata"
        self.source = data
        self.p0 = np.array(data["J0"]["punto_mm"]) / 1000
        self.p1 = np.array(data["J1"]["punto_mm"]) / 1000
        self.p2 = np.array(data["J2"]["punto_mm"]) / 1000
        self.pf = np.array(foot_point_mm if foot_point_mm is not None else data["pie"]["centro_esfera_mm"]) / 1000
        self.foot_radius = data["pie"]["radio_mm"] / 1000
        R = np.array(data["marco"]["ejes_en_fusion_filas"])
        y = np.array(data["J1"]["eje"], float)
        y = y / np.linalg.norm(y) * np.sign(y[1])                   # +Y del marco
        # Sentidos positivos = los de Fusion: J0 gira alrededor del eje de su joint (-Z),
        # J1 alrededor de -Y (sube el fémur, README de cinemática) y J2 alrededor de +Y (baja el pie).
        j0 = [j for j in data["joints_fusion"] if "coxa" in j["name"].lower()]
        self.a0 = R @ np.array(j0[0]["axis"]) if j0 else np.array([0.0, 0.0, -1.0])
        self.a1 = -y
        jt = [j for j in data["joints_fusion"] if "tibia" in j["name"].lower()]
        self.a2 = R @ np.array(jt[0]["axis"]) if jt else y
        assert abs(abs(self.a2 @ y) - 1) < 1e-6, "J2 debe ser paralelo a Y"
        self.j1_axis_raw = y

    def _load_v1(self, data, cfg, foot_point_mm):
        """Medición anterior (medicion_fusion_u2.json), en el marco raíz de Fusion."""
        self.frame = "fusion"
        self.source = data
        self.foot_radius = 0.0

        def joint(parent, child):
            hits = [u for u in data["uniones"] if not u["suprimida"]
                    and parent in u["ocurrencias"] and child in u["ocurrencias"]]
            assert len(hits) == 1, (parent, child)
            assert abs(hits[0]["angulo_rad"]) < 1e-8, "la medición debe estar en la pose cero"
            return hits[0]

        j0 = joint("base_J0:1", "coxa_link_1:1")
        j1 = joint("coxa_link_1:1", "femur_link_1:1")
        j2 = joint("femur_link_1:1", "tibia_link_1:1")
        self.p0 = np.array(j0["origen_mm"]) / 1000
        self.p1 = np.array(j1["origen_mm"]) / 1000
        self.p2 = np.array(j2["origen_mm"]) / 1000
        if foot_point_mm is None:
            body = [b for b in data["tibias"][0]["cuerpos"] if b["nombre"] == cfg["geometria"]["cuerpo_pie"]][0]
            foot_point_mm = body["min_z_mm"]
        self.pf = np.array(foot_point_mm, float) / 1000
        # Ejes con los signos que reproducen los giros positivos de Fusion (README, líneas 10-12).
        self.a0 = np.array(j0["eje"]) / np.linalg.norm(j0["eje"])
        self.a1 = -np.array(j1["eje"]) / np.linalg.norm(j1["eje"])
        self.a2 = np.array(j2["eje"]) / np.linalg.norm(j2["eje"])
        self.j1_axis_raw = np.array(j1["eje"], float)

    def _finish(self, cfg):
        self.c = self.p1 - self.p0
        self.f = self.p2 - self.p1
        self.t = self.pf - self.p2

        # --- Base para la inversa ---------------------------------------------------
        u = np.array(self.j1_axis_raw, float)
        u[2] = 0.0
        self.u = u / np.linalg.norm(u)                 # eje de J1 (horizontal)
        self.r0 = np.cross(self.u, [0.0, 0.0, 1.0])     # radial hacia afuera en q0 = 0
        assert abs(abs(np.dot(self.a0, [0, 0, 1])) - 1) < 1e-9, "J0 debe ser vertical"
        assert abs(abs(np.dot(self.a1, self.a2)) - 1) < 1e-9, "J1 y J2 deben ser paralelos"
        dec = lambda v: np.array([self.u @ v, self.r0 @ v, v[2]])
        self.c_d, self.f_d, self.t_d = dec(self.c), dec(self.f), dec(self.t)
        self.d_ax = self.c_d[0] + self.f_d[0] + self.t_d[0]
        self.L1 = np.hypot(self.f_d[1], self.f_d[2])
        self.L2 = np.hypot(self.t_d[1], self.t_d[2])
        self.alpha1 = np.arctan2(self.f_d[2], self.f_d[1])
        self.alpha2 = np.arctan2(self.t_d[2], self.t_d[1])
        # Signo del giro de J0 respecto de +z (eje medido hacia -z -> -1).
        self.s0 = np.sign(self.a0[2])

        # --- Mapeo encoder <-> CAD y límites ---------------------------------------
        art = cfg["articulaciones"]
        self.sign = np.array(art["signo_encoder_a_cad"], float)
        self.offset = np.array(art["offset_cad_rad"], float)
        lim_enc = np.array(art["limites_encoder_rad"], float)
        lim_cad = self.sign[:, None] * lim_enc + self.offset[:, None]
        self.lim_cad = np.sort(lim_cad, axis=1)
        self.lim_enc = lim_enc
        ik = cfg["cinematica_inversa"]
        self.branch = ik["rama_codo"]
        self.sing_thr = ik["umbral_singularidad"]

    # ---------------------------------------------------------------- mapeos
    def enc_to_cad(self, q_enc):
        return self.sign * np.asarray(q_enc) + self.offset

    def cad_to_enc(self, q_cad):
        return (np.asarray(q_cad) - self.offset) / self.sign

    def within_limits(self, q_cad, margin=1e-9):
        q = np.atleast_2d(q_cad)
        return np.all((q >= self.lim_cad[:, 0] - margin) & (q <= self.lim_cad[:, 1] + margin), axis=1)

    # ---------------------------------------------------------------- directa
    def _rots(self, q):
        q = np.atleast_2d(np.asarray(q, float))
        R0 = rot(self.a0, q[:, 0])
        R01 = R0 @ rot(self.a1, q[:, 1])
        R012 = R01 @ rot(self.a2, q[:, 2])
        return R0, R01, R012

    def points(self, q):
        """Orígenes J0, J1, J2 y pie: array (N,4,3) en m."""
        R0, R01, R012 = self._rots(q)
        J0 = np.broadcast_to(self.p0, (R0.shape[0], 3))
        J1 = J0 + R0 @ self.c
        J2 = J1 + R01 @ self.f
        P = J2 + R012 @ self.t
        return np.stack([J0, J1, J2, P], axis=1)

    def fk(self, q):
        """Posición del pie (N,3) en m (o (3,) si q es un solo vector)."""
        p = self.points(q)[:, 3]
        return p[0] if np.ndim(q) == 1 else p

    def link_transforms(self, q):
        """(R_k, pivote_k(q), pivote_k(0)) de coxa, fémur y tibia, para mover mallas."""
        R0, R01, R012 = self._rots(q)
        pts = self.points(q)
        piv0 = [self.p0, self.p1, self.p2]
        return [(R0[0], pts[0, 0], piv0[0]), (R01[0], pts[0, 1], piv0[1]), (R012[0], pts[0, 2], piv0[2])]

    def jacobian(self, q, h=1e-7):
        q = np.asarray(q, float)
        J = np.zeros((3, 3))
        for k in range(3):
            dq = np.zeros(3)
            dq[k] = h
            J[:, k] = (self.fk(q + dq) - self.fk(q - dq)) / (2 * h)
        return J

    def rel_knee_angle(self, q):
        """Ángulo relativo fémur–tibia en el plano de flexión (0 = pata estirada)."""
        q = np.atleast_2d(q)
        return self.alpha2 - self.alpha1 - q[:, 2]

    # ---------------------------------------------------------------- inversa
    def ik(self, p, branch=None, coxa=1):
        """Inversa analítica. p (3,) o (N,3) en m.

        Devuelve (q_cad (N,3), estado (N,)) con estado:
          0 = solución dentro de límites, 1 = solución fuera de límites,
          2 = fuera del alcance geométrico, 3 = dentro del cilindro de la coxa (|xy| < d_ax).
        """
        branch = self.branch if branch is None else branch
        P = np.atleast_2d(np.asarray(p, float))
        v = P - self.p0
        h2 = v[:, 0] ** 2 + v[:, 1] ** 2
        status = np.zeros(len(P), int)
        inside = h2 < self.d_ax ** 2
        # coxa = +1: pie por delante del eje de la coxa (radial > 0); -1: pie "por detrás" (pata muy plegada)
        rho = coxa * np.sqrt(np.clip(h2 - self.d_ax ** 2, 0, None))
        # Pie en planta en el marco de la coxa (q0 = 0): d_ax·u + rho·r0
        w = self.d_ax * self.u[None, :2] + rho[:, None] * self.r0[None, :2]
        ang_w = np.arctan2(w[:, 1], w[:, 0])
        ang_v = np.arctan2(v[:, 1], v[:, 0])
        q0 = wrap((ang_v - ang_w) / self.s0)
        # Plano de flexión, relativo a J1
        X = rho - self.c_d[1]
        Y = v[:, 2] - self.c_d[2]
        D = (X ** 2 + Y ** 2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        unreachable = np.abs(D) > 1
        phi = branch * np.arccos(np.clip(D, -1, 1))
        q2 = self.alpha2 - self.alpha1 - phi
        th1 = np.arctan2(Y, X) - np.arctan2(self.L2 * np.sin(phi), self.L1 + self.L2 * np.cos(phi))
        q1 = wrap(th1 - self.alpha1)
        q = np.stack([q0, q1, wrap(q2)], axis=1)
        status[~self.within_limits(q)] = 1
        status[unreachable] = 2
        status[inside] = 3
        return (q[0], status[0]) if np.ndim(p) == 1 else (q, status)

    def ik_best(self, p, q_prev=None):
        """Elige la rama dentro de límites (y la más cercana a q_prev si hay dos)."""
        sols = [self.ik(p, b, c) for c in (1, -1) for b in (self.branch, -self.branch)]
        ok = [(q, s) for q, s in sols if s == 0]
        if not ok:
            return sols[0]
        if q_prev is None or len(ok) == 1:
            return ok[0]
        return min(ok, key=lambda qs: np.linalg.norm(qs[0] - q_prev))

    # ---------------------------------------------------------------- análisis
    def workspace(self, n=31):
        """Muestreo del espacio articular dentro de límites -> (q, p, cerca_de_singular)."""
        g = [np.linspace(lo, hi, n) for lo, hi in self.lim_cad]
        Q = np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)
        P = self.fk(Q)
        near = np.abs(np.sin(self.rel_knee_angle(Q))) < self.sing_thr
        return Q, P, near

    def singularity_report(self):
        """Configuraciones singulares dentro de los límites articulares."""
        out = []
        # Pata estirada / plegada: ángulo relativo 0 o pi
        for k, name in [(0, "pata estirada (fémur y tibia alineados)"), (np.pi, "pata plegada")]:
            q2 = wrap(self.alpha2 - self.alpha1 - k)
            inside = self.lim_cad[2, 0] <= q2 <= self.lim_cad[2, 1]
            out.append({"tipo": name, "q2_cad_rad": float(q2), "q2_enc_rad": float(self.cad_to_enc([0, 0, q2])[2]),
                        "dentro_de_limites": bool(inside)})
        # Pie sobre el eje de la coxa: |xy| = d_ax (rho = 0)
        # Pie sobre el eje de la coxa (componente radial rho = 0): se busca en el espacio articular
        Q, P, _ = self.workspace(41)
        v = P - self.p0
        R0 = rot(self.a0, Q[:, 0])
        rho = np.einsum("nij,j->ni", R0, self.r0)
        rho = np.einsum("ni,ni->n", rho, v)
        out.append({"tipo": "pie sobre el eje de la coxa (rho = 0; la coxa queda indeterminada)",
                    "radio_mm": float(abs(self.d_ax) * 1000),
                    "rho_min_mm": float(rho.min() * 1000), "rho_max_mm": float(rho.max() * 1000),
                    "dentro_de_limites": bool(rho.min() < 0 < rho.max())})
        return out

    def summary(self):
        return {
            "coxa_radial_mm": self.c_d[1] * 1000, "coxa_z_mm": self.c_d[2] * 1000, "coxa_axial_mm": self.c_d[0] * 1000,
            "femur_L1_mm": self.L1 * 1000, "femur_angulo_deg": np.degrees(self.alpha1),
            "tibia_L2_mm": self.L2 * 1000, "tibia_angulo_deg": np.degrees(self.alpha2),
            "desplazamiento_axial_total_mm": self.d_ax * 1000,
        }
