import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import load_config  # noqa: E402
from kinematics import LegKinematics  # noqa: E402


@pytest.fixture(scope="module")
def kin():
    return LegKinematics(load_config())


def _legacy_cfg():
    cfg = load_config()
    cfg["geometria"] = {"medicion_fusion": "../../01-mecanica/cinematica-directa-pata/medicion_fusion_u2.json",
                        "cuerpo_pie": "soporte_tibia_A"}
    return cfg


def test_fk_matches_matlab_model():
    """Con la medición anterior (marco raíz de Fusion), mismo punto y pose que el README de
    cinemática: MATLAB da [220.999, -129.260, 7.519] mm."""
    k = LegKinematics(_legacy_cfg(), foot_point_mm=[222.264, -130.525, -21.475])
    p = k.fk(np.radians([0, 20, 15])) * 1000
    assert np.allclose(p, [220.999, -129.260, 7.519], atol=1e-3)


def test_fk_zero_pose_is_foot_sphere_center(kin):
    """En el marco de la pata, q = 0 (estirada) da el centro de la esfera del pie leído de Fusion."""
    assert np.allclose(kin.fk(np.zeros(3)) * 1000, kin.source["pie"]["centro_esfera_mm"], atol=1e-6)


def test_leg_frame_axes_and_signs(kin):
    """Origen en el eje de la coxa; +q0 gira hacia -Y (horario visto desde arriba),
    +q1 sube el pie y +q2 lo baja (convención de Fusion)."""
    assert np.allclose(kin.p0, 0)
    d = lambda i: kin.fk(np.eye(3)[i] * 0.1) - kin.fk(np.zeros(3))
    assert d(0)[1] < 0 and d(1)[2] > 0 and d(2)[2] < 0
    assert np.allclose(kin.p1[1:], kin.p2[1:])          # J1 y J2 a la misma altura (fémur horizontal)


def test_legacy_geometry_still_loads():
    k = LegKinematics(_legacy_cfg())
    assert np.allclose(k.fk(np.zeros(3)) * 1000, [244.830, -109.677, -39.550], atol=1e-3)


def test_fk_ik_roundtrip_whole_workspace(kin):
    """FK(IK(p)) = p con error < 1e-6 m en todo el espacio de trabajo (con la rama correcta)."""
    Q, P, _ = kin.workspace(25)
    best = np.full(len(P), np.inf)
    for c in (1, -1):
        for b in (-1, 1):
            q, s = kin.ik(P, b, c)
            best = np.minimum(best, np.linalg.norm(kin.fk(q) - P, axis=1))
    assert best.max() < kin.cfg["cinematica_inversa"]["tolerancia_fk_ik_m"]


def test_ik_recovers_joint_angles(kin):
    rng = np.random.default_rng(1)
    Q = rng.uniform(kin.lim_cad[:, 0], kin.lim_cad[:, 1], (2000, 3))
    P = kin.fk(Q)
    rec = np.zeros(len(Q), bool)
    for c in (1, -1):              # coxa: pie por delante / por detrás de su eje
        for b in (-1, 1):          # codo
            q, _ = kin.ik(P, b, c)
            rec |= np.abs(q - Q).max(1) < 1e-7
    # Solo fallan los puntos pegados a la singularidad de pata estirada
    far = np.abs(np.sin(kin.rel_knee_angle(Q))) > 1e-3
    assert rec[far].all()


def test_unreachable_points_are_flagged(kin):
    far = kin.p0 + np.array([1.0, -1.0, 0.0])
    _, s = kin.ik(far)
    assert s == 2


def test_encoder_cad_mapping_roundtrip(kin):
    q = np.array([0.3, -0.4, 0.9])
    assert np.allclose(kin.cad_to_enc(kin.enc_to_cad(q)), q)
