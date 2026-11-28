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


def test_fk_matches_matlab_model():
    """Mismo punto y pose que el README de cinemática (MATLAB: [220.999, -129.260, 7.519] mm)."""
    k = LegKinematics(load_config(), foot_point_mm=[222.264, -130.525, -21.475])
    p = k.fk(np.radians([0, 20, 15])) * 1000
    assert np.allclose(p, [220.999, -129.260, 7.519], atol=1e-3)


def test_fk_zero_pose_is_measured_foot(kin):
    assert np.allclose(kin.fk(np.zeros(3)) * 1000, [244.830, -109.677, -39.550], atol=1e-3)


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
