"""Estabilidad de cada lazo (análisis lineal) y comportamiento de la réplica del firmware."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import experiments as ex  # noqa: E402
import loop_analysis as la  # noqa: E402
from cascade_control import CurrentPI, SpeedPI  # noqa: E402
from common import load_config  # noqa: E402

CFG = load_config()


@pytest.mark.parametrize("j", [0, 1, 2])
def test_current_loop_discrete_poles_inside_unit_circle(j):
    assert np.all(np.abs(la.discrete_current_poles(CFG, j)) < 1)


@pytest.mark.parametrize("j", [0, 1, 2])
@pytest.mark.parametrize("loop", ["corriente", "velocidad", "posicion"])
def test_each_loop_has_positive_margins(j, loop):
    w, L, _ = la.loops(CFG, j)
    m = la.margins(w, L[loop][0])
    assert m["pm_deg"] > 0 and m["gm"] > 1, m


@pytest.mark.parametrize("j", [0, 1, 2])
def test_frequency_separation_order(j):
    s = la.summary(CFG, j)
    assert s["corriente"]["bw_hz"] > s["velocidad"]["bw_hz"] > s["posicion"]["bw_hz"]


@pytest.mark.parametrize("j", [0, 1, 2])
def test_position_step_reaches_tolerance_band(j):
    r, m = ex.position_step(CFG, j)
    assert m["ess"] <= CFG["control"]["tolerancia_rad"][j] + 1e-9
    assert np.isfinite(m["ts"])


@pytest.mark.parametrize("j", [0, 1, 2])
def test_current_step_settles(j):
    _, m = ex.current_step(CFG, j)
    assert m["ess"] < 1e-3 and m["ts"] < 0.05


def test_current_pi_antiwindup_matches_firmware():
    """Con la salida saturada y error positivo el integrador no crece (current_pi.h:63-65)."""
    p = CurrentPI(kp=100.0, ki=1e5, ref=1.0)
    p.update(0.0, 1000)
    for k in range(2, 20):
        p.update(0.0, k * 1000)
    assert p.output == 100.0 and p.integrator == 0.0


def test_speed_pi_output_clamped_to_ilimit():
    s = SpeedPI(kp=1.0, ki=0.0, limit=0.1, ref=10.0)
    s.update(0.0, 10000, False)
    assert s.output == pytest.approx(0.1) and s.limited
