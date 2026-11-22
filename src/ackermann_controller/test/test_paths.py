import math

import numpy as np

from ackermann_controller import paths


def test_figure8_feasible_for_robot():
    p = paths.resample(paths.align_to_start(paths.figure8(3.5, 1.5)), 0.05, True)
    r_min_robot = 0.1339 / math.tan(math.radians(18))  # 0.412 m
    assert 1.0 / paths.max_curvature(p) > r_min_robot
    assert np.allclose(p[0], [0.0, 0.0])


def test_oval_starts_at_origin_heading_x():
    p = paths.resample(paths.oval(3.0, 1.2), 0.05, True)
    assert np.allclose(p[0], [0.0, 0.0])
    assert abs(paths.headings(p)[1]) < 1e-6
