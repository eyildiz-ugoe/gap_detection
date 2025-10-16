import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

import helpers


def test_calculate_convex_hulls_and_centers_returns_correct_sizes():
    width = 0.02
    depth = 0.03
    height = 0.01
    center = np.array([0.05, 0.1, 0.2])

    offsets = np.array([
        [-width / 2, -depth / 2, -height / 2],
        [-width / 2, -depth / 2, height / 2],
        [-width / 2, depth / 2, -height / 2],
        [-width / 2, depth / 2, height / 2],
        [width / 2, -depth / 2, -height / 2],
        [width / 2, -depth / 2, height / 2],
        [width / 2, depth / 2, -height / 2],
        [width / 2, depth / 2, height / 2],
    ])

    prism = center + offsets

    info = helpers.calculate_convex_hulls_and_centers([prism])[0]
    returned_center, _, _, _, size, num_points = info

    assert np.allclose(returned_center, center)
    assert np.isclose(size[0], width / 2)
    assert np.isclose(size[1], depth / 2)
    assert np.isclose(size[2], height / 2)
    assert num_points == prism.shape[0]
