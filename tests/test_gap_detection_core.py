import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

import gap_detection_core as core


def _surface_points(size=5, spacing=0.02):
    axis = np.linspace(-spacing, spacing, size)
    grid_x, grid_y = np.meshgrid(axis, axis)
    zeros = np.zeros(grid_x.size)
    return np.column_stack((grid_x.ravel(), grid_y.ravel(), zeros))


def _gap_cluster(center, scale=0.004):
    offsets = np.array([
        [-scale, -scale, 0.0],
        [-scale, scale, 0.0],
        [scale, -scale, 0.0],
        [scale, scale, 0.0],
        [0.0, 0.0, scale],
        [scale / 2, 0.0, scale / 2],
    ])
    return center + offsets


def test_detect_gaps_single_cluster():
    surface = _surface_points()
    gap_center = np.array([0.0, 0.0, 0.02])
    gap_points = _gap_cluster(gap_center)

    cloud = np.vstack([surface, gap_points])

    detector_params = core.DetectorParameters(depth_axis=2, manual_threshold=0.005)
    clustering_params = core.ClusteringParameters(
        method=core.ClusteringMethod.DBSCAN,
        dbscan_eps=0.01,
        dbscan_min_samples=1,
    )

    result = core.detect_gaps(cloud, detector_params, clustering_params)

    assert result.threshold == detector_params.manual_threshold
    assert np.isclose(result.surface_height, 0.0)
    assert result.potential_gap_points.shape[0] == gap_points.shape[0]
    assert len(result.gaps) == 1

    gap = result.gaps[0]
    assert gap.original_points == gap_points.shape[0]
    assert gap.total_points == gap.original_points + gap.volume_correction_points
    assert gap.volume > 0
    assert gap.volume_cm3 > 0


def test_detect_gaps_requires_minimum_points():
    surface = _surface_points()
    gap_center = np.array([0.0, 0.0, 0.02])
    gap_points = gap_center + np.array([
        [-0.001, 0.0, 0.0],
        [0.001, 0.0, 0.0],
        [0.0, 0.001, 0.001],
    ])

    cloud = np.vstack([surface, gap_points])

    detector_params = core.DetectorParameters(depth_axis=2, manual_threshold=0.005)
    clustering_params = core.ClusteringParameters(
        method=core.ClusteringMethod.DBSCAN,
        dbscan_eps=0.01,
        dbscan_min_samples=1,
    )

    result = core.detect_gaps(cloud, detector_params, clustering_params)

    assert result.gaps == []
    assert result.potential_gap_points.shape[0] == gap_points.shape[0]
