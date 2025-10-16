"""Core gap detection logic that can be tested without ROS.

The original implementation in :mod:`gap_detector` mixes the numerical
processing of point clouds with ROS communication primitives.  This coupling
makes it difficult to test the detection behaviour in isolation.  The helpers
in this module provide a small abstraction that operates purely on NumPy
arrays.  The ROS facing code can delegate the heavy lifting to these functions
while unit tests can exercise the behaviour directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, List, Optional, Sequence

import numpy as np
from scipy.spatial import ConvexHull, QhullError
from skimage.filters import (
    threshold_li,
    threshold_minimum,
    threshold_otsu,
    threshold_yen,
)
from sklearn import cluster

import helpers

try:  # pragma: no cover - optional dependency
    import hdbscan  # type: ignore
except ImportError:  # pragma: no cover - allow running without the package
    hdbscan = None


class ClusteringMethod(Enum):
    """Supported clustering algorithms."""

    KMEANS = 0
    BIRCH = 1
    DBSCAN = 2
    HDBSCAN = 3

    @classmethod
    def from_value(cls, value: int | "ClusteringMethod") -> "ClusteringMethod":
        if isinstance(value, ClusteringMethod):
            return value
        try:
            return ClusteringMethod(value)
        except ValueError as exc:  # pragma: no cover - defensive programming
            raise ValueError(f"Unsupported clustering method: {value!r}") from exc


@dataclass
class DetectorParameters:
    """Configuration for the geometric part of the detector."""

    depth_axis: int = 2
    automatic_thresholding: int = 3
    otsu_bins: int = 256
    manual_threshold: Optional[float] = None


@dataclass
class ClusteringParameters:
    """Parameters for the clustering stage."""

    method: ClusteringMethod = ClusteringMethod.DBSCAN
    kmeans_clusters: int = 2
    birch_branching_factor: int = 50
    birch_threshold: float = 0.5
    dbscan_eps: float = 0.01
    dbscan_min_samples: int = 5
    hdbscan_min_cluster_size: int = 5


@dataclass
class GapInfo:
    """Represents a single detected gap."""

    center: Sequence[float]
    vertices: Sequence[Sequence[float]]
    simplices: Sequence[Sequence[Sequence[float]]]
    volume: float
    size: Sequence[float]
    total_points: int
    original_points: int
    volume_correction_points: int

    @property
    def volume_cm3(self) -> float:
        """Return the volume in cubic centimetres for convenience."""

        return self.volume * 1_000_000.0


@dataclass
class GapDetectionResult:
    """Result of a gap detection run."""

    gaps: List[GapInfo]
    threshold: float
    surface_height: float
    potential_gap_points: np.ndarray


class GapDetectionError(RuntimeError):
    """Raised when the detector is unable to continue."""


def detect_gaps(
    points: Iterable[Sequence[float]] | np.ndarray,
    detector_params: DetectorParameters,
    clustering_params: ClusteringParameters,
) -> GapDetectionResult:
    """Detect gaps in the given point cloud.

    Parameters
    ----------
    points:
        Sequence of 3D points.
    detector_params:
        Parameters that control the thresholding stage.
    clustering_params:
        Configuration of the clustering stage.
    """

    np_points = _normalise_points(points)

    if np_points.size == 0:
        return GapDetectionResult([], float("nan"), float("nan"), np_points)

    depth_axis = detector_params.depth_axis
    depth_values = np_points[:, depth_axis]

    threshold = _determine_threshold(depth_values, detector_params)

    device_surface_mask = depth_values <= threshold
    device_surface_points = np_points[device_surface_mask]
    if device_surface_points.size == 0:
        surface_height = float(np.median(depth_values))
    else:
        surface_height = float(np.median(device_surface_points[:, depth_axis]))

    potential_gap_points = np_points[~device_surface_mask]
    if potential_gap_points.size == 0:
        return GapDetectionResult([], threshold, surface_height, potential_gap_points)

    labels = _cluster_points(potential_gap_points, clustering_params)
    unique_labels = [label for label in sorted(set(labels)) if label != -1]

    volume_corrected_clusters: List[np.ndarray] = []
    original_counts: List[int] = []
    volume_correction_counts: List[int] = []

    for label in unique_labels:
        cluster_points = potential_gap_points[labels == label]
        if len(cluster_points) < 4:
            continue
        try:
            corrected_cluster, added_points = _apply_volume_correction(
                cluster_points, depth_axis, surface_height
            )
        except QhullError as exc:
            raise GapDetectionError("Failed to construct convex hull for cluster") from exc

        volume_corrected_clusters.append(corrected_cluster)
        original_counts.append(len(cluster_points))
        volume_correction_counts.append(added_points)

    if not volume_corrected_clusters:
        return GapDetectionResult([], threshold, surface_height, potential_gap_points)

    convex_infos = helpers.calculate_convex_hulls_and_centers(volume_corrected_clusters)

    gaps = [
        GapInfo(
            center=info[0],
            vertices=info[1],
            simplices=info[2],
            volume=info[3],
            size=info[4],
            total_points=info[5],
            original_points=orig,
            volume_correction_points=added,
        )
        for info, orig, added in zip(convex_infos, original_counts, volume_correction_counts)
    ]

    return GapDetectionResult(gaps, threshold, surface_height, potential_gap_points)


def _normalise_points(points: Iterable[Sequence[float]] | np.ndarray) -> np.ndarray:
    np_points = np.asarray(points, dtype=float)

    if np_points.ndim == 1:
        if np_points.size % 3 != 0:
            raise ValueError("Expected points with three coordinates each.")
        np_points = np_points.reshape((-1, 3))
    elif np_points.ndim != 2:
        raise ValueError("Point array must be two-dimensional.")

    if np_points.shape[1] != 3:
        raise ValueError("Points must have exactly three coordinates.")

    return np_points.astype(float, copy=False)


def _determine_threshold(depth_values: np.ndarray, params: DetectorParameters) -> float:
    if params.manual_threshold is not None:
        return float(params.manual_threshold)

    strategy = params.automatic_thresholding

    if strategy == 0:
        func = threshold_minimum
        kwargs = {}
    elif strategy == 1:
        func = threshold_li
        kwargs = {}
    elif strategy == 2:
        func = threshold_yen
        kwargs = {}
    elif strategy == 3:
        func = threshold_otsu
        kwargs = {"nbins": params.otsu_bins}
    else:  # pragma: no cover - defensive programming
        raise ValueError(f"Automatic threshold value out of bounds: {strategy!r}")

    if depth_values.size == 0:
        raise GapDetectionError("Cannot determine threshold without depth values.")

    try:
        threshold = func(depth_values, **kwargs)
    except RuntimeError as exc:
        raise GapDetectionError("Automatic thresholding failed.") from exc

    return float(threshold)


def _cluster_points(points: np.ndarray, params: ClusteringParameters) -> np.ndarray:
    method = ClusteringMethod.from_value(params.method)

    if method == ClusteringMethod.KMEANS:
        estimator = cluster.KMeans(n_clusters=params.kmeans_clusters)
    elif method == ClusteringMethod.BIRCH:
        estimator = cluster.Birch(
            branching_factor=params.birch_branching_factor,
            threshold=params.birch_threshold,
            n_clusters=None,
            compute_labels=True,
        )
    elif method == ClusteringMethod.DBSCAN:
        estimator = cluster.DBSCAN(eps=params.dbscan_eps, min_samples=params.dbscan_min_samples)
    elif method == ClusteringMethod.HDBSCAN:
        if hdbscan is None:  # pragma: no cover - optional dependency
            raise GapDetectionError("HDBSCAN is not available. Please install the 'hdbscan' package.")
        estimator = hdbscan.HDBSCAN(min_cluster_size=params.hdbscan_min_cluster_size)
    else:  # pragma: no cover - defensive programming
        raise GapDetectionError(f"Unsupported clustering method: {method}")

    labels = estimator.fit_predict(points)
    return np.asarray(labels)


def _apply_volume_correction(
    cluster_points: np.ndarray,
    depth_axis: int,
    surface_height: float,
) -> tuple[np.ndarray, int]:
    hull = ConvexHull(cluster_points, qhull_options="QJ")

    corrected = cluster_points.tolist()
    added_points = 0

    for vertex_index in hull.vertices:
        vertex = cluster_points[vertex_index]
        if depth_axis == 0:
            volume_point = [surface_height, vertex[1], vertex[2]]
        elif depth_axis == 1:
            volume_point = [vertex[0], surface_height, vertex[2]]
        elif depth_axis == 2:
            volume_point = [vertex[0], vertex[1], surface_height]
        else:  # pragma: no cover - defensive programming
            raise GapDetectionError(f"Unsupported depth axis: {depth_axis}")

        corrected.append(volume_point)
        added_points += 1

    return np.asarray(corrected, dtype=float), added_points
