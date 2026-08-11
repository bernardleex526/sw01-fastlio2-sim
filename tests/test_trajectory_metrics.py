import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "src" / "sw01_navigation" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import trajectory_metrics as metrics


def test_se2_alignment_and_ate_remove_initial_frame_offset():
    gt = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 1.0], [3.0, 1.0]])
    angle = np.deg2rad(30.0)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    estimate = gt @ rotation.T + np.array([5.0, -2.0])

    aligned, _, _ = metrics.align_se2(estimate, gt)
    stats = metrics.compute_ate(aligned, gt)

    assert np.allclose(aligned, gt, atol=1e-9)
    assert stats["rmse"] < 1e-9


@pytest.mark.parametrize(
    ("estimate", "ground_truth", "message"),
    [
        (
            np.array(
                [[2.0, -1.0], [2.0 + 1e-12, -1.0], [2.0, -1.0 + 1e-12]]
            ),
            np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]),
            "estimate_xy spatial span is too small",
        ),
        (
            np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]),
            np.array(
                [[3.0, 4.0], [3.0 + 1e-12, 4.0], [3.0, 4.0 + 1e-12]]
            ),
            "ground_truth_xy spatial span is too small",
        ),
    ],
)
def test_se2_alignment_rejects_trajectories_with_near_zero_span(
    estimate, ground_truth, message
):
    with pytest.raises(ValueError, match=message):
        metrics.align_se2(estimate, ground_truth)


def test_se2_alignment_allows_a_nonzero_straight_trajectory():
    estimate = np.array([[2.0, 3.0], [2.0, 4.0], [2.0, 5.0]])
    ground_truth = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])

    aligned, _, _ = metrics.align_se2(estimate, ground_truth)

    assert np.allclose(aligned, ground_truth, atol=1e-12)


def test_alignment_returns_proper_rotation_when_data_suggests_reflection():
    estimate = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 2.0]])
    reflected = estimate * np.array([-1.0, 1.0])
    root_13 = np.sqrt(13.0)
    expected_rotation = np.array(
        [[3.0 / root_13, 2.0 / root_13], [-2.0 / root_13, 3.0 / root_13]]
    )
    expected_translation = np.array(
        [
            -(root_13 + 7.0) / (3.0 * root_13),
            (2.0 * root_13 - 4.0) / (3.0 * root_13),
        ]
    )
    expected_aligned = np.array(
        [
            expected_translation,
            [
                (2.0 - root_13) / (3.0 * root_13),
                (2.0 * root_13 - 10.0) / (3.0 * root_13),
            ],
            [
                (5.0 - root_13) / (3.0 * root_13),
                (2.0 * root_13 + 14.0) / (3.0 * root_13),
            ],
        ]
    )

    aligned, rotation, translation = metrics.align_se2(estimate, reflected)

    assert np.isclose(np.linalg.det(rotation), 1.0)
    assert np.allclose(rotation, expected_rotation, atol=1e-12)
    assert np.allclose(translation, expected_translation, atol=1e-12)
    assert np.allclose(aligned, expected_aligned, atol=1e-12)
    assert np.linalg.norm(aligned - reflected) > 0.5


def test_apply_se2_to_poses_transforms_xy_and_yaw_in_the_same_direction():
    poses = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 3.0 * np.pi / 4.0]])
    rotation = np.array([[0.0, -1.0], [1.0, 0.0]])
    translation = np.array([10.0, -2.0])

    transformed = metrics.apply_se2_to_poses(poses, rotation, translation)

    assert np.allclose(
        transformed,
        [[10.0, -1.0, np.pi / 2.0], [8.0, -2.0, -3.0 * np.pi / 4.0]],
        atol=1e-12,
    )


def test_nearest_sync_rejects_samples_outside_tolerance():
    est_times = np.array([0.00, 1.00, 2.00])
    gt_times = np.array([0.02, 1.20, 2.01])

    estimate, ground_truth, times = metrics.synchronize_nearest(
        est_times,
        np.zeros((3, 3)),
        gt_times,
        np.ones((3, 3)),
        0.05,
    )

    assert estimate.shape[0] == 2
    assert ground_truth.shape[0] == 2
    assert times.shape[0] == 2
    assert np.allclose(times, [0.0, 2.0])


def test_nearest_sync_uses_each_ground_truth_sample_at_most_once():
    estimate, ground_truth, times = metrics.synchronize_nearest(
        np.array([0.00, 0.03]),
        np.array([[10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]),
        np.array([0.02]),
        np.array([[30.0, 0.0, 0.0]]),
        0.05,
    )

    assert estimate.tolist() == [[10.0, 0.0, 0.0]]
    assert ground_truth.tolist() == [[30.0, 0.0, 0.0]]
    assert times.tolist() == [0.0]


def test_nearest_sync_never_crosses_back_to_an_earlier_ground_truth_sample():
    _, ground_truth, _ = metrics.synchronize_nearest(
        np.array([0.099, 0.101]),
        np.zeros((2, 3)),
        np.array([0.080, 0.100]),
        np.array([[0.080, 0.0, 0.0], [0.100, 0.0, 0.0]]),
        0.05,
    )

    matched_ground_truth_times = ground_truth[:, 0]
    assert np.all(np.diff(matched_ground_truth_times) > 0.0)
    assert np.unique(matched_ground_truth_times).size == ground_truth.shape[0]


def test_rte_is_zero_for_identical_relative_motion():
    times = np.arange(0.0, 5.0, 0.5)
    poses = np.column_stack((times, 0.2 * times, 0.1 * times))

    stats = metrics.compute_rte(times, poses, poses, delta_s=1.0)

    assert stats["translation_rmse"] < 1e-12
    assert stats["yaw_rmse"] < 1e-12


def test_rte_uses_full_se2_relative_motion_and_wraps_yaw():
    times = np.array([0.0, 1.0, 2.0])
    ground_truth = np.array(
        [[0.0, 0.0, 3.10], [1.0, 0.0, -3.10], [1.0, 1.0, -2.90]]
    )
    angle = 0.7
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    estimate = ground_truth.copy()
    estimate[:, :2] = ground_truth[:, :2] @ rotation.T + np.array([4.0, -3.0])
    estimate[:, 2] = metrics.wrap_angle(ground_truth[:, 2] + angle)

    stats = metrics.compute_rte(times, estimate, ground_truth, delta_s=1.0)

    assert stats["translation_rmse"] < 1e-12
    assert stats["yaw_rmse"] < 1e-12
    assert stats["pair_count"] == 2


def test_rte_rejects_sparse_pairs_far_from_requested_interval():
    times = np.array([0.0, 0.1, 10.0])
    poses = np.column_stack((times, np.zeros(3), np.zeros(3)))

    with pytest.raises(ValueError, match="within 10%"):
        metrics.compute_rte(times, poses, poses, delta_s=1.0)


def test_ate_returns_hand_checked_statistics_and_per_sample_errors():
    stats = metrics.compute_ate(
        np.array([[0.0, 0.0], [4.0, 0.0]]),
        np.array([[0.0, 0.0], [1.0, 0.0]]),
    )

    assert stats["rmse"] == pytest.approx(np.sqrt(4.5))
    assert stats["mean"] == pytest.approx(1.5)
    assert stats["median"] == pytest.approx(1.5)
    assert stats["max"] == pytest.approx(3.0)
    assert stats["per_sample"].tolist() == [0.0, 3.0]


@pytest.mark.parametrize(
    ("call", "message"),
    [
        (
            lambda: metrics.synchronize_nearest(
                [0.0], [[0.0, 0.0, 0.0]], [0.0], [[0.0, 0.0, 0.0]], 0.0
            ),
            "max_dt must be positive",
        ),
        (
            lambda: metrics.synchronize_nearest(
                [0.0, 0.0],
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                [0.0],
                [[0.0, 0.0, 0.0]],
                0.05,
            ),
            "est_times must be strictly increasing",
        ),
        (
            lambda: metrics.synchronize_nearest(
                [0.0], [[0.0, 0.0]], [0.0], [[0.0, 0.0]], 0.05
            ),
            "3 columns",
        ),
        (
            lambda: metrics.align_se2([[0.0, 0.0]], [[0.0, 0.0]]),
            "at least 2 samples",
        ),
        (
            lambda: metrics.compute_ate(
                [[0.0, np.nan], [1.0, 0.0]], [[0.0, 0.0], [1.0, 0.0]]
            ),
            "finite",
        ),
        (
            lambda: metrics.compute_rte(
                [0.0, 1.0],
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                delta_s=-1.0,
            ),
            "delta_s must be positive",
        ),
        (
            lambda: metrics.compute_rte(
                [0.0], [[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]]
            ),
            "at least 2 samples",
        ),
    ],
)
def test_invalid_metric_inputs_raise_descriptive_value_errors(call, message):
    with pytest.raises(ValueError, match=message):
        call()


def test_wrap_angle_maps_to_half_open_interval():
    wrapped = metrics.wrap_angle(np.array([-np.pi, np.pi, 3.0 * np.pi, 0.25]))

    assert np.allclose(wrapped, [-np.pi, -np.pi, -np.pi, 0.25])
    assert np.all(wrapped >= -np.pi)
    assert np.all(wrapped < np.pi)
