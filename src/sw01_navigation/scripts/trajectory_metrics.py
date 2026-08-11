"""Pure NumPy trajectory synchronization and SE(2) error metrics."""

import numpy as np


def _finite_array(value, name, dimensions):
    array = np.asarray(value, dtype=float)
    if array.ndim != dimensions:
        raise ValueError(f"{name} must be a {dimensions}-D array")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _strict_times(value, name, minimum=1):
    times = _finite_array(value, name, 1)
    if times.size < minimum:
        raise ValueError(f"{name} must contain at least {minimum} samples")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError(f"{name} must be strictly increasing")
    return times


def _paired_arrays(first, second, first_name, second_name, columns, minimum):
    first_array = _finite_array(first, first_name, 2)
    second_array = _finite_array(second, second_name, 2)
    if first_array.shape != second_array.shape:
        raise ValueError(f"{first_name} and {second_name} must have matching shapes")
    if first_array.shape[1] != columns:
        raise ValueError(f"{first_name} and {second_name} must have {columns} columns")
    if first_array.shape[0] < minimum:
        raise ValueError(
            f"{first_name} and {second_name} must contain at least {minimum} samples"
        )
    return first_array, second_array


def synchronize_nearest(est_times, est_poses, gt_times, gt_poses, max_dt):
    """Return (matched_est, matched_gt, matched_times) using nearest unique GT samples."""
    if not np.isscalar(max_dt) or not np.isfinite(max_dt) or max_dt <= 0.0:
        raise ValueError("max_dt must be positive and finite")

    estimate_times = _strict_times(est_times, "est_times")
    ground_truth_times = _strict_times(gt_times, "gt_times")
    estimate_poses = _finite_array(est_poses, "est_poses", 2)
    ground_truth_poses = _finite_array(gt_poses, "gt_poses", 2)
    if estimate_poses.shape[0] != estimate_times.size:
        raise ValueError("est_poses length must match est_times")
    if ground_truth_poses.shape[0] != ground_truth_times.size:
        raise ValueError("gt_poses length must match gt_times")
    if estimate_poses.shape[1] != 3 or ground_truth_poses.shape[1] != 3:
        raise ValueError("est_poses and gt_poses must have 3 columns [x, y, yaw]")

    estimate_indices = []
    ground_truth_indices = []
    used_ground_truth = set()
    for estimate_index, stamp in enumerate(estimate_times):
        order = np.argsort(np.abs(ground_truth_times - stamp), kind="stable")
        for ground_truth_index in order:
            index = int(ground_truth_index)
            difference = abs(ground_truth_times[index] - stamp)
            if difference > max_dt:
                break
            if index not in used_ground_truth:
                estimate_indices.append(estimate_index)
                ground_truth_indices.append(index)
                used_ground_truth.add(index)
                break

    return (
        estimate_poses[estimate_indices].copy(),
        ground_truth_poses[ground_truth_indices].copy(),
        estimate_times[estimate_indices].copy(),
    )


def align_se2(estimate_xy, ground_truth_xy):
    """Return (aligned_xy, rotation_2x2, translation_2) using SVD without scale."""
    estimate, ground_truth = _paired_arrays(
        estimate_xy,
        ground_truth_xy,
        "estimate_xy",
        "ground_truth_xy",
        columns=2,
        minimum=2,
    )

    estimate_center = np.mean(estimate, axis=0)
    ground_truth_center = np.mean(ground_truth, axis=0)
    covariance = (estimate - estimate_center).T @ (
        ground_truth - ground_truth_center
    )
    left, _, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T

    translation = ground_truth_center - estimate_center @ rotation.T
    aligned = estimate @ rotation.T + translation
    return aligned, rotation, translation


def compute_ate(aligned_xy, ground_truth_xy):
    """Return dict with rmse, mean, median, max and per_sample."""
    aligned, ground_truth = _paired_arrays(
        aligned_xy,
        ground_truth_xy,
        "aligned_xy",
        "ground_truth_xy",
        columns=2,
        minimum=1,
    )
    errors = np.linalg.norm(aligned - ground_truth, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(errors)))),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "max": float(np.max(errors)),
        "per_sample": errors,
    }


def _relative_pose(poses, start, end):
    delta_position = poses[end, :2] - poses[start, :2]
    yaw = poses[start, 2]
    inverse_start_rotation = np.array(
        [[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]
    )
    translation = inverse_start_rotation @ delta_position
    delta_yaw = wrap_angle(poses[end, 2] - yaw)
    return translation, delta_yaw


def compute_rte(times, estimate_xyyaw, ground_truth_xyyaw, delta_s=1.0):
    """Return translation_rmse, translation_mean, yaw_rmse, pair_count."""
    if not np.isscalar(delta_s) or not np.isfinite(delta_s) or delta_s <= 0.0:
        raise ValueError("delta_s must be positive and finite")

    sample_times = _strict_times(times, "times", minimum=2)
    estimate, ground_truth = _paired_arrays(
        estimate_xyyaw,
        ground_truth_xyyaw,
        "estimate_xyyaw",
        "ground_truth_xyyaw",
        columns=3,
        minimum=2,
    )
    if estimate.shape[0] != sample_times.size:
        raise ValueError("pose lengths must match times")

    translation_errors = []
    yaw_errors = []
    for start, stamp in enumerate(sample_times[:-1]):
        target = stamp + delta_s
        if target > sample_times[-1]:
            continue
        insertion = int(np.searchsorted(sample_times, target, side="left"))
        candidates = [insertion]
        if insertion - 1 > start:
            candidates.append(insertion - 1)
        end = min(candidates, key=lambda index: abs(sample_times[index] - target))

        estimate_translation, estimate_yaw = _relative_pose(estimate, start, end)
        ground_truth_translation, ground_truth_yaw = _relative_pose(
            ground_truth, start, end
        )
        translation_errors.append(
            np.linalg.norm(estimate_translation - ground_truth_translation)
        )
        yaw_errors.append(abs(wrap_angle(estimate_yaw - ground_truth_yaw)))

    if not translation_errors:
        raise ValueError("times must span at least delta_s to form a relative pair")

    translation_errors = np.asarray(translation_errors)
    yaw_errors = np.asarray(yaw_errors)
    return {
        "translation_rmse": float(
            np.sqrt(np.mean(np.square(translation_errors)))
        ),
        "translation_mean": float(np.mean(translation_errors)),
        "yaw_rmse": float(np.sqrt(np.mean(np.square(yaw_errors)))),
        "pair_count": int(translation_errors.size),
    }


def wrap_angle(angle):
    """Map radians to [-pi, pi)."""
    values = np.asarray(angle)
    wrapped = (values + np.pi) % (2.0 * np.pi) - np.pi
    if np.isscalar(angle):
        return float(wrapped)
    return wrapped
