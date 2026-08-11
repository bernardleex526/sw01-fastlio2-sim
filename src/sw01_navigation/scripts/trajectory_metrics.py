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
    last_ground_truth_index = -1
    for estimate_index, stamp in enumerate(estimate_times):
        candidate_times = ground_truth_times[last_ground_truth_index + 1 :]
        if candidate_times.size == 0:
            break
        relative_index = int(np.argmin(np.abs(candidate_times - stamp)))
        index = last_ground_truth_index + 1 + relative_index
        if abs(ground_truth_times[index] - stamp) <= max_dt:
            estimate_indices.append(estimate_index)
            ground_truth_indices.append(index)
            last_ground_truth_index = index

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
    centered_estimate = estimate - estimate_center
    centered_ground_truth = ground_truth - ground_truth_center
    minimum_span = np.sqrt(np.finfo(float).eps)
    if np.linalg.norm(centered_estimate) <= minimum_span:
        raise ValueError("estimate_xy spatial span is too small for SE(2) alignment")
    if np.linalg.norm(centered_ground_truth) <= minimum_span:
        raise ValueError(
            "ground_truth_xy spatial span is too small for SE(2) alignment"
        )

    covariance = centered_estimate.T @ centered_ground_truth
    left, _, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T

    translation = ground_truth_center - estimate_center @ rotation.T
    aligned = estimate @ rotation.T + translation
    return aligned, rotation, translation


def apply_se2_to_poses(poses_xyyaw, rotation_2x2, translation_2):
    """Apply one SE(2) transform to pose positions and yaw angles."""
    poses = _finite_array(poses_xyyaw, "poses_xyyaw", 2)
    rotation = _finite_array(rotation_2x2, "rotation_2x2", 2)
    translation = _finite_array(translation_2, "translation_2", 1)
    if poses.shape[1] != 3:
        raise ValueError("poses_xyyaw must have 3 columns [x, y, yaw]")
    if rotation.shape != (2, 2):
        raise ValueError("rotation_2x2 must have shape (2, 2)")
    if translation.shape != (2,):
        raise ValueError("translation_2 must have shape (2,)")

    transformed = poses.copy()
    transformed[:, :2] = poses[:, :2] @ rotation.T + translation
    alignment_yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    transformed[:, 2] = wrap_angle(poses[:, 2] + alignment_yaw)
    return transformed


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
        interval = sample_times[end] - stamp
        epsilon = np.finfo(float).eps * max(1.0, abs(interval), abs(delta_s))
        if abs(interval - delta_s) > 0.1 * delta_s + epsilon:
            continue

        estimate_translation, estimate_yaw = _relative_pose(estimate, start, end)
        ground_truth_translation, ground_truth_yaw = _relative_pose(
            ground_truth, start, end
        )
        translation_errors.append(
            np.linalg.norm(estimate_translation - ground_truth_translation)
        )
        yaw_errors.append(abs(wrap_angle(estimate_yaw - ground_truth_yaw)))

    if not translation_errors:
        raise ValueError("no sample pair is within 10% of delta_s")

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
