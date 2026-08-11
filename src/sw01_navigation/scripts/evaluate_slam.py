#!/usr/bin/env python3
"""Record synchronized SLAM and ground-truth odometry and report errors."""

import argparse
import csv
import math
from pathlib import Path
import threading
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from trajectory_metrics import (
    align_se2,
    apply_se2_to_poses,
    compute_ate,
    compute_rte,
    synchronize_nearest,
)


# /Odometry 是 FAST-LIO2 在 map 坐标系中的估计；/ground_truth/odom 来自 Gazebo 真值插件。
ESTIMATE_TOPIC = "/Odometry"
GROUND_TRUTH_TOPIC = "/ground_truth/odom"


class TrajectoryRecorder(Node):
    """Collect timestamped planar poses from the estimate and truth topics."""

    def __init__(self):
        super().__init__("slam_trajectory_evaluator")
        self.lock = threading.Lock()
        self.estimate_samples = []
        self.ground_truth_samples = []
        self.create_subscription(
            Odometry,
            ESTIMATE_TOPIC,
            self._estimate_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            GROUND_TRUTH_TOPIC,
            self._ground_truth_callback,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _sample(message):
        stamp = message.header.stamp
        time_seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        return (time_seconds, float(position.x), float(position.y), yaw)

    def _estimate_callback(self, message):
        sample = self._sample(message)
        with self.lock:
            self.estimate_samples.append(sample)

    def _ground_truth_callback(self, message):
        sample = self._sample(message)
        with self.lock:
            self.ground_truth_samples.append(sample)

    def snapshot(self):
        with self.lock:
            return list(self.estimate_samples), list(self.ground_truth_samples)


def _sorted_unique_arrays(samples):
    ordered = sorted(samples, key=lambda sample: sample[0])
    unique = []
    for sample in ordered:
        if unique and sample[0] == unique[-1][0]:
            unique[-1] = sample
        else:
            unique.append(sample)
    values = np.asarray(unique, dtype=float)
    if not unique:
        return np.empty(0, dtype=float), np.empty((0, 3), dtype=float)
    return values[:, 0], values[:, 1:4]


def _write_csv(path, times, estimate, aligned, ground_truth, errors):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "estimate_x",
                "estimate_y",
                "estimate_yaw",
                "aligned_x",
                "aligned_y",
                "aligned_yaw",
                "ground_truth_x",
                "ground_truth_y",
                "ground_truth_yaw",
                "ate_m",
            ]
        )
        for stamp, raw, fitted, truth, error in zip(
            times, estimate, aligned, ground_truth, errors
        ):
            writer.writerow([stamp, *raw, *fitted, *truth, error])


def _write_plot(path, aligned, ground_truth):
    figure, axis = plt.subplots(figsize=(8, 7))
    axis.plot(ground_truth[:, 0], ground_truth[:, 1], label="Ground truth")
    axis.plot(aligned[:, 0], aligned[:, 1], label="FAST-LIO2 (SE(2) aligned)")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("SLAM trajectory comparison")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def _evaluate(estimate_samples, ground_truth_samples, output_dir, tolerance):
    estimate_times, estimate_poses = _sorted_unique_arrays(estimate_samples)
    ground_truth_times, ground_truth_poses = _sorted_unique_arrays(
        ground_truth_samples
    )
    if estimate_times.size == 0 or ground_truth_times.size == 0:
        raise ValueError("both odometry topics must provide samples")

    estimate, ground_truth, matched_times = synchronize_nearest(
        estimate_times,
        estimate_poses,
        ground_truth_times,
        ground_truth_poses,
        tolerance,
    )
    if matched_times.size < 10:
        raise ValueError(
            f"fewer than 10 synchronized samples ({matched_times.size})"
        )

    _, rotation, translation = align_se2(
        estimate[:, :2], ground_truth[:, :2]
    )
    aligned = apply_se2_to_poses(estimate, rotation, translation)
    ate = compute_ate(aligned[:, :2], ground_truth[:, :2])
    rte = compute_rte(matched_times, aligned, ground_truth, delta_s=1.0)

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "trajectory_samples.csv"
    plot_path = output_dir / "trajectory_comparison.png"
    _write_csv(
        csv_path,
        matched_times,
        estimate,
        aligned,
        ground_truth,
        ate["per_sample"],
    )
    _write_plot(plot_path, aligned, ground_truth)
    return ate, rte, matched_times.size, csv_path, plot_path, translation


def _argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    # 默认采集 60 秒、同步容差 0.05 秒，结果写入当前目录的 slam_evaluation。
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output-dir", type=Path, default=Path("slam_evaluation"))
    parser.add_argument("--sync-tolerance", type=float, default=0.05)
    return parser


def main(argv=None):
    parser = _argument_parser()
    args, ros_args = parser.parse_known_args(argv)
    if args.duration <= 0.0 or not math.isfinite(args.duration):
        parser.error("--duration must be positive and finite")
    if args.sync_tolerance <= 0.0 or not math.isfinite(args.sync_tolerance):
        parser.error("--sync-tolerance must be positive and finite")

    rclpy.init(args=ros_args)
    node = TrajectoryRecorder()
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Ctrl-C received; evaluating collected samples")
    finally:
        estimate_samples, ground_truth_samples = node.snapshot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(
        f"Collected estimate={len(estimate_samples)}, "
        f"ground_truth={len(ground_truth_samples)} samples"
    )
    try:
        ate, rte, matched_count, csv_path, plot_path, _ = _evaluate(
            estimate_samples,
            ground_truth_samples,
            args.output_dir,
            args.sync_tolerance,
        )
    except (OSError, ValueError) as error:
        print(f"Evaluation failed: {error}")
        return 1

    print(f"Synchronized samples: {matched_count}")
    print(
        "ATE [m]: "
        f"rmse={ate['rmse']:.6f}, mean={ate['mean']:.6f}, "
        f"median={ate['median']:.6f}, max={ate['max']:.6f}"
    )
    print(
        "RTE (1 s): "
        f"translation_rmse={rte['translation_rmse']:.6f} m, "
        f"translation_mean={rte['translation_mean']:.6f} m, "
        f"yaw_rmse={rte['yaw_rmse']:.6f} rad, pairs={rte['pair_count']}"
    )
    print(f"CSV: {csv_path}")
    print(f"PNG: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
