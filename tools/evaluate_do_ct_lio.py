#!/usr/bin/env python3

import argparse
import csv
import math
from typing import Dict, Iterable, List, Tuple

import numpy as np
import rosbag


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def metric(values: np.ndarray) -> Dict[str, float]:
    if values.size == 0:
        raise ValueError("empty metric input")
    return {
        "rmse": float(np.sqrt(np.mean(values * values))),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95.0)),
        "max": float(np.max(values)),
    }


def print_metric(label: str, values: np.ndarray, unit: str = "m") -> None:
    m = metric(values)
    print(
        f"{label}: rmse={m['rmse']:.4f} {unit}, mean={m['mean']:.4f} {unit}, "
        f"median={m['median']:.4f} {unit}, p95={m['p95']:.4f} {unit}, "
        f"max={m['max']:.4f} {unit}"
    )


def read_trajectory_csv(path: str) -> np.ndarray:
    rows: List[List[float]] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"time", "x", "y", "z", "qx", "qy", "qz", "qw"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"trajectory CSV missing fields: {sorted(missing)}")
        for row in reader:
            qx = float(row["qx"])
            qy = float(row["qy"])
            qz = float(row["qz"])
            qw = float(row["qw"])
            rows.append(
                [
                    float(row["time"]),
                    float(row["x"]),
                    float(row["y"]),
                    float(row["z"]),
                    yaw_from_quaternion(qx, qy, qz, qw),
                ]
            )
    data = np.asarray(rows, dtype=float)
    if data.size == 0:
        raise ValueError("trajectory CSV is empty")
    data[:, 4] = np.unwrap(data[:, 4])
    return data


def read_ground_truth_from_bag(path: str, topic: str) -> np.ndarray:
    rows: List[List[float]] = []
    with rosbag.Bag(path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            pose = msg.pose.pose
            q = pose.orientation
            rows.append(
                [
                    msg.header.stamp.to_sec(),
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    yaw_from_quaternion(q.x, q.y, q.z, q.w),
                ]
            )
    data = np.asarray(rows, dtype=float)
    if data.size == 0:
        raise ValueError(f"no ground-truth messages found on {topic}")
    data[:, 4] = np.unwrap(data[:, 4])
    return data


def interpolate_ground_truth(gt: np.ndarray, times: np.ndarray) -> np.ndarray:
    gt_times = gt[:, 0]
    return np.column_stack(
        [
            np.interp(times, gt_times, gt[:, 1]),
            np.interp(times, gt_times, gt[:, 2]),
            np.interp(times, gt_times, gt[:, 3]),
            np.interp(times, gt_times, gt[:, 4]),
        ]
    )


def se2_align(src_xy: np.ndarray, dst_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    src_center = src_xy.mean(axis=0)
    dst_center = dst_xy.mean(axis=0)
    src_zero = src_xy - src_center
    dst_zero = dst_xy - dst_center
    u, _, vt = np.linalg.svd(src_zero.T @ dst_zero)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1
        rot = vt.T @ u.T
    trans = dst_center - rot @ src_center
    aligned = (rot @ src_xy.T).T + trans
    return aligned, rot, trans


def rpe_translation_error(traj_xyz: np.ndarray, gt_xyz: np.ndarray) -> np.ndarray:
    if len(traj_xyz) < 2:
        return np.asarray([], dtype=float)
    traj_delta = np.diff(traj_xyz, axis=0)
    gt_delta = np.diff(gt_xyz, axis=0)
    return np.linalg.norm(traj_delta - gt_delta, axis=1)


def pose_jump_count(traj_xyz: np.ndarray, threshold: float) -> int:
    if len(traj_xyz) < 2:
        return 0
    step = np.linalg.norm(np.diff(traj_xyz, axis=0), axis=1)
    return int(np.count_nonzero(step > threshold))


def evaluate(args: argparse.Namespace) -> None:
    traj = read_trajectory_csv(args.trajectory)
    gt = read_ground_truth_from_bag(args.bag, args.ground_truth_topic)

    times = traj[:, 0] + args.time_offset
    overlap = (times >= gt[0, 0]) & (times <= gt[-1, 0])
    matched_ratio = float(np.count_nonzero(overlap)) / float(len(times))
    traj = traj[overlap]
    times = times[overlap]
    if len(traj) == 0:
        raise ValueError("no overlapping timestamps between trajectory and ground truth")

    gt_interp = interpolate_ground_truth(gt, times)
    traj_xyz = traj[:, 1:4]
    gt_xyz = gt_interp[:, 0:3]

    raw_3d = np.linalg.norm(traj_xyz - gt_xyz, axis=1)
    raw_xy = np.linalg.norm(traj_xyz[:, 0:2] - gt_xyz[:, 0:2], axis=1)
    raw_z_abs = np.abs(traj_xyz[:, 2] - gt_xyz[:, 2])
    aligned_xy, rot, trans = se2_align(traj_xyz[:, 0:2], gt_xyz[:, 0:2])
    aligned_xy_error = np.linalg.norm(aligned_xy - gt_xyz[:, 0:2], axis=1)
    rpe = rpe_translation_error(traj_xyz, gt_xyz)

    print(f"bag={args.bag}")
    print(f"trajectory={args.trajectory}")
    print(f"ground_truth_topic={args.ground_truth_topic}")
    print(f"gt_messages={len(gt)}")
    print(f"trajectory_poses={len(times)}")
    print(f"pose_count_match_ratio={matched_ratio:.4f}")
    print(f"time_range={times[0]:.6f}->{times[-1]:.6f}")
    print(
        "start_offset_lio_minus_gt="
        f"dx={traj_xyz[0,0]-gt_xyz[0,0]:.4f}, "
        f"dy={traj_xyz[0,1]-gt_xyz[0,1]:.4f}, "
        f"dz={traj_xyz[0,2]-gt_xyz[0,2]:.4f}, "
        f"norm={raw_3d[0]:.4f} m"
    )
    print_metric("raw_3d_ate", raw_3d)
    print_metric("raw_xy_ate", raw_xy)
    print_metric("raw_z_abs", raw_z_abs)
    print_metric("se2_aligned_xy_ate", aligned_xy_error)
    if rpe.size:
        print_metric("rpe_translation_step", rpe)
    else:
        print("rpe_translation_step: insufficient poses")
    print(f"pose_jumps>{args.jump_threshold:.3f}m={pose_jump_count(traj_xyz, args.jump_threshold)}")
    print(f"se2_rotation_deg={math.degrees(math.atan2(rot[1,0], rot[0,0])):.4f}")
    print(f"se2_translation=tx={trans[0]:.4f}, ty={trans[1]:.4f}")

    raw = metric(raw_3d)
    passed = raw["rmse"] <= args.rmse_threshold and raw["p95"] <= args.p95_threshold
    print(
        "acceptance="
        f"{'PASS' if passed else 'FAIL'} "
        f"(rmse<={args.rmse_threshold:.3f}, p95<={args.p95_threshold:.3f})"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate DO-CT-LIO trajectory against bag ground truth.")
    parser.add_argument("--bag", required=True, help="Validation rosbag path.")
    parser.add_argument("--trajectory", required=True, help="Trajectory CSV path.")
    parser.add_argument("--ground-truth-topic", default="/ground_truth/odom")
    parser.add_argument("--time-offset", type=float, default=0.0)
    parser.add_argument("--jump-threshold", type=float, default=0.5)
    parser.add_argument("--rmse-threshold", type=float, default=0.05)
    parser.add_argument("--p95-threshold", type=float, default=0.05)
    return parser.parse_args()


def main() -> None:
    evaluate(parse_args())


if __name__ == "__main__":
    main()
