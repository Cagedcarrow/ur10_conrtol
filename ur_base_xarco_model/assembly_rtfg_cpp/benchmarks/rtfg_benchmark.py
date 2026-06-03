#!/usr/bin/env python3
import csv
import json
import argparse
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory

from assembly_rtfg_cpp.srv import FitPreview, LoadConfig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", default="", help="runtime yaml path, empty means package default")
    parser.add_argument("--output-tag", default="latest", help="output suffix, e.g. safe or diagnostic")
    parser.add_argument("--clearance-threshold", type=float, default=0.002)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("rtfg_benchmark_client")
    load_cli = node.create_client(LoadConfig, "/rtfg/load_config")
    fit_cli = node.create_client(FitPreview, "/rtfg/fit_preview")
    if not load_cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("/rtfg/load_config unavailable")
    if not fit_cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("/rtfg/fit_preview unavailable")

    load_req = LoadConfig.Request()
    load_req.yaml_path = args.yaml
    load_future = load_cli.call_async(load_req)
    rclpy.spin_until_future_complete(node, load_future, timeout_sec=10.0)
    cfg = load_future.result()
    if cfg is None:
        raise RuntimeError("/rtfg/load_config timeout or unavailable response")
    if not cfg.success:
        raise RuntimeError(cfg.message)

    fit_req = FitPreview.Request()
    fit_req.left_wall_offset = cfg.left_wall_offset
    fit_req.mud_height = cfg.mud_height
    fit_req.approach_len = cfg.approach_len
    fit_req.theta_deg = cfg.theta_deg
    fit_req.depth = cfg.depth
    fit_req.x_plane = cfg.x_plane
    fit_req.pose_x = cfg.pose_x
    fit_req.pose_y = cfg.pose_y
    fit_req.pose_z = cfg.pose_z
    fit_req.roll_deg = cfg.roll_deg
    fit_req.pitch_deg = cfg.pitch_deg
    fit_req.yaw_deg = cfg.yaw_deg
    fit_req.current_q = []
    fit_req.clearance_threshold = args.clearance_threshold

    future = fit_cli.call_async(fit_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=120.0)
    res = future.result()
    if res is None:
        raise RuntimeError("/rtfg/fit_preview timeout or unavailable response")
    row = {
        "success": bool(res.success),
        "message": res.message,
        "anchor_count": int(res.anchor_count),
        "playback_count": int(res.playback_count),
        "timing_total_wall_s": float(res.timing_total_wall_s),
        "timing_ik_total_s": float(res.timing_ik_total_s),
        "timing_collision_total_s": float(res.timing_collision_total_s),
        "timing_avg_per_pose_s": float(res.timing_avg_per_pose_s),
        "min_self_clearance": float(res.min_self_clearance),
        "min_tool_body_clearance": float(res.min_tool_body_clearance),
        "min_tool_basin_clearance": float(res.min_tool_basin_clearance),
        "min_tool_basin_object": res.min_tool_basin_object,
        "yaml_path": args.yaml if args.yaml else "<package-default>",
        "clearance_threshold": float(args.clearance_threshold),
    }

    share = Path(get_package_share_directory("assembly_rtfg_cpp"))
    out_dir = share / "benchmarks"
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / f"rtfg_ros2_cpp_benchmark_{args.output_tag}.json"
    csv_path = out_dir / f"rtfg_ros2_cpp_benchmark_{args.output_tag}.csv"
    json_path.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)
    print(json.dumps(row, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
