#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from assembly_rtfg_cpp.srv import FitPreview, LoadConfig


def call(cli, req, node, timeout_sec):
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
    if fut.result() is None:
        raise RuntimeError("service call timeout or failed")
    return fut.result()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", default="")
    parser.add_argument("--clearance", type=float, default=0.002)
    parser.add_argument("--max-candidates", type=int, default=60)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("rtfg_param_sweep")
    load_cli = node.create_client(LoadConfig, "/rtfg/load_config")
    fit_cli = node.create_client(FitPreview, "/rtfg/fit_preview")
    if not load_cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("/rtfg/load_config unavailable")
    if not fit_cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("/rtfg/fit_preview unavailable")

    load_req = LoadConfig.Request()
    load_req.yaml_path = args.yaml
    cfg = call(load_cli, load_req, node, 15.0)
    if not cfg.success:
        raise RuntimeError(cfg.message)

    base = {
        "left_wall_offset": float(cfg.left_wall_offset),
        "mud_height": float(cfg.mud_height),
        "approach_len": float(cfg.approach_len),
        "theta_deg": float(cfg.theta_deg),
        "depth": float(cfg.depth),
        "x_plane": float(cfg.x_plane),
        "pose_x": float(cfg.pose_x),
        "pose_y": float(cfg.pose_y),
        "pose_z": float(cfg.pose_z),
        "roll_deg": float(cfg.roll_deg),
        "pitch_deg": float(cfg.pitch_deg),
        "yaw_deg": float(cfg.yaw_deg),
    }

    # Prioritize safe execution over exact shape; keep search window compact.
    candidates = []
    for x_plane in (-0.10, -0.08, -0.12, -0.06, -0.135, -0.04):
        for left_wall_offset in (0.18, 0.17, 0.19, 0.16, 0.20):
            for depth in (0.02, 0.025, 0.03, 0.035, 0.04):
                for theta_deg in (-35.0, -40.0, -30.0, -45.0):
                    p = dict(base)
                    p["x_plane"] = x_plane
                    p["left_wall_offset"] = left_wall_offset
                    p["depth"] = depth
                    p["theta_deg"] = theta_deg
                    candidates.append(p)

    best = None
    best_cost = None
    for idx, p in enumerate(candidates, 1):
        req = FitPreview.Request()
        req.left_wall_offset = p["left_wall_offset"]
        req.mud_height = p["mud_height"]
        req.approach_len = p["approach_len"]
        req.theta_deg = p["theta_deg"]
        req.depth = p["depth"]
        req.x_plane = p["x_plane"]
        req.pose_x = p["pose_x"]
        req.pose_y = p["pose_y"]
        req.pose_z = p["pose_z"]
        req.roll_deg = p["roll_deg"]
        req.pitch_deg = p["pitch_deg"]
        req.yaw_deg = p["yaw_deg"]
        req.current_q = []
        req.clearance_threshold = args.clearance
        res = call(fit_cli, req, node, 90.0)
        row = {
            "idx": idx,
            "success": bool(res.success),
            "message": res.message,
            "left_wall_offset": p["left_wall_offset"],
            "depth": p["depth"],
            "theta_deg": p["theta_deg"],
            "x_plane": p["x_plane"],
            "timing_total_wall_s": float(res.timing_total_wall_s),
            "min_tool_basin_clearance": float(res.min_tool_basin_clearance),
            "min_tool_basin_object": res.min_tool_basin_object,
            "anchor_count": int(res.anchor_count),
            "playback_count": int(res.playback_count),
        }
        print(json.dumps(row, ensure_ascii=False), flush=True)
        if res.success:
            cost = float(res.timing_total_wall_s)
            if best is None or cost < best_cost:
                best = dict(p)
                best_cost = cost
        if idx >= args.max_candidates:
            break
        if best is not None and idx >= 12:
            # Once success exists, avoid full sweep latency.
            break

    share = Path(get_package_share_directory("assembly_rtfg_cpp"))
    out = share / "benchmarks" / "rtfg_param_sweep_best.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "best": best,
        "best_total_wall_s": best_cost,
        "baseline": base,
        "clearance_threshold": args.clearance,
    }
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
