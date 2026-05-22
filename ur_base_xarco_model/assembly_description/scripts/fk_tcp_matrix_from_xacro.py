#!/usr/bin/env python3
import argparse
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd


JOINT_ORDER = [
    "ur10_shoulder_pan",
    "ur10_shoulder_lift",
    "ur10_elbow",
    "ur10_wrist_1",
    "ur10_wrist_2",
    "ur10_wrist_3",
]
CSV_Q_COLS = ["Act_q0", "Act_q1", "Act_q2", "Act_q3", "Act_q4", "Act_q5"]


def rotx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def roty(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rotz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rotz(yaw) @ roty(pitch) @ rotx(roll)


def rot_from_quat_xyzw(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rot_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    n = np.linalg.norm(axis)
    if n <= 1e-12 or abs(angle) <= 1e-12:
        return np.eye(3)
    x, y, z = axis / n
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c
    return np.array(
        [
            [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
        ],
        dtype=float,
    )


def rot_from_rotvec(rv: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rv))
    if angle <= 1e-12:
        return np.eye(3)
    axis = rv / angle
    return rot_from_axis_angle(axis, angle)


def make_tf(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def parse_triplet(s: Optional[str], default: Tuple[float, float, float]) -> Tuple[float, float, float]:
    if not s:
        return default
    vals = [float(v) for v in s.strip().split()]
    if len(vals) != 3:
        return default
    return vals[0], vals[1], vals[2]


def parse_quat4(s: Optional[str]) -> Optional[Tuple[float, float, float, float]]:
    if not s:
        return None
    vals = [float(v) for v in s.strip().split()]
    if len(vals) != 4:
        return None
    return vals[0], vals[1], vals[2], vals[3]


def sanitize_xacro_text(text: str) -> str:
    lines = []
    for line in text.splitlines():
        if line.strip() == "+":
            continue
        lines.append(line)
    return "\n".join(lines)


def parse_joints_from_xacro(xacro_path: Path) -> Dict[str, dict]:
    raw = xacro_path.read_text(encoding="utf-8", errors="ignore")
    cleaned = sanitize_xacro_text(raw)
    root = ET.fromstring(cleaned)

    joints = {}
    for joint in root.findall(".//joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        name = joint.attrib.get("name", "")
        jtype = joint.attrib.get("type", "fixed")
        origin_el = joint.find("origin")
        axis_el = joint.find("axis")

        ox, oy, oz = (0.0, 0.0, 0.0)
        rr, rp, ry = (0.0, 0.0, 0.0)
        qxyzw = None
        if origin_el is not None:
            ox, oy, oz = parse_triplet(origin_el.attrib.get("xyz"), (0.0, 0.0, 0.0))
            rr, rp, ry = parse_triplet(origin_el.attrib.get("rpy"), (0.0, 0.0, 0.0))
            qxyzw = parse_quat4(origin_el.attrib.get("quat_xyzw"))

        ax, ay, az = (0.0, 0.0, 1.0)
        if axis_el is not None:
            ax, ay, az = parse_triplet(axis_el.attrib.get("xyz"), (0.0, 0.0, 1.0))

        joints[name] = {
            "name": name,
            "type": jtype,
            "parent": parent.attrib.get("link"),
            "child": child.attrib.get("link"),
            "xyz": np.array([ox, oy, oz], dtype=float),
            "rpy": np.array([rr, rp, ry], dtype=float),
            "quat_xyzw": qxyzw,
            "axis": np.array([ax, ay, az], dtype=float),
        }
    return joints


def joint_origin_tf(joint: dict) -> np.ndarray:
    if joint["quat_xyzw"] is not None:
        x, y, z, w = joint["quat_xyzw"]
        R = rot_from_quat_xyzw(x, y, z, w)
    else:
        rr, rp, ry = joint["rpy"]
        R = rot_from_rpy(rr, rp, ry)
    return make_tf(R, joint["xyz"])


def find_chain(joints: Dict[str, dict], base_link: str, target_link: str) -> List[dict]:
    graph: Dict[str, List[str]] = {}
    for name, j in joints.items():
        graph.setdefault(j["parent"], []).append(name)

    path: List[dict] = []
    visited_links = set()

    def dfs(link: str) -> bool:
        if link in visited_links:
            return False
        visited_links.add(link)
        if link == target_link:
            return True
        for jname in graph.get(link, []):
            j = joints[jname]
            path.append(j)
            if dfs(j["child"]):
                return True
            path.pop()
        return False

    if not dfs(base_link):
        raise RuntimeError(f"No chain found from {base_link} to {target_link}")
    return path


def fk_from_chain(chain: List[dict], q_map: Dict[str, float]) -> np.ndarray:
    T = np.eye(4)
    for j in chain:
        T = T @ joint_origin_tf(j)
        if j["type"] != "fixed":
            q = q_map.get(j["name"], 0.0)
            R = rot_from_axis_angle(j["axis"], q)
            T = T @ make_tf(R, np.zeros(3, dtype=float))
    return T


def matrix_to_flat_row(T: np.ndarray) -> Dict[str, float]:
    out = {}
    for r in range(4):
        for c in range(4):
            out[f"t{r}{c}"] = float(T[r, c])
    return out


def rot_error_angle_rad(R_fk: np.ndarray, R_act: np.ndarray) -> float:
    R_err = R_fk @ R_act.T
    val = (np.trace(R_err) - 1.0) * 0.5
    val = max(-1.0, min(1.0, float(val)))
    return math.acos(val)


def write_latex_formula(path: Path, chain: List[dict]) -> None:
    ordered = [j["name"] for j in chain]
    revolute = [j["name"] for j in chain if j["type"] != "fixed"]
    lines = [
        r"\begin{align}",
        r"{}^{base\_jizuo}\mathbf{T}_{sensor\_shovel\_shovel\_tcp} &= \prod_{k=1}^{N}\mathbf{A}_k(q_k), \\",
        r"\mathbf{A}_k(q_k) &= \mathbf{T}(\mathbf{r}_k,\mathbf{R}_{xyz,k})\cdot",
        r"\begin{cases}",
        r"\mathbf{R}(\hat{\mathbf{a}}_k, q_k), & \text{if joint }k\text{ is revolute}\\",
        r"\mathbf{I}, & \text{if joint }k\text{ is fixed}",
        r"\end{cases}",
        r"\end{align}",
        "",
        r"% Joint order in chain:",
        r"% " + " -> ".join(ordered),
        r"% Revolute mapping:",
        r"% " + ", ".join([f"{name}=Act_q{i}" for i, name in enumerate(revolute)]),
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Compute FK matrices for sensor_shovel_shovel_tcp from xacro + Act_q")
    parser.add_argument(
        "--csv",
        required=True,
        help="Input csv path with Act_q0..Act_q5 and optional Act_X..Act_RZ columns",
    )
    parser.add_argument(
        "--xacro",
        default="/root/ur10_ws/src/ur_base_xarco_model/assembly_xacro/assembly/assembly.urdf.xacro",
        help="Xacro/URDF source path",
    )
    parser.add_argument(
        "--base-link",
        default="base_jizuo",
        help="Base frame link name",
    )
    parser.add_argument(
        "--target-link",
        default="sensor_shovel_shovel_tcp",
        help="TCP frame link name",
    )
    parser.add_argument(
        "--output-dir",
        default=".",
        help="Output directory for csv/md/tex",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv).resolve()
    xacro_path = Path(args.xacro).resolve()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    joints = parse_joints_from_xacro(xacro_path)
    chain = find_chain(joints, args.base_link, args.target_link)
    chain_joint_names = [j["name"] for j in chain]

    df = pd.read_csv(csv_path)
    missing = [c for c in CSV_Q_COLS if c not in df.columns]
    if missing:
        raise RuntimeError(f"Input CSV missing columns: {missing}")

    fk_rows = []
    err_rows = []

    has_act_pose = all(col in df.columns for col in ["Act_X", "Act_Y", "Act_Z", "Act_RX", "Act_RY", "Act_RZ"])

    for idx, row in df.iterrows():
        q_map = {JOINT_ORDER[i]: float(row[CSV_Q_COLS[i]]) for i in range(6)}
        T = fk_from_chain(chain, q_map)
        fk_item = {"index": int(idx)}
        fk_item.update(matrix_to_flat_row(T))
        fk_rows.append(fk_item)

        err_item = {"index": int(idx)}
        if has_act_pose:
            act_p = np.array([float(row["Act_X"]), float(row["Act_Y"]), float(row["Act_Z"])], dtype=float)
            fk_p = T[:3, 3]
            pos_err_m = float(np.linalg.norm(fk_p - act_p))

            act_rotvec = np.array([float(row["Act_RX"]), float(row["Act_RY"]), float(row["Act_RZ"])], dtype=float)
            R_act = rot_from_rotvec(act_rotvec)
            R_fk = T[:3, :3]
            ang_err_rad = rot_error_angle_rad(R_fk, R_act)

            ortho_err = float(np.linalg.norm(R_fk @ R_fk.T - np.eye(3), ord="fro"))
            det_r = float(np.linalg.det(R_fk))

            err_item.update(
                {
                    "pos_error_m": pos_err_m,
                    "pos_error_mm": pos_err_m * 1000.0,
                    "ori_error_rad": ang_err_rad,
                    "ori_error_deg": math.degrees(ang_err_rad),
                    "fk_orthogonality_fro": ortho_err,
                    "fk_det_r": det_r,
                }
            )
        err_rows.append(err_item)

    fk_df = pd.DataFrame(fk_rows)
    err_df = pd.DataFrame(err_rows)

    fk_csv = out_dir / "tcp_fk_matrices.csv"
    err_csv = out_dir / "fk_vs_act_pose_error.csv"
    summary_md = out_dir / "fk_summary.md"
    formula_tex = out_dir / "fk_formula.tex"

    fk_df.to_csv(fk_csv, index=False)
    err_df.to_csv(err_csv, index=False)
    write_latex_formula(formula_tex, chain)

    summary_lines = [
        "# FK Summary",
        "",
        f"- xacro: `{xacro_path}`",
        f"- csv: `{csv_path}`",
        f"- base_link: `{args.base_link}`",
        f"- target_link: `{args.target_link}`",
        f"- samples: `{len(df)}`",
        f"- chain: `{' -> '.join(chain_joint_names)}`",
    ]
    if has_act_pose and not err_df.empty and "pos_error_mm" in err_df.columns:
        summary_lines.extend(
            [
                "",
                "## Error Stats",
                f"- pos_error_mm mean: `{err_df['pos_error_mm'].mean():.4f}`",
                f"- pos_error_mm max: `{err_df['pos_error_mm'].max():.4f}`",
                f"- ori_error_deg mean: `{err_df['ori_error_deg'].mean():.4f}`",
                f"- ori_error_deg max: `{err_df['ori_error_deg'].max():.4f}`",
                f"- det(R) mean: `{err_df['fk_det_r'].mean():.6f}`",
                f"- ||R*R^T-I||_F mean: `{err_df['fk_orthogonality_fro'].mean():.6e}`",
            ]
        )
    summary_lines.extend(
        [
            "",
            "## Outputs",
            f"- `{fk_csv}`",
            f"- `{err_csv}`",
            f"- `{summary_md}`",
            f"- `{formula_tex}`",
        ]
    )
    summary_md.write_text("\n".join(summary_lines), encoding="utf-8")

    print(f"[done] wrote: {fk_csv}")
    print(f"[done] wrote: {err_csv}")
    print(f"[done] wrote: {summary_md}")
    print(f"[done] wrote: {formula_tex}")


if __name__ == "__main__":
    main()
