#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import socket
import struct
import subprocess
import sys
from dataclasses import dataclass
from typing import Callable


ROBOT_PORTS = {
    "dashboard": 29999,
    "primary": 30001,
    "secondary": 30002,
    "realtime": 30003,
    "rtde": 30004,
}

RESIDUAL_PATTERNS = (
    "ur_ros2_control_node",
    "dashboard_client",
    "controller_stopper",
    "robot_state_helper",
    "urscript_interface",
    "trajectory_until_node",
    "rtde_control",
    "rtde_receive",
    "ExternalControl",
    "external_control",
    "urx",
    "RoboDK",
)


@dataclass
class Result:
    ok: bool
    lines: list[str]


def run_command(args: list[str], timeout: float = 3.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return proc.returncode, (proc.stdout + proc.stderr).strip()
    except Exception as exc:
        return 1, str(exc)


def connect_port(host: str, port: int, timeout: float) -> tuple[bool, str, bytes]:
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            data = b""
            if port in (29999, 30001, 30002, 30003):
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    data = b""
            return True, "connected", data
    except Exception as exc:
        return False, str(exc), b""


def read_realtime_packet(host: str, port: int, timeout: float) -> tuple[bool, str]:
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            header = recv_exact(sock, 4)
            if header is None:
                return False, "no realtime packet header"
            packet_len = struct.unpack("!i", header)[0]
            if packet_len <= 4 or packet_len > 10000:
                return False, f"invalid realtime packet length: {packet_len}"
            body = recv_exact(sock, packet_len - 4)
            if body is None:
                return False, "incomplete realtime packet"
            packet = header + body
            if len(packet) < 300:
                return False, f"realtime packet too short: {len(packet)}"
            joints = struct.unpack("!6d", packet[252:300])
            formatted = ", ".join(f"{joint:.4f}" for joint in joints)
            return True, f"realtime act_q=[{formatted}] packet_len={packet_len}"
    except Exception as exc:
        return False, str(exc)


def recv_exact(sock: socket.socket, nbytes: int) -> bytes | None:
    data = bytearray()
    while len(data) < nbytes:
        try:
            chunk = sock.recv(nbytes - len(data))
        except socket.timeout:
            return None
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def check_network(robot_ip: str, reverse_ip: str, timeout: float) -> Result:
    lines: list[str] = []
    ok = True

    code, addr_out = run_command(["ip", "-brief", "-4", "addr"], timeout)
    local_ips = set(re.findall(r"\b(?:\d{1,3}\.){3}\d{1,3}\b", addr_out))
    lines.append("=== LOCAL IPv4 ADDRESSES ===")
    lines.append(addr_out or "NO_ADDR_OUTPUT")
    if reverse_ip in local_ips:
        lines.append(f"OK reverse_ip_present {reverse_ip}")
    else:
        lines.append(f"FAIL reverse_ip_missing {reverse_ip}")
        ok = False

    code, route_out = run_command(["ip", "route", "get", robot_ip], timeout)
    lines.append("=== ROUTE TO ROBOT ===")
    lines.append(route_out or "NO_ROUTE_OUTPUT")
    route_src_match = re.search(r"\bsrc\s+((?:\d{1,3}\.){3}\d{1,3})", route_out)
    route_src = route_src_match.group(1) if route_src_match else ""
    if code == 0 and route_src == reverse_ip:
        lines.append(f"OK route_src_matches_reverse_ip {route_src}")
    else:
        lines.append(f"FAIL route_src={route_src or 'UNKNOWN'} expected={reverse_ip}")
        ok = False

    code, ping_out = run_command(["ping", "-c", "1", "-W", "1", robot_ip], timeout)
    lines.append("=== PING ROBOT ===")
    lines.append(ping_out or "NO_PING_OUTPUT")
    if code == 0:
        lines.append("OK robot_ping")
    else:
        lines.append("FAIL robot_ping")
        ok = False

    return Result(ok, lines)


def check_ports(robot_ip: str, timeout: float) -> Result:
    lines = ["=== ROBOT TCP PORTS ==="]
    ok = True
    for name, port in ROBOT_PORTS.items():
        connected, message, data = connect_port(robot_ip, port, timeout)
        if connected:
            suffix = ""
            if data:
                preview = data[:80].replace(b"\r", b" ").replace(b"\n", b" ")
                suffix = f" bytes={len(data)} preview={preview!r}"
            lines.append(f"OK {name}:{port} {message}{suffix}")
        else:
            lines.append(f"FAIL {name}:{port} {message}")
            ok = False
    return Result(ok, lines)


def check_realtime(robot_ip: str, timeout: float) -> Result:
    ok, message = read_realtime_packet(robot_ip, ROBOT_PORTS["realtime"], timeout)
    lines = ["=== REALTIME 30003 PACKET ===", ("OK " if ok else "FAIL ") + message]
    return Result(ok, lines)


def check_primary(robot_ip: str, timeout: float) -> Result:
    lines = ["=== PRIMARY 30001 CONFIG STREAM ==="]
    packets: list[tuple[int, int]] = []
    data = bytearray()
    ok = True
    try:
        with socket.create_connection((robot_ip, ROBOT_PORTS["primary"]), timeout=timeout) as sock:
            sock.settimeout(timeout)
            while len(packets) < 10:
                header = recv_exact(sock, 4)
                if header is None:
                    break
                packet_len = struct.unpack("!i", header)[0]
                if packet_len <= 4 or packet_len > 100000:
                    lines.append(f"FAIL invalid_primary_packet_len={packet_len}")
                    ok = False
                    break
                body = recv_exact(sock, packet_len - 4)
                if body is None:
                    break
                packet = header + body
                data.extend(packet)
                packet_type = packet[4] if len(packet) > 4 else -1
                packets.append((packet_len, packet_type))
    except Exception as exc:
        lines.append(f"FAIL primary_connect {exc}")
        return Result(False, lines)

    if packets:
        packet_desc = ", ".join(f"len={length}/type={packet_type}" for length, packet_type in packets)
        lines.append(f"OK primary_packets count={len(packets)} bytes={len(data)} {packet_desc}")
    else:
        lines.append("FAIL no_primary_packets")
        ok = False

    if b"URControl" in data:
        lines.append("OK primary_version_message_contains_URControl")
    else:
        lines.append("FAIL primary_version_message_missing_URControl")
        ok = False

    if len(data) >= 100:
        lines.append("OK primary_stream_has_configuration_sized_payload")
    else:
        lines.append(f"FAIL primary_stream_too_short bytes={len(data)}")
        ok = False

    return Result(ok, lines)


def check_residuals(timeout: float) -> Result:
    lines = ["=== POSSIBLE CONFLICT PROCESSES ==="]
    ok = True
    for pattern in RESIDUAL_PATTERNS:
        code, out = run_command(["pgrep", "-a", "-f", pattern], timeout)
        filtered = [
            line
            for line in out.splitlines()
            if "ur10_comm_diagnostics.py" not in line and "pgrep -a -f" not in line
        ]
        if filtered:
            lines.append(f"FAIL pattern={pattern}")
            lines.extend(filtered)
            ok = False
    if ok:
        lines.append("OK no_conflict_processes")

    code, ss_out = run_command(["ss", "-lntp"], timeout)
    listeners = [
        line
        for line in ss_out.splitlines()
        if any(f":5000{i}" in line for i in range(1, 5))
    ]
    lines.append("=== LOCAL REVERSE PORT LISTENERS 50001-50004 ===")
    if listeners:
        lines.extend(f"INFO {line}" for line in listeners)
    else:
        lines.append("INFO no_5000x_listener")
    return Result(ok, lines)


def check_all(robot_ip: str, reverse_ip: str, timeout: float) -> Result:
    checks: tuple[Callable[[], Result], ...] = (
        lambda: check_network(robot_ip, reverse_ip, timeout),
        lambda: check_primary(robot_ip, timeout),
        lambda: check_ports(robot_ip, timeout),
        lambda: check_realtime(robot_ip, timeout),
        lambda: check_residuals(timeout),
    )
    lines: list[str] = []
    ok = True
    for check in checks:
        result = check()
        lines.extend(result.lines)
        ok = ok and result.ok
    return Result(ok, lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode",
        choices=("network", "primary", "ports", "realtime", "residuals", "all"),
    )
    parser.add_argument("--robot-ip", default="10.160.9.21")
    parser.add_argument("--reverse-ip", default="10.160.9.10")
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.mode == "network":
        result = check_network(args.robot_ip, args.reverse_ip, args.timeout)
    elif args.mode == "primary":
        result = check_primary(args.robot_ip, args.timeout)
    elif args.mode == "ports":
        result = check_ports(args.robot_ip, args.timeout)
    elif args.mode == "realtime":
        result = check_realtime(args.robot_ip, args.timeout)
    elif args.mode == "residuals":
        result = check_residuals(args.timeout)
    else:
        result = check_all(args.robot_ip, args.reverse_ip, args.timeout)

    for line in result.lines:
        print(line)
    print("DIAG_OK" if result.ok else "DIAG_FAIL")
    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())
