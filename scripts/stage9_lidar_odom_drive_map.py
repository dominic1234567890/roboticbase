#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
import threading
from time import monotonic, sleep
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _lidar_filter_cli import add_lidar_filter_args, clean_lidar_points, filter_config_from_args
from _odom_common import (
    HeadingPidController,
    add_current_motor_args,
    add_encoder_pin_args,
    add_heading_pid_args,
    add_odometry_args,
    describe_motor_args,
    format_pose,
    heading_pid_config_from_args,
    make_encoder_reader,
    make_odometry,
    motor_config_from_args,
)
from tcr_minibot.hardware.motors import DifferentialMotors
from tcr_minibot.motion.differential_drive import arcade_to_wheel_power, clamp
from tcr_minibot.odometry.differential_odometry import Pose2D
from tcr_minibot.perception.edge_mapping import EdgeMappingConfig, EdgeSegment, MappingScan, RoomMapper, XYPoint
from tcr_minibot.perception.lidar_filters import PointCloudFilterConfig, aggregate_one_scan, compute_zone_distances, valid_points
from tcr_minibot.perception.occupancy_grid import OccupancyGrid
from tcr_minibot.sensors.lidar_ld20 import LidarPoint, SerialLD20
from tcr_minibot.utils.config import load_config

ARM_FLAG = "--i-understand-this-will-drive-the-robot"


@dataclass
class SharedDriveState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    pose: Pose2D = field(default_factory=Pose2D)
    left_ticks: int = 0
    right_ticks: int = 0
    elapsed_s: float = 0.0
    traveled_m: float = 0.0
    heading_error_deg: float = 0.0
    turn_correction_percent: float = 0.0
    left_command_percent: float = 0.0
    right_command_percent: float = 0.0
    done: bool = False
    done_reason: str = "not started"
    exception_text: str | None = None


def _cfg_float(section: dict[str, Any], key: str, default: float) -> float:
    return float(section.get(key, default))


def _cfg_bool(section: dict[str, Any], key: str, default: bool) -> bool:
    value = section.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def distance_from_origin(x_m: float, y_m: float) -> float:
    return math.hypot(x_m, y_m)


def pose_copy(pose: Pose2D) -> Pose2D:
    return Pose2D(x_m=pose.x_m, y_m=pose.y_m, heading_rad=pose.heading_rad)


def write_scan_summary_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "scan_id",
        "label",
        "time_s",
        "raw_points",
        "clean_points",
        "front_min_m",
        "x_m",
        "y_m",
        "heading_deg",
        "traveled_m",
        "left_ticks",
        "right_ticks",
        "edge_count",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved {path}")


def write_point_cloud_csv(path: Path, scans: list[tuple[str, MappingScan]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scan_id", "x_m", "y_m"])
        for scan_id, scan in scans:
            for point in scan.points:
                writer.writerow([scan_id, f"{point.x_m:.4f}", f"{point.y_m:.4f}"])
    print(f"saved {path}")


def write_edges_csv(path: Path, scans: list[tuple[str, MappingScan]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scan_id", "start_x_m", "start_y_m", "end_x_m", "end_y_m", "length_m", "heading_deg"])
        for scan_id, scan in scans:
            for segment in scan.edge_segments:
                writer.writerow([
                    scan_id,
                    f"{segment.start_x_m:.4f}",
                    f"{segment.start_y_m:.4f}",
                    f"{segment.end_x_m:.4f}",
                    f"{segment.end_y_m:.4f}",
                    f"{segment.length_m:.4f}",
                    f"{segment.heading_deg:.2f}",
                ])
    print(f"saved {path}")


def front_distance_m(raw_points: list[LidarPoint], args: argparse.Namespace, cfg: dict[str, Any]) -> float | None:
    points = valid_points(
        raw_points,
        min_m=cfg["lidar"].get("min_distance_m", 0.08),
        max_m=cfg["lidar"].get("max_distance_m", 8.0),
        min_confidence=cfg["lidar"].get("min_confidence", 0),
    )
    zones = compute_zone_distances(points, front_half_angle_deg=args.front_half_angle_deg)
    return zones.front_min_m


def collect_filtered_scan(
    lidar: SerialLD20,
    *,
    args: argparse.Namespace,
    cfg: dict[str, Any],
    filter_config: PointCloudFilterConfig,
    label: str,
) -> tuple[list[LidarPoint], list[LidarPoint], float | None]:
    print(f"Collecting {label}...")
    raw_points = aggregate_one_scan(lidar.frames(), min_points=args.min_points)
    distance = front_distance_m(raw_points, args, cfg)
    clean_points = clean_lidar_points(raw_points, cfg=cfg, filter_config=filter_config, label=label)
    return raw_points, clean_points, distance


def drive_forward_worker(args: argparse.Namespace, state: SharedDriveState, stop_event: threading.Event) -> None:
    reader = None
    motors: DifferentialMotors | None = None
    try:
        reader = make_encoder_reader(args)
        reader.reset()
        odom = make_odometry(args)
        start_s = monotonic()
        odom.reset(
            left_ticks=reader.read_left_ticks(),
            right_ticks=reader.read_right_ticks(),
            now_s=start_s,
        )
        pid = HeadingPidController(heading_pid_config_from_args(args))
        pid.reset(odom.pose.heading_rad)
        motors = DifferentialMotors(motor_config_from_args(args), armed=True)
        period_s = 1.0 / args.control_hz
        print_period_s = 1.0 / args.print_hz
        next_print_s = start_s

        with state.lock:
            state.done = False
            state.done_reason = "driving"
            state.exception_text = None
            state.pose = pose_copy(odom.pose)

        while not stop_event.is_set():
            loop_start_s = monotonic()
            elapsed_s = loop_start_s - start_s
            left_ticks = reader.read_left_ticks()
            right_ticks = reader.read_right_ticks()
            update = odom.update(left_ticks, right_ticks, now_s=loop_start_s)
            pose = update.pose
            traveled_m = distance_from_origin(pose.x_m, pose.y_m)

            if args.target_distance_m > 0.0 and traveled_m >= args.target_distance_m:
                with state.lock:
                    state.done_reason = "target distance reached"
                break
            if elapsed_s >= max(0.0, args.max_seconds):
                with state.lock:
                    state.done_reason = "max seconds reached"
                break

            remaining_m = max(0.0, args.target_distance_m - traveled_m) if args.target_distance_m > 0.0 else args.target_distance_m
            slow_factor = 1.0
            if args.target_distance_m > 0.0:
                slow_factor = clamp(remaining_m / max(args.slowdown_distance_m, 1e-6), 0.35, 1.0)
            forward_power = args.power * slow_factor
            turn_correction, heading_error_deg = pid.update(pose.heading_rad, update.dt_s or period_s)
            cmd = arcade_to_wheel_power(forward_power, turn_correction, max_abs=args.max_power)
            motors.drive_power(cmd)

            with state.lock:
                state.pose = pose_copy(pose)
                state.left_ticks = left_ticks
                state.right_ticks = right_ticks
                state.elapsed_s = elapsed_s
                state.traveled_m = traveled_m
                state.heading_error_deg = heading_error_deg
                state.turn_correction_percent = turn_correction
                state.left_command_percent = cmd.left
                state.right_command_percent = cmd.right

            if loop_start_s >= next_print_s:
                print(
                    f"drive: t={elapsed_s:5.2f}s dist={traveled_m:6.3f} m "
                    f"{format_pose(pose.x_m, pose.y_m, pose.heading_rad)} "
                    f"err={heading_error_deg:+6.2f} deg turn={turn_correction:+5.2f}%"
                )
                next_print_s = loop_start_s + print_period_s

            sleep(max(0.0, period_s - (monotonic() - loop_start_s)))
    except BaseException as exc:
        with state.lock:
            state.exception_text = f"{type(exc).__name__}: {exc}"
            state.done_reason = "drive thread error"
        stop_event.set()
    finally:
        if motors is not None:
            motors.stop()
        if reader is not None:
            reader.close()
        with state.lock:
            state.done = True


def snapshot_state(state: SharedDriveState) -> dict[str, object]:
    with state.lock:
        return {
            "pose": pose_copy(state.pose),
            "elapsed_s": state.elapsed_s,
            "traveled_m": state.traveled_m,
            "left_ticks": state.left_ticks,
            "right_ticks": state.right_ticks,
            "done": state.done,
            "done_reason": state.done_reason,
            "exception_text": state.exception_text,
        }


def add_scan_to_map(
    mapper: RoomMapper,
    scans: list[tuple[str, MappingScan]],
    summary_rows: list[dict[str, object]],
    *,
    scan_id: str,
    label: str,
    raw_points: list[LidarPoint],
    clean_points: list[LidarPoint],
    front_min_m: float | None,
    state_snapshot: dict[str, object],
) -> None:
    pose = state_snapshot["pose"]
    assert isinstance(pose, Pose2D)
    mapped_scan = mapper.add_scan(clean_points, pose=pose)
    scans.append((scan_id, mapped_scan))
    front_text = "unknown" if front_min_m is None else f"{front_min_m:.3f} m"
    print(
        f"{scan_id}: mapped {len(clean_points)}/{len(raw_points)} points at "
        f"{format_pose(pose.x_m, pose.y_m, pose.heading_rad)} front={front_text} edges={len(mapped_scan.edge_segments)}"
    )
    summary_rows.append(
        {
            "scan_id": scan_id,
            "label": label,
            "time_s": f"{float(state_snapshot['elapsed_s']):.4f}",
            "raw_points": len(raw_points),
            "clean_points": len(clean_points),
            "front_min_m": "" if front_min_m is None else f"{front_min_m:.4f}",
            "x_m": f"{pose.x_m:.5f}",
            "y_m": f"{pose.y_m:.5f}",
            "heading_deg": f"{math.degrees(pose.heading_rad):.3f}",
            "traveled_m": f"{float(state_snapshot['traveled_m']):.5f}",
            "left_ticks": state_snapshot["left_ticks"],
            "right_ticks": state_snapshot["right_ticks"],
            "edge_count": len(mapped_scan.edge_segments),
        }
    )


def plot_map(mapper: RoomMapper, scans: list[tuple[str, MappingScan]], save_path: Path) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(
        mapper.grid.as_display_image(),
        extent=mapper.grid.image_extent_m(),
        origin="upper",
        cmap="gray",
        vmin=0,
        vmax=255,
    )

    for index, (scan_id, scan) in enumerate(scans):
        _scatter_scan(ax, scan.points, label=scan_id)
        _plot_edges(ax, scan.edge_segments)
        ax.scatter([scan.pose.x_m], [scan.pose.y_m], marker="x", s=45)
        ax.arrow(
            scan.pose.x_m,
            scan.pose.y_m,
            0.18 * math.cos(scan.pose.heading_rad),
            0.18 * math.sin(scan.pose.heading_rad),
            head_width=0.04,
            length_includes_head=True,
            alpha=0.75,
        )
        if index == 0 or index == len(scans) - 1:
            ax.text(scan.pose.x_m, scan.pose.y_m, scan_id, fontsize=8)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x forward from start pose (m)")
    ax.set_ylabel("y left from start pose (m)")
    ax.set_title("Stage 9 LiDAR + encoder odometry moving map")
    ax.grid(True, alpha=0.25)
    if len(scans) <= 12:
        ax.legend(loc="upper right")

    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=170)
    print(f"saved {save_path}")


def _scatter_scan(ax, points: list[XYPoint], *, label: str) -> None:
    if points:
        ax.scatter([p.x_m for p in points], [p.y_m for p in points], s=3, alpha=0.28, label=label)


def _plot_edges(ax, segments: list[EdgeSegment]) -> None:
    for segment in segments:
        ax.plot(
            [segment.start_x_m, segment.end_x_m],
            [segment.start_y_m, segment.end_y_m],
            linewidth=1.4,
            alpha=0.75,
        )


def main() -> None:
    cfg = load_config()
    mapping_cfg = cfg.get("mapping", {})
    drive_cfg = cfg.get("drive_forward_test", {})
    safety_cfg = cfg.get("safety", {})

    ap = argparse.ArgumentParser(
        description=(
            "Stage 9: drive forward with encoder heading hold while repeatedly taking LD20 scans. "
            "The saved map uses odometry poses to place each LiDAR scan."
        )
    )
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="Print resolved config but do not touch GPIO, motors, or LiDAR")
    ap.add_argument("--enable-motors", action="store_true", help="Required before any motion command is sent")
    ap.add_argument(ARM_FLAG, action="store_true", dest="armed_ack")

    drive = ap.add_argument_group("drive command")
    drive.add_argument("--target-distance-m", type=float, default=_cfg_float(drive_cfg, "target_distance_m", 0.75))
    drive.add_argument("--max-seconds", type=float, default=_cfg_float(drive_cfg, "seconds", 8.0))
    drive.add_argument("--power", type=float, default=_cfg_float(drive_cfg, "power_percent", 12.0))
    drive.add_argument("--control-hz", type=float, default=_cfg_float(drive_cfg, "control_hz", 20.0))
    drive.add_argument("--print-hz", type=float, default=_cfg_float(drive_cfg, "print_hz", 4.0))
    drive.add_argument("--slowdown-distance-m", type=float, default=0.25)

    scan = ap.add_argument_group("LiDAR mapping")
    scan.add_argument("--min-points", type=int, default=int(mapping_cfg.get("moving_scan_min_points", 420)))
    scan.add_argument("--scan-period-s", type=float, default=float(mapping_cfg.get("moving_scan_period_s", 0.0)))
    scan.add_argument("--final-scan", action=argparse.BooleanOptionalAction, default=True)
    scan.add_argument("--save", default="data/captures/lidar_odom_drive_map.png")
    scan.add_argument("--summary-csv", default="data/captures/lidar_odom_drive_map_summary.csv")
    scan.add_argument("--point-cloud-csv", default=None)
    scan.add_argument("--edges-csv", default=None)
    scan.add_argument("--max-gap-m", type=float, default=mapping_cfg.get("edge_max_neighbor_gap_m", 0.18))
    scan.add_argument("--max-line-error-m", type=float, default=mapping_cfg.get("edge_max_line_error_m", 0.035))

    safety = ap.add_argument_group("front safety gate")
    safety.add_argument("--front-gate", action=argparse.BooleanOptionalAction, default=_cfg_bool(safety_cfg, "stage9_front_gate", True))
    safety.add_argument("--front-stop-distance-m", type=float, default=float(safety_cfg.get("front_stop_distance_m", 0.35)))
    safety.add_argument("--front-half-angle-deg", type=float, default=float(safety_cfg.get("front_zone_half_angle_deg", 25.0)))

    add_lidar_filter_args(ap, mapping_cfg)
    add_encoder_pin_args(ap, cfg)
    add_odometry_args(ap, cfg)
    add_heading_pid_args(ap, drive_cfg)
    add_current_motor_args(ap, cfg)
    args = ap.parse_args()
    filter_config = filter_config_from_args(args)

    print(f"Using motor config: {describe_motor_args(args)}")
    print(
        f"stage9: target_distance={args.target_distance_m:.3f} m max_seconds={args.max_seconds:.2f} "
        f"power={args.power:.1f}% min_points={args.min_points} front_gate={args.front_gate}"
    )
    print(
        f"Straight heading PID: kp={args.heading_kp:.3f} ki={args.heading_ki:.3f} "
        f"kd={args.heading_kd:.3f} max_correction={args.max_turn_correction:.1f}%"
    )

    if args.dry_run or not args.enable_motors:
        print("Dry run only. Add --enable-motors and the acknowledgement flag to drive and map.")
        return
    if not args.armed_ack:
        print(f"Refusing to drive. Re-run with {ARM_FLAG} after the robot is on the floor and the area is clear.")
        raise SystemExit(2)
    if args.control_hz <= 0.0:
        raise ValueError("--control-hz must be greater than zero")
    if args.print_hz <= 0.0:
        raise ValueError("--print-hz must be greater than zero")

    grid = OccupancyGrid(
        size_m=float(mapping_cfg.get("size_m", 6.0)),
        cell_size_m=float(mapping_cfg.get("cell_size_m", 0.05)),
    )
    mapper = RoomMapper(
        grid=grid,
        edge_config=EdgeMappingConfig(
            max_neighbor_gap_m=args.max_gap_m,
            max_line_error_m=args.max_line_error_m,
            min_points_per_segment=mapping_cfg.get("edge_min_segment_points", 8),
            min_segment_length_m=mapping_cfg.get("edge_min_segment_length_m", 0.25),
        ),
    )

    lidar = SerialLD20(
        args.port,
        args.baud,
        mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0),
        check_crc=not args.skip_crc,
    )
    state = SharedDriveState()
    stop_event = threading.Event()
    drive_thread: threading.Thread | None = None
    scans: list[tuple[str, MappingScan]] = []
    summary_rows: list[dict[str, object]] = []

    try:
        raw_points, clean_points, front_min_m = collect_filtered_scan(
            lidar,
            args=args,
            cfg=cfg,
            filter_config=filter_config,
            label="pre-drive scan",
        )
        if args.front_gate and (front_min_m is None or front_min_m < args.front_stop_distance_m):
            shown = "unknown" if front_min_m is None else f"{front_min_m:.3f} m"
            print(f"Emergency stop gate refused motion before driving: front={shown}")
            raise SystemExit(2)
        add_scan_to_map(
            mapper,
            scans,
            summary_rows,
            scan_id="scan_000",
            label="pre-drive",
            raw_points=raw_points,
            clean_points=clean_points,
            front_min_m=front_min_m,
            state_snapshot=snapshot_state(state),
        )

        drive_thread = threading.Thread(target=drive_forward_worker, args=(args, state, stop_event), daemon=True)
        drive_thread.start()

        scan_index = 1
        while True:
            snap = snapshot_state(state)
            if bool(snap["done"]):
                break

            scan_start_s = monotonic()
            raw_points, clean_points, front_min_m = collect_filtered_scan(
                lidar,
                args=args,
                cfg=cfg,
                filter_config=filter_config,
                label=f"moving scan {scan_index}",
            )
            snap = snapshot_state(state)
            add_scan_to_map(
                mapper,
                scans,
                summary_rows,
                scan_id=f"scan_{scan_index:03d}",
                label="moving",
                raw_points=raw_points,
                clean_points=clean_points,
                front_min_m=front_min_m,
                state_snapshot=snap,
            )
            scan_index += 1

            if args.front_gate and (front_min_m is None or front_min_m < args.front_stop_distance_m):
                shown = "unknown" if front_min_m is None else f"{front_min_m:.3f} m"
                print(f"Emergency stop: front zone became blocked or unknown while mapping: front={shown}")
                stop_event.set()
                break

            if args.scan_period_s > 0.0:
                sleep(max(0.0, args.scan_period_s - (monotonic() - scan_start_s)))

        stop_event.set()
        if drive_thread is not None:
            drive_thread.join(timeout=3.0)

        if args.final_scan:
            raw_points, clean_points, front_min_m = collect_filtered_scan(
                lidar,
                args=args,
                cfg=cfg,
                filter_config=filter_config,
                label="final stopped scan",
            )
            add_scan_to_map(
                mapper,
                scans,
                summary_rows,
                scan_id=f"scan_{scan_index:03d}",
                label="final-stopped",
                raw_points=raw_points,
                clean_points=clean_points,
                front_min_m=front_min_m,
                state_snapshot=snapshot_state(state),
            )

        final = snapshot_state(state)
        print(f"drive done reason: {final['done_reason']}")
        if final["exception_text"]:
            raise RuntimeError(str(final["exception_text"]))
    except KeyboardInterrupt:
        print("\nInterrupted; stopping motors and saving what was collected.")
        stop_event.set()
    finally:
        stop_event.set()
        if drive_thread is not None:
            drive_thread.join(timeout=3.0)
        lidar.close()

    if not scans:
        print("No scans were collected; no map was written.")
        return
    plot_map(mapper, scans, Path(args.save))
    if args.summary_csv:
        write_scan_summary_csv(Path(args.summary_csv), summary_rows)
    if args.point_cloud_csv:
        write_point_cloud_csv(Path(args.point_cloud_csv), scans)
    if args.edges_csv:
        write_edges_csv(Path(args.edges_csv), scans)


if __name__ == "__main__":
    main()
