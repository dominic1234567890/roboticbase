#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
import threading
from time import monotonic, sleep
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _lidar_filter_cli import add_lidar_filter_args, filter_config_from_args
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
from stage9_lidar_odom_drive_map import (
    add_scan_to_map,
    collect_filtered_scan,
    plot_map,
    write_edges_csv,
    write_point_cloud_csv,
    write_scan_summary_csv,
)
from tcr_minibot.hardware.motors import DifferentialMotors
from tcr_minibot.motion.differential_drive import arcade_to_wheel_power, clamp
from tcr_minibot.odometry.differential_odometry import Pose2D
from tcr_minibot.perception.edge_mapping import EdgeMappingConfig, MappingScan, RoomMapper
from tcr_minibot.perception.lidar_filters import PointCloudFilterConfig
from tcr_minibot.perception.occupancy_grid import OccupancyGrid
from tcr_minibot.sensors.lidar_ld20 import LidarPoint, SerialLD20
from tcr_minibot.sensors.phyphox_phone import PhyphoxConfig, PhyphoxError, PhyphoxPhoneGyro
from tcr_minibot.utils.config import load_config

ARM_FLAG = "--i-understand-this-will-drive-the-robot"


@dataclass
class SharedPhoneGyroDriveState:
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
    gyro_yaw_rate_radps: float | None = None
    heading_source: str = "not started"
    phone_gyro_ok: bool = False
    phone_gyro_error: str | None = None
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


def pose_copy(pose: Pose2D) -> Pose2D:
    return Pose2D(x_m=pose.x_m, y_m=pose.y_m, heading_rad=pose.heading_rad)


def distance_from_origin(x_m: float, y_m: float) -> float:
    return math.hypot(x_m, y_m)


def snapshot_state(state: SharedPhoneGyroDriveState) -> dict[str, object]:
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
            "gyro_yaw_rate_radps": state.gyro_yaw_rate_radps,
            "heading_source": state.heading_source,
            "phone_gyro_ok": state.phone_gyro_ok,
            "phone_gyro_error": state.phone_gyro_error,
        }


def make_phone_gyro(args: argparse.Namespace) -> PhyphoxPhoneGyro | None:
    if not args.phone_gyro_url:
        return None
    return PhyphoxPhoneGyro(
        PhyphoxConfig(
            base_url=args.phone_gyro_url,
            yaw_rate_buffer=args.phone_gyro_buffer,
            timeout_s=args.phone_gyro_timeout_s,
            yaw_rate_scale=args.phone_gyro_scale,
            invert_yaw_rate=args.phone_gyro_invert,
        )
    )


def maybe_read_phone_gyro(
    phone: PhyphoxPhoneGyro | None,
    args: argparse.Namespace,
    state: SharedPhoneGyroDriveState,
    *,
    now_s: float,
    next_warning_s: list[float],
) -> float | None:
    if phone is None:
        return None
    try:
        yaw_rate = phone.read_yaw_rate_radps()
    except PhyphoxError as exc:
        with state.lock:
            state.phone_gyro_ok = False
            state.phone_gyro_error = str(exc)
            state.gyro_yaw_rate_radps = None
        if args.require_phone_gyro:
            raise
        if now_s >= next_warning_s[0]:
            print(f"WARNING: phone gyro read failed; falling back to wheel heading for now: {exc}")
            next_warning_s[0] = now_s + 2.0
        return None

    with state.lock:
        state.phone_gyro_ok = True
        state.phone_gyro_error = None
        state.gyro_yaw_rate_radps = yaw_rate
    return yaw_rate


def drive_forward_worker(args: argparse.Namespace, state: SharedPhoneGyroDriveState, stop_event: threading.Event) -> None:
    reader = None
    motors: DifferentialMotors | None = None
    phone: PhyphoxPhoneGyro | None = None
    try:
        reader = make_encoder_reader(args)
        reader.reset()
        odom = make_odometry(args)

        phone = make_phone_gyro(args)
        if phone is not None:
            print(f"Using phone gyro: {phone.base_url} buffer={args.phone_gyro_buffer!r}")
            if args.phone_gyro_clear:
                phone.clear()
            if args.phone_gyro_start:
                phone.start()
            if args.phone_gyro_calibrate_s > 0.0:
                print(
                    f"Hold the robot completely still: calibrating phone gyro for "
                    f"{args.phone_gyro_calibrate_s:.1f}s..."
                )
                bias = phone.calibrate_bias(
                    duration_s=args.phone_gyro_calibrate_s,
                    sample_hz=max(5.0, args.control_hz),
                )
                print(f"phone gyro bias={bias:+.6f} rad/s")
        else:
            print("No --phone-gyro-url was provided; this run is wheel-only odometry.")

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
        next_warning_s = [start_s]

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
            yaw_rate_radps = maybe_read_phone_gyro(
                phone,
                args,
                state,
                now_s=loop_start_s,
                next_warning_s=next_warning_s,
            )
            update = odom.update(
                left_ticks,
                right_ticks,
                now_s=loop_start_s,
                gyro_yaw_rate_radps=yaw_rate_radps,
            )
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
                state.heading_source = update.heading_source

            if loop_start_s >= next_print_s:
                yaw_text = "none" if yaw_rate_radps is None else f"{math.degrees(yaw_rate_radps):+.1f} deg/s"
                print(
                    f"drive: t={elapsed_s:5.2f}s dist={traveled_m:6.3f} m "
                    f"{format_pose(pose.x_m, pose.y_m, pose.heading_rad)} "
                    f"err={heading_error_deg:+6.2f} deg turn={turn_correction:+5.2f}% "
                    f"gyro={yaw_text} source={update.heading_source}"
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


def add_phone_gyro_args(ap: argparse.ArgumentParser, odom_cfg: dict[str, Any]) -> None:
    group = ap.add_argument_group("temporary iPhone/phyphox gyro")
    group.add_argument(
        "--phone-gyro-url",
        default=str(odom_cfg.get("phone_gyro_url", "")),
        help="URL shown by phyphox Remote Access, e.g. http://192.168.1.42",
    )
    group.add_argument(
        "--phone-gyro-buffer",
        default=str(odom_cfg.get("phone_gyro_buffer", "z")),
        help="phyphox gyroscope buffer used as robot yaw-rate; usually z for a flat phone",
    )
    group.add_argument("--phone-gyro-timeout-s", type=float, default=float(odom_cfg.get("phone_gyro_timeout_s", 0.25)))
    group.add_argument("--phone-gyro-calibrate-s", type=float, default=float(odom_cfg.get("phone_gyro_calibrate_s", 2.0)))
    group.add_argument("--phone-gyro-scale", type=float, default=float(odom_cfg.get("phone_gyro_scale", 1.0)))
    group.add_argument(
        "--phone-gyro-invert",
        action=argparse.BooleanOptionalAction,
        default=_cfg_bool(odom_cfg, "phone_gyro_invert", False),
        help="Flip phone yaw-rate sign if left turns make heading decrease",
    )
    group.add_argument("--phone-gyro-start", action=argparse.BooleanOptionalAction, default=True)
    group.add_argument("--phone-gyro-clear", action=argparse.BooleanOptionalAction, default=False)
    group.add_argument(
        "--phone-gyro-weight",
        type=float,
        default=float(odom_cfg.get("phone_gyro_weight", 0.65)),
        help="Fusion weight for phone gyro heading delta. 0=wheels only, 1=phone gyro only.",
    )
    group.add_argument(
        "--require-phone-gyro",
        action="store_true",
        help="Stop the drive if the phone gyro cannot be read instead of falling back to wheel-only heading.",
    )


def main() -> None:
    cfg = load_config()
    mapping_cfg = cfg.get("mapping", {})
    drive_cfg = cfg.get("drive_forward_test", {})
    safety_cfg = cfg.get("safety", {})
    odom_cfg = cfg.get("odometry", {})

    ap = argparse.ArgumentParser(
        description=(
            "Stage 10 temporary scaffold: drive forward while mapping, but fuse encoder odometry "
            "with iPhone/phyphox gyro yaw-rate to make heading less fragile."
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
    scan.add_argument("--save", default="data/captures/lidar_phone_gyro_odom_drive_map.png")
    scan.add_argument("--summary-csv", default="data/captures/lidar_phone_gyro_odom_drive_map_summary.csv")
    scan.add_argument("--point-cloud-csv", default=None)
    scan.add_argument("--edges-csv", default=None)
    scan.add_argument("--max-gap-m", type=float, default=mapping_cfg.get("edge_max_neighbor_gap_m", 0.18))
    scan.add_argument("--max-line-error-m", type=float, default=mapping_cfg.get("edge_max_line_error_m", 0.035))

    safety = ap.add_argument_group("front safety gate")
    safety.add_argument("--front-gate", action=argparse.BooleanOptionalAction, default=_cfg_bool(safety_cfg, "stage9_front_gate", True))
    safety.add_argument("--front-stop-distance-m", type=float, default=float(safety_cfg.get("front_stop_distance_m", 0.35)))
    safety.add_argument("--front-half-angle-deg", type=float, default=float(safety_cfg.get("front_zone_half_angle_deg", 25.0)))

    add_phone_gyro_args(ap, odom_cfg)
    add_lidar_filter_args(ap, mapping_cfg)
    add_encoder_pin_args(ap, cfg)
    add_odometry_args(ap, cfg)
    add_heading_pid_args(ap, drive_cfg)
    add_current_motor_args(ap, cfg)
    args = ap.parse_args()
    filter_config: PointCloudFilterConfig = filter_config_from_args(args)

    if args.phone_gyro_url:
        args.gyro_delta_weight = args.phone_gyro_weight

    print(f"Using motor config: {describe_motor_args(args)}")
    print(
        f"stage10 phone gyro map: target_distance={args.target_distance_m:.3f} m "
        f"max_seconds={args.max_seconds:.2f} power={args.power:.1f}% min_points={args.min_points}"
    )
    print(
        f"Phone gyro: url={args.phone_gyro_url or '(not set)'} buffer={args.phone_gyro_buffer!r} "
        f"weight={args.gyro_delta_weight:.2f} invert={args.phone_gyro_invert}"
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
    state = SharedPhoneGyroDriveState()
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
        print(f"final heading source: {final.get('heading_source')}")
        if final.get("phone_gyro_error"):
            print(f"last phone gyro error: {final['phone_gyro_error']}")
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
