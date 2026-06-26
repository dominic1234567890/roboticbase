# Safe Bring-Up Order

Use this order while the robot is propped up.

## Phase A — no battery, no motors

1. Boot Raspberry Pi 5 from USB-C power.
2. Keep the Fusion HAT+ 7.2 V motor battery unplugged.
3. Confirm LD20 is spinning and CP2102 appears:

```bash
ls /dev/ttyUSB*
dmesg | tail -30
```

4. Run raw bytes:

```bash
python scripts/stage1_lidar_raw.py --port /dev/ttyUSB0 --baud 230400
```

5. Run packet parser:

```bash
python scripts/stage2_lidar_packets.py --port /dev/ttyUSB0 --baud 230400
```

6. Capture and export a calibration scan:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0
```

This writes PNG, PLY, XYZ, and CSV files under `data/captures/`. Open the `.ply` in CloudCompare, or run this for a live 2D view:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 --live
```

7. Run no-motor safety bubble:

```bash
python scripts/stage4_lidar_safety_bubble.py --port /dev/ttyUSB0
```

Move your hand around the front/left/right of the LiDAR and make sure the printed distances make sense.

8. Capture one edge map:

```bash
python scripts/stage5_lidar_edge_map.py --port /dev/ttyUSB0 --save data/captures/lidar_edges.png
```

9. Build a fixed-pose room map while the robot stays still:

```bash
python scripts/stage6_lidar_room_mapper.py --port /dev/ttyUSB0 --seconds 10 --save data/captures/lidar_room_map.png
```

10. Test two-scan map composition with a manual measured move:

```bash
python scripts/stage7_lidar_two_scan_forward_test.py --port /dev/ttyUSB0 --forward-m 0.30 --save data/captures/lidar_two_scan_forward_test.png
```

Measure the move from the LiDAR center. Keep the heading straight; this test assumes the second scan is translated forward with no rotation unless you pass `--heading-deg`.

## Phase B — camera only

```bash
python scripts/cam0_list_cameras.py
python scripts/cam1_preview.py --camera 0
python scripts/cam2_edges.py --camera 0
python scripts/cam3_contours.py --camera 0
```

## Phase C — sensor fusion experiment

```bash
python scripts/fusion0_lidar_plus_camera.py --port /dev/ttyUSB0 --camera 0
```

This is still no-motor code.

## Phase D — Fusion HAT+ import only

```bash
python scripts/motor0_import_check.py
```

This does not drive motors.

## Phase E — tiny motor pulse

Only after:

- robot is propped up,
- battery is plugged in,
- LiDAR packet parser works,
- you are intentionally ready for a tiny movement test.

```bash
python scripts/motor1_tiny_pulse.py --i-understand-motors-are-propped-and-battery-is-plugged
```
