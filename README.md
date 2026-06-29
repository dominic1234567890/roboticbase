# roboticbase

Starter repository for Dominic's small Raspberry Pi 5 robot platform.

Current hardware target:

- Raspberry Pi 5
- Youyeetoo / LDROBOT LD20 2D spinning LiDAR through a CP2102 USB-to-TTL adapter
- Logitech C920 USB webcam
- SunFounder Fusion HAT+
- 2 yellow TT motors on a 2-wheel differential drive base
- Front caster wheel
- Wheel-center-to-wheel-center distance: **4.5 in = 0.1143 m**
- Future: 2 wheel encoders for odometry

This repo is intentionally staged so you can test sensors first while the robot is propped up and the Fusion HAT+ battery is still unplugged.

For a slower, beginner-friendly explanation of how the software and math fit together, read [docs/FOR_DUMMIES.md](docs/FOR_DUMMIES.md).

## Bring-up philosophy

Do **not** start by driving motors. Prove the sensors first.

Recommended order:

1. Boot Pi and update OS.
2. Confirm USB serial adapter appears as `/dev/ttyUSB0`.
3. Run raw LD20 byte test.
4. Run LD20 packet parser.
5. Run simple LiDAR safety bubble / obstacle printout.
6. Confirm camera opens.
7. Run camera preview / edges / contours.
8. Run the lidar-camera fusion experiment.
9. Only after the above works, install/test Fusion HAT+ motor imports.
10. Only after you intentionally plug in the 7.2V motor battery, run tiny motor pulse tests.
11. Add encoders later and enable odometry.

## Install on Raspberry Pi

From the Pi terminal:

```bash
cd ~
git clone <your-github-repo-url> trashcan-mini-pi5-robot
cd trashcan-mini-pi5-robot
bash scripts/install_pi_basics.sh
source .venv/bin/activate
```

If you are not using GitHub yet, copy this folder to the Pi and run the same install commands from inside it.

## Check devices

```bash
ls /dev/ttyUSB*
v4l2-ctl --list-devices
```

Expected:

- LD20 serial adapter: usually `/dev/ttyUSB0`
- Logitech C920: usually `/dev/video0`, sometimes `/dev/video2` depending on the Pi

## Stage tests

### Stage 0: system check

```bash
python scripts/stage0_system_check.py
```

### Stage 1: LD20 raw bytes

```bash
python scripts/stage1_lidar_raw.py --port /dev/ttyUSB0 --baud 230400
```

You want changing hex bytes and recurring `54 2c` packet starts.

### Stage 2: LD20 packets

```bash
python scripts/stage2_lidar_packets.py --port /dev/ttyUSB0 --baud 230400
```

This decodes packets into distance/angle points and prints scan stats.

### Stage 3: LiDAR viewer

```bash
python scripts/stage3_lidar_scan_viewer.py --port /dev/ttyUSB0 --baud 230400
```

If you are SSH-only, save one plot instead:

```bash
python scripts/stage3_lidar_scan_viewer.py --port /dev/ttyUSB0 --save data/captures/lidar_scan.png
```

### LiDAR calibration capture/export

Once packets are decoding, use this as the easiest calibration tool:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0
```

By default it captures one approximate 360-degree scan and writes timestamped files under `data/captures/`:

- `.png`: quick 2D point plot
- `.ply`: CloudCompare-friendly horizontal slice with range, bearing, confidence, scan index, and timestamp scalar fields
- `.xyz`: plain `x y z` points with `z=0`
- `.csv`: debug table for spreadsheets or scripts

For a live local 2D point view:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 --live
```

For a static plot window plus export:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 --show
```

For several scans in one CloudCompare file:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 --scans 5 --formats ply,csv
```

You can use the same cleanup filters here too:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 \
  --voxel-size-m 0.04 \
  --sor-mean-k 8 --sor-std-ratio 1.2 \
  --radius-outlier-radius-m 0.15 --radius-outlier-min-neighbors 2
```

### Stage 4: no-motor safety bubble

```bash
python scripts/stage4_lidar_safety_bubble.py --port /dev/ttyUSB0
```

This does **not** drive motors. It only tells you whether the front/left/right zones look blocked.

### Stage 5: LiDAR edge map

```bash
python scripts/stage5_lidar_edge_map.py --port /dev/ttyUSB0 --save data/captures/lidar_edges.png
```

This extracts wall-like line segments from one horizontal LiDAR scan and overlays them on the point cloud.

### Stage 6: fixed-pose room map

```bash
python scripts/stage6_lidar_room_mapper.py --port /dev/ttyUSB0 --seconds 10 --save data/captures/lidar_room_map.png
```

This accumulates repeated scans into a simple occupancy-grid image. Keep the robot still for this stage; odometry can be added later once encoders are working.

The mapping scripts can optionally clean the LiDAR point cloud before edge extraction:

```bash
python scripts/stage6_lidar_room_mapper.py --port /dev/ttyUSB0 --seconds 10 \
  --voxel-size-m 0.04 \
  --sor-mean-k 8 --sor-std-ratio 1.2 \
  --radius-outlier-radius-m 0.15 --radius-outlier-min-neighbors 2 \
  --save data/captures/lidar_room_map_filtered.png
```

Available filters:

- Voxel grid downsampling: `--voxel-size-m`
- Statistical outlier removal: `--sor-mean-k`, `--sor-std-ratio`
- Radius outlier removal: `--radius-outlier-radius-m`, `--radius-outlier-min-neighbors`
- Passthrough filtering: `--pass-x-min-m`, `--pass-x-max-m`, `--pass-y-min-m`, `--pass-y-max-m`, `--pass-range-min-m`, `--pass-range-max-m`, `--pass-bearing-min-deg`, `--pass-bearing-max-deg`

### Stage 7: two-scan measured forward test

```bash
python scripts/stage7_lidar_two_scan_forward_test.py --port /dev/ttyUSB0 --forward-m 0.30 --save data/captures/lidar_two_scan_forward_test.png
```

This captures one scan, waits while you manually move the robot forward by the measured distance, captures a second scan, and composes both scans into one map. This is a useful pre-encoder test for coordinate math and LiDAR yaw calibration. Measure from the LiDAR center, and keep the robot heading as straight as possible.

### Camera stages

```bash
python scripts/cam0_list_cameras.py
python scripts/cam1_preview.py --camera 0
python scripts/cam2_edges.py --camera 0
python scripts/cam3_contours.py --camera 0
```

### Funky LiDAR + Camera fusion experiment

```bash
python scripts/fusion0_lidar_plus_camera.py --port /dev/ttyUSB0 --camera 0
```

This does a simple bearing-based fusion:

- camera finds contour boxes,
- each box center is converted to an approximate bearing angle,
- the nearest LiDAR range near that bearing is attached to the detected object.

It is not full calibration yet. It is a useful bridge toward object detection + range estimation.

## Motor safety

The motor files are present but intentionally guarded.

Start with import only:

```bash
python scripts/motor0_import_check.py
```

Only when the robot is propped up, the 7.2V Fusion HAT+ battery is plugged in, and you intentionally want to test motors:

```bash
python scripts/motor1_tiny_pulse.py --i-understand-motors-are-propped-and-battery-is-plugged
```

No autonomous drive code should be enabled until LiDAR + safety bubble are reliable.

## File structure

```text
config/
  robot.yaml                    # robot measurements, serial ports, sensor guesses
src/tcr_minibot/
  sensors/
    lidar_ld20.py               # raw serial + LD20 parser
    camera_c920.py              # USB camera helper
  perception/
    lidar_filters.py            # zones, safety bubble, scan utilities
    point_cloud_export.py       # CSV/XYZ/PLY exports for CloudCompare and calibration
    occupancy_grid.py           # simple 2D grid mapping scaffold
    edge_mapping.py             # LiDAR point cloud, edge extraction, fixed-pose room mapper
    vision_simple.py            # edges/contours, later YOLO can plug in here
  fusion/
    lidar_camera_fusion.py      # camera detection + lidar range association
  hardware/
    motors.py                   # Fusion HAT+ motor wrapper with safety guards
  motion/
    differential_drive.py       # wheel speed / robot velocity math
  odometry/
    encoder_interface.py        # placeholder for future encoders
    differential_odometry.py    # dead-reckoning odometry math
  utils/
    config.py
    geometry.py
    rate.py
scripts/
  stage*_*.py                   # bring-up tests
  cam*_*.py                     # camera stages
  fusion0_*.py                  # sensor-fusion experiment
  motor*_*.py                   # guarded motor checks
```

## Development path

### LiDAR stages

1. Raw bytes
2. Packet decode
3. Single scan aggregation
4. Front/side obstacle zones
5. Occupancy grid
6. Wall/room corner detection
7. Mapping while moving
8. SLAM later, probably ROS 2 or a custom lightweight approach

### Camera stages

1. Camera opens
2. Preview
3. Edge detection
4. Contour/object blobs
5. Distance hints from LiDAR
6. Lightweight YOLO / object detector
7. Track object size over time
8. Combine object class + LiDAR range + robot velocity

### Odometry stages

1. Add encoder reader class
2. Confirm left/right tick counts
3. Compute wheel distances
4. Differential-drive pose estimate
5. Fuse odometry with LiDAR scan updates

## Notes from the hardware docs

The LD20 sends measurement data through one-way UART after stable operation. Its packet format starts with fixed header `0x54`, uses fixed VerLen `0x2c`, and contains 12 measurement points per packet. Each point has a 2-byte distance value and 1-byte confidence/intensity value.

The Fusion HAT+ Python docs use `from fusion_hat.motor import Motor`, create motors with ports like `Motor('M0')`, and command motor power using `motor.power(percent)`.
