# TrashCan Mini Robot Software for Dummies

This document explains the software like you are just starting to calibrate the robot and want to know what every piece is doing.

The short version:

1. The LD20 LiDAR sends binary packets over USB serial.
2. The parser turns those bytes into distance + angle points.
3. Geometry code turns distance + angle into `x, y` points on the floor plane.
4. Filters remove junk points or simplify the cloud.
5. Mapping code turns points into pictures, edges, occupancy grids, and export files.
6. Later, odometry will tell the mapper where the robot moved between scans.

The robot does not need motors to start doing useful LiDAR work. Right now the best path is:

```bash
python scripts/stage1_lidar_raw.py --port /dev/ttyUSB0 --baud 230400
python scripts/stage2_lidar_packets.py --port /dev/ttyUSB0 --baud 230400
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 --show
python scripts/stage5_lidar_edge_map.py --port /dev/ttyUSB0 --save data/captures/lidar_edges.png
python scripts/stage6_lidar_room_mapper.py --port /dev/ttyUSB0 --seconds 10 --save data/captures/lidar_room_map.png
```

## Big Picture

The codebase is split into two kinds of files:

- `src/tcr_minibot/...`: reusable robot software.
- `scripts/...`: command-line tools you run during bring-up and calibration.

The scripts should stay boring. They parse command-line flags, call the reusable code, print useful output, and save files.

The reusable code does the important math:

- serial packet parsing
- coordinate transforms
- filtering
- line extraction
- occupancy grids
- camera bearing estimates
- differential-drive math
- future odometry

## Coordinate System

The repo uses a robot-centered 2D coordinate system:

```text
          +y left
            ^
            |
            |
            o----> +x forward
          robot
```

Angles use this convention:

- `0 deg` is straight forward.
- positive angles turn left.
- negative angles turn right.
- `180 deg` or `-180 deg` is backward.

The LD20 sensor reports raw angles clockwise. Robot math is easier if angles increase counter-clockwise, so the parser converts:

```text
robot_bearing_deg = -(ld20_raw_angle_deg + mount_yaw_offset_deg)
```

Then it wraps the result into `[-180, 180)`.

The most important conversion in the whole project is polar to Cartesian:

```text
x = distance_m * cos(bearing_rad)
y = distance_m * sin(bearing_rad)
```

That gives one horizontal slice of the world. In CloudCompare exports, `z = 0` because the LD20 is a 2D scanner.

## The LiDAR Pipeline

The LiDAR flow looks like this:

```text
USB serial bytes
  -> LD20Parser
  -> LidarFrame
  -> LidarPoint
  -> valid_points()
  -> optional filters
  -> plot/export/edge extraction/occupancy grid
```

`LidarPoint` is the main data object for LiDAR:

```text
raw_angle_deg_cw   angle from the LD20 packet, clockwise
bearing_deg        robot bearing, counter-clockwise
distance_m         range measurement in meters
confidence         sensor intensity/confidence byte
x_m, y_m           robot-frame point
timestamp_ms       LD20 timestamp
```

If the points look rotated in the plot, fix `lidar.mount_yaw_offset_deg` in `config/robot.yaml`.

## Calibration Workflow

Start with raw bytes:

```bash
python scripts/stage1_lidar_raw.py --port /dev/ttyUSB0 --baud 230400
```

You want to see changing hex bytes and recurring `54 2c`, which is the LD20 packet start.

Then decode packets:

```bash
python scripts/stage2_lidar_packets.py --port /dev/ttyUSB0 --baud 230400
```

Then capture a scan:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 --show
```

This writes:

- `.png`: 2D plot for quick visual checking
- `.ply`: CloudCompare-friendly point cloud
- `.xyz`: plain point file
- `.csv`: spreadsheet/debug file

In a good scan:

- nearby walls look like lines
- a flat wall in front of the robot is mostly horizontal in the image
- the forward direction is up the robot `+x` axis in code, shown on plots as `x forward`
- random isolated points should be rare

If the shape is good but rotated, adjust `mount_yaw_offset_deg`.

If there are lots of speckles, try filters:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 \
  --voxel-size-m 0.04 \
  --sor-mean-k 8 --sor-std-ratio 1.2 \
  --radius-outlier-radius-m 0.15 --radius-outlier-min-neighbors 2
```

## Point Cloud Filters

The filters live in `src/tcr_minibot/perception/lidar_filters.py`.

They are optional. Raw data is best for first calibration. Filters are best after you understand what the raw scan looks like.

### Passthrough Filter

Passthrough keeps only points inside a selected region.

Examples:

```text
keep only x from 0 to 3 m
keep only y from -1 to +1 m
keep only range from 0.1 to 4 m
keep only bearings from -45 to +45 deg
```

The math is just comparisons:

```text
min_x <= x <= max_x
min_y <= y <= max_y
min_range <= range <= max_range
```

Bearing passthrough handles wraparound. A window like `340 deg` to `20 deg` means points near zero degrees.

### Voxel Grid Downsampling

Voxel grid downsampling reduces the number of points.

In 3D, a voxel is a little cube. Here the LiDAR is 2D, so it is really a grid cell:

```text
voxel_x = floor(x / leaf_size)
voxel_y = floor(y / leaf_size)
```

All points in one cell are grouped together. The code keeps the real point closest to the cell's centroid. It does not invent a brand-new averaged `LidarPoint`, because the original point carries confidence, bearing, and timestamp.

Why use it:

- fewer points to draw
- faster line fitting
- cleaner-looking exports

Why not always use it:

- it can hide small objects
- it can make corners less crisp if the leaf size is too big

Good starting values:

```text
0.03 m to 0.06 m
```

### Radius Outlier Removal

Radius outlier removal deletes lonely points.

For each point, the code asks:

```text
How many other points are within radius_m?
```

If that count is less than `min_neighbors`, the point is removed.

Useful starting values:

```text
radius_outlier_radius_m = 0.12 to 0.20
radius_outlier_min_neighbors = 2
```

### Statistical Outlier Removal

Statistical outlier removal, or SOR, deletes points whose nearest-neighbor distances are unusually large.

For each point:

1. Compute distances to all other points.
2. Sort them.
3. Average the nearest `k` distances.
4. Compare that average against the global average.

The threshold is:

```text
threshold = global_mean + std_ratio * global_std
```

If a point's mean neighbor distance is above the threshold, it is probably an outlier.

Useful starting values:

```text
sor_mean_k = 8 to 12
sor_std_ratio = 1.0 to 1.5
```

## Occupancy Grid Math

An occupancy grid is a 2D array where each cell represents a small square of floor.

Current defaults:

```text
map size = 6.0 m by 6.0 m
cell size = 0.05 m
grid width = 6.0 / 0.05 = 120 cells
```

World position to grid cell:

```text
cell_x = origin_x + x_m / cell_size_m
cell_y = origin_y - y_m / cell_size_m
```

`cell_y` subtracts because image rows go downward, while robot `+y` goes left/up in the map.

The mapper marks:

- free cells along the LiDAR beam
- occupied cell at the hit point

The beam is traced through grid cells using Bresenham line math. This is the classic integer line algorithm used to draw lines on pixel grids.

The grid values are simple scores:

```text
negative = probably free
zero     = unknown
positive = probably occupied
```

This is not full SLAM yet. It is a practical starter map.

## Edge Mapping Math

Edge mapping tries to turn a cloud of points into wall-like line segments.

The steps:

1. Sort points by bearing.
2. Split into clusters when neighboring points are too far apart.
3. Fit a line to each cluster.
4. If the cluster does not fit one line well, split it and try again.

Line fitting uses principal component analysis through NumPy SVD.

In plain English:

- Find the center of the points.
- Find the direction where the points stretch the most.
- That direction is the wall direction.
- The perpendicular errors tell us how well the points fit a straight wall.

The line error is approximately:

```text
error = distance from each point to the fitted line
rms_error = sqrt(mean(error^2))
```

If the maximum error is too high, the code splits the cluster at the worst point. This helps turn an L-shaped corner into two wall segments.

## Two-Scan Math Before Encoders

Before wheel encoders exist, you can still test map composition.

Run:

```bash
python scripts/stage7_lidar_two_scan_forward_test.py --port /dev/ttyUSB0 --forward-m 0.30
```

The script:

1. Captures scan 1 at pose `(0, 0, 0)`.
2. You move the robot forward a measured distance.
3. It captures scan 2 at pose `(forward_m, 0, heading_deg)`.
4. It draws both scans in one shared world frame.

The transform from robot frame to world frame is:

```text
x_world = pose_x + x_robot * cos(theta) - y_robot * sin(theta)
y_world = pose_y + x_robot * sin(theta) + y_robot * cos(theta)
```

For a straight forward move with no rotation:

```text
x_world = forward_m + x_robot
y_world = y_robot
```

This is a great way to test coordinate math before encoders. It is not yet autonomous odometry.

## Camera Math

The camera code uses a simple pinhole-ish approximation.

The image `x` coordinate is converted to a bearing:

```text
normalized = (x_px - image_width / 2) / (image_width / 2)
bearing = -normalized * (horizontal_fov_deg / 2) + camera_yaw_offset_deg
```

Why the negative sign?

- image left has smaller x
- robot left is positive bearing
- so image-left should become positive robot bearing

This is approximate until the camera is calibrated.

## LiDAR + Camera Fusion Math

The fusion experiment is simple:

1. Camera finds a contour box.
2. Box center gives an approximate bearing.
3. LiDAR points near that bearing are searched.
4. The nearest LiDAR range is attached to the camera detection.

This does not know full 3D position yet. It is a practical way to answer:

```text
That blob in the camera is roughly 1.2 m away at +10 degrees.
```

## Differential Drive Math

The robot has two driven wheels.

For arcade-style control:

```text
left_power = forward - turn
right_power = forward + turn
```

For velocity control:

```text
left_speed = linear_speed - angular_speed * wheel_track / 2
right_speed = linear_speed + angular_speed * wheel_track / 2
```

If the robot rotates left, the right wheel moves faster than the left wheel.

## Odometry Math

Odometry will estimate pose from wheel encoder ticks.

Ticks to distance:

```text
wheel_circumference = 2 * pi * wheel_radius
distance = ticks / ticks_per_rev * wheel_circumference
```

Differential-drive update:

```text
d_center = (d_left + d_right) / 2
d_heading = (d_right - d_left) / wheel_track
```

Then pose moves forward by `d_center` at the middle heading:

```text
mid_heading = old_heading + d_heading / 2
x += d_center * cos(mid_heading)
y += d_center * sin(mid_heading)
heading += d_heading
```

Encoders are not installed yet, so this is scaffolded but not the main calibration path today.

## File-by-File Guide

### Top-Level Files

#### `README.md`

Main project guide. It is the quickest place to find install commands, bring-up order, and stage scripts.

Math: none directly. It describes how to run the math/code elsewhere.

#### `requirements.txt`

Python package list for the Pi.

Math: none.

#### `pyproject.toml`

Python package metadata. It tells Python packaging tools this repo uses `src/` layout and lists dependencies.

Math: none.

#### `.gitignore`

Keeps generated files like caches and virtual environments out of git.

Math: none.

### Config

#### `config/robot.yaml`

The robot's editable settings:

- wheel track
- wheel radius
- LiDAR serial port
- LiDAR angle offset
- camera size/FOV
- mapping size
- filtering defaults
- motor safety settings

Math:

- `wheel_track_m` affects turning and odometry.
- `wheel_radius_m` affects encoder tick-to-distance conversion.
- `mount_yaw_offset_deg` rotates LiDAR angles into robot coordinates.
- `cell_size_m` controls occupancy grid resolution.

### Scripts

#### `scripts/install_pi_basics.sh`

Installs basic packages and creates the Python environment on the Pi.

Math: none.

#### `scripts/_bootstrap.py`

Adds `src/` to Python's import path so scripts can import `tcr_minibot`.

Math: none.

#### `scripts/_lidar_filter_cli.py`

Shared command-line flags for voxel, SOR, radius outlier, and passthrough filtering.

Math:

- It builds a `PointCloudFilterConfig`.
- The real filter math is in `lidar_filters.py`.

#### `scripts/stage0_system_check.py`

Basic environment check.

Math: none.

#### `scripts/stage1_lidar_raw.py`

Reads raw serial bytes from the LD20 and prints hex.

Math: none. It proves the USB serial link is alive.

#### `scripts/stage2_lidar_packets.py`

Reads LD20 bytes and runs the packet parser.

Math:

- packet angle interpolation
- checksum validation
- polar-to-XY conversion through the parser

#### `scripts/stage3_lidar_scan_viewer.py`

Captures one approximate scan and plots `x_m, y_m`.

Math:

- uses parser-generated `x_m, y_m`
- plot is a horizontal LiDAR slice

#### `scripts/lidar_capture_export.py`

Calibration/export tool.

What it does:

- captures one or more scans
- optionally shows live 2D points
- saves PNG, PLY, XYZ, and CSV
- can apply point cloud filters

Math:

- uses polar-to-XY points from parser
- optional filtering
- exports 2D points as `z=0`

#### `scripts/stage4_lidar_safety_bubble.py`

Prints minimum obstacle distances in front/left/right/rear zones.

Math:

- selects points by angle window
- finds minimum distance in each zone

#### `scripts/stage5_lidar_edge_map.py`

Captures one scan and extracts wall-like line segments.

Math:

- optional filters
- cluster by point distance
- SVD line fitting
- split-and-fit edge extraction

#### `scripts/stage6_lidar_room_mapper.py`

Accumulates repeated fixed-pose scans into an occupancy grid.

Math:

- grid cell conversion
- ray tracing free space
- occupied endpoint marking
- line extraction for visible edges

#### `scripts/stage7_lidar_two_scan_forward_test.py`

Captures two scans with a measured manual forward move between them.

Math:

- rigid transform from robot frame to world frame
- tests the same pose math encoders will eventually feed

#### `scripts/cam0_list_cameras.py`

Finds working camera indices.

Math: none.

#### `scripts/cam1_preview.py`

Shows camera frames.

Math: none beyond image dimensions.

#### `scripts/cam2_edges.py`

Shows Canny edges.

Math:

- grayscale conversion
- Gaussian blur
- Canny gradient thresholding through OpenCV

#### `scripts/cam3_contours.py`

Finds contour blobs and draws boxes.

Math:

- contour area threshold
- bounding rectangle
- center point of each detection

#### `scripts/fusion0_lidar_plus_camera.py`

Runs camera detection and attaches LiDAR ranges.

Math:

- pixel x to bearing
- nearest LiDAR point near that bearing

#### `scripts/motor0_import_check.py`

Checks whether Fusion HAT+ motor imports work.

Math: none.

#### `scripts/motor1_tiny_pulse.py`

Runs a guarded tiny motor pulse.

Math:

- clamps requested power to safe maximum
- no autonomy or navigation

### Source Package

#### `src/tcr_minibot/__init__.py` and folder `__init__.py` files

These mark directories as Python packages.

Files:

- `src/tcr_minibot/__init__.py`
- `src/tcr_minibot/sensors/__init__.py`
- `src/tcr_minibot/perception/__init__.py`
- `src/tcr_minibot/fusion/__init__.py`
- `src/tcr_minibot/hardware/__init__.py`
- `src/tcr_minibot/motion/__init__.py`
- `src/tcr_minibot/odometry/__init__.py`
- `src/tcr_minibot/utils/__init__.py`

Math: none.

#### `src/tcr_minibot/sensors/lidar_ld20.py`

The LD20 parser and serial reader.

Math and logic:

- finds packet header `0x54 0x2c`
- checks CRC-8
- decodes little-endian fields
- interpolates 12 angles between packet start and end angle
- converts raw clockwise LD20 angle to robot bearing
- converts polar distance/bearing into `x_m, y_m`

Important formulas:

```text
span = (end_angle - start_angle) % 360
step = span / (points_per_packet - 1)
raw_angle_i = start_angle + step * i
x = distance * cos(bearing)
y = distance * sin(bearing)
```

#### `src/tcr_minibot/sensors/camera_c920.py`

USB camera wrapper.

Math:

- frame width/height/FPS settings
- no geometry yet

#### `src/tcr_minibot/perception/lidar_filters.py`

LiDAR scan utilities and point cloud filters.

Math:

- angle window filtering
- min distance by zone
- passthrough comparisons
- voxel grid indexing
- radius neighbor counting
- statistical outlier thresholding

#### `src/tcr_minibot/perception/point_cloud_export.py`

Writes point cloud rows to PLY, XYZ, and CSV.

Math:

- keeps LiDAR points as horizontal 3D rows: `(x, y, z=0)`
- stores extra scalar fields for CloudCompare

#### `src/tcr_minibot/perception/occupancy_grid.py`

2D map grid.

Math:

- world-to-cell conversion
- Bresenham line tracing
- free/occupied score updates
- display image conversion

#### `src/tcr_minibot/perception/edge_mapping.py`

Point cloud to edges and room map helper.

Math:

- robot-to-world pose transform
- scan clustering by point gap
- SVD line fitting
- split-and-fit segmentation

#### `src/tcr_minibot/perception/vision_simple.py`

Simple OpenCV perception.

Math:

- Canny edges
- contour area
- bounding box center

#### `src/tcr_minibot/fusion/lidar_camera_fusion.py`

Connects camera detections to LiDAR range.

Math:

- pixel x to bearing
- nearest LiDAR range within tolerance

#### `src/tcr_minibot/hardware/motors.py`

Guarded Fusion HAT+ motor wrapper.

Math:

- clamps power values
- applies left/right reversal settings

#### `src/tcr_minibot/motion/differential_drive.py`

Drive math helpers.

Math:

- arcade drive to left/right wheel power
- linear/angular velocity to wheel speeds

#### `src/tcr_minibot/odometry/encoder_interface.py`

Encoder interfaces and dummy placeholder.

Math:

- none yet; it defines the shape for future tick readers

#### `src/tcr_minibot/odometry/differential_odometry.py`

Dead-reckoning pose estimate from wheel ticks.

Math:

- ticks to distance
- differential-drive pose update
- heading wrapping to `[-pi, pi)`

#### `src/tcr_minibot/utils/geometry.py`

Shared geometry helpers.

Math:

- angle wrapping
- degrees to radians
- polar-to-XY
- LD20 clockwise angle to robot CCW bearing

#### `src/tcr_minibot/utils/config.py`

Loads `config/robot.yaml`.

Math: none, but it provides parameters used by math elsewhere.

#### `src/tcr_minibot/utils/rate.py`

Loop rate helper.

Math:

- sleeps enough time to make a loop run at approximately a target frequency

### Tests

#### `tests/test_ld20_parser.py`

Checks LD20 packet length, CRC, and parsed values.

Math tested:

- CRC-8
- angle parsing
- distance parsing

#### `tests/test_lidar_filters.py`

Checks passthrough, voxel, radius outlier, and SOR behavior.

Math tested:

- wraparound bearing windows
- grid grouping
- neighbor distances
- statistical thresholding

#### `tests/test_edge_mapping.py`

Checks wall extraction and room mapper transforms.

Math tested:

- line headings
- occupancy grid free/occupied marking
- pose rotation and translation

#### `tests/test_point_cloud_export.py`

Checks PLY, XYZ, and CSV exports.

Math tested:

- horizontal point export with `z=0`
- preservation of range/bearing/confidence metadata

### Data Folders

#### `data/captures/`

Place for generated plots, PLY files, CSVs, and scan captures.

The `.gitkeep` file inside it exists only so git keeps the empty directory.

Math: none.

#### `data/logs/`

Place for future logs.

The `.gitkeep` file inside it exists only so git keeps the empty directory.

Math: none.

### Docs

#### `docs/BRINGUP_ORDER.md`

Safe staged bring-up checklist.

Math: none directly.

#### `docs/LIDAR_NOTES.md`

LD20 packet facts and troubleshooting notes.

Math:

- packet fields
- CRC expectation

#### `docs/ROADMAP.md`

Project direction.

Math: none directly.

#### `docs/GITHUB_SETUP.md`

GitHub setup notes.

Math: none.

#### `docs/FOR_DUMMIES.md`

This file.

Math: all the high-level explanations in one place.

## What To Calibrate First

### 1. Serial Port and Baud

If no packets decode, check:

```bash
ls /dev/ttyUSB*
python scripts/stage1_lidar_raw.py --port /dev/ttyUSB0 --baud 230400
python scripts/stage1_lidar_raw.py --port /dev/ttyUSB0 --baud 115200
```

### 2. LiDAR Yaw Offset

Put a flat wall directly in front of the robot and capture:

```bash
python scripts/lidar_capture_export.py --port /dev/ttyUSB0 --show
```

If the wall is rotated, adjust:

```yaml
lidar:
  mount_yaw_offset_deg: 0.0
```

Change the value, rerun, repeat.

### 3. Distance Sanity

Measure a wall with a tape measure. Compare against the plotted/CSV range.

If it is wildly wrong, check:

- wrong baud
- bad parsing
- sensor tilted
- sensor seeing the robot body
- units confused between mm and m

### 4. Filter Tuning

Start raw. Then add one filter at a time.

Good first filter:

```bash
--radius-outlier-radius-m 0.15 --radius-outlier-min-neighbors 2
```

Good downsample:

```bash
--voxel-size-m 0.04
```

Do not turn on every filter at once until you know what each one changes.

## Common Failure Modes

### The scan is mirrored

Likely angle convention or physical mounting mismatch.

Check `ld20_clockwise_to_robot_ccw()` and `mount_yaw_offset_deg`.

### The scan is rotated

Adjust `lidar.mount_yaw_offset_deg`.

### The scan has a blind wedge

Could be:

- robot body blocking the LiDAR
- cable or mount in the way
- sensor not level

### The room map looks smeared

If the robot is moving during fixed-pose mapping, the map will smear. Stage 6 assumes the robot is still.

Use stage 7 for measured movement. Use encoders later for real moving maps.

### CloudCompare opens a flat line

That is expected if you view from the side. This is a horizontal 2D slice with `z=0`. Switch to top view.

## Safe Development Rule

Sensor code first. Motor code later.

The current mapping and calibration scripts do not drive motors. That is intentional. Keep it that way until the LiDAR direction, safety bubble, and emergency-stop behavior are reliable.
