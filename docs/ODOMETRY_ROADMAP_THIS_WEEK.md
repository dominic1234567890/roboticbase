# Odometry and moving-scan roadmap

This roadmap assumes the LD20 LiDAR stays horizontal for now. The goal this week is not perfect SLAM; it is to replace manual two-scan pose offsets with live wheel odometry so horizontal scans can accumulate while the robot moves.

## Current wiring assumptions

Encoder defaults in the new scripts are intentionally swapped after the first encoder test:

```text
Left encoder:  GPIO27 + GPIO17
Right encoder: GPIO4  + GPIO22
Left motor:    Fusion HAT+ M3
Right motor:   Fusion HAT+ M2
```

If the robot moves forward and one side counts negative, use `--left-invert` or `--right-invert` instead of rewiring.

## Day 1: encoder truth and calibration

1. Watch raw counts:

   ```bash
   python scripts/odom0_watch_encoders.py
   ```

2. Spin one wheel exactly one full turn by hand and write down the count change.
3. Repeat three times per wheel.
4. Use the average as `--ticks-per-rev` for the rest of the scripts.

The ticks-per-rev value should be measured with this code because it counts every valid quadrature edge.

## Day 2: wheel-only pose

Run:

```bash
python scripts/odom1_watch_pose.py --ticks-per-rev YOUR_MEASURED_VALUE
```

Push the robot forward about 1 meter. You want `x` to increase by roughly 1 meter and heading to stay near zero. Then rotate in place 90 degrees and make sure heading changes in the expected direction.

If x goes negative during forward motion, invert one or both encoders with:

```bash
--left-invert
--right-invert
```

## Day 3: LiDAR emergency gate

Run a no-motion front-zone check:

```bash
python scripts/safety0_lidar_front_gate.py
```

Then test the guarded motor pulse only when the robot is safe to move:

```bash
python scripts/safety0_lidar_front_gate.py \
  --drive-forward-test \
  --i-understand-this-can-move-the-robot
```

The script fails closed: unknown front distance is treated as blocked.

## Day 4: moving horizontal scan accumulation

Run:

```bash
python scripts/stage8_lidar_odom_mapper.py \
  --ticks-per-rev YOUR_MEASURED_VALUE \
  --seconds 15 \
  --save data/captures/lidar_odom_map.png
```

Move slowly and mostly straight for the first attempt. This builds a 2D map by placing each horizontal scan at the current encoder odometry pose. It does not yet de-skew individual LiDAR points inside a scan.

## Day 5: route scaffold

Start with dry run:

```bash
python scripts/odom2_route_drive.py --dry-run
```

Then test with wheels off the ground at low power. Only after that, put it on the floor:

```bash
python scripts/odom2_route_drive.py \
  --route "straight:1.0,right:90,straight:1.0,left:90,straight:1.0" \
  --ticks-per-rev YOUR_MEASURED_VALUE \
  --enable-motors \
  --i-understand-this-will-drive-the-robot
```

Tune slowly: `--forward-power`, `--turn-power`, `--heading-kp`, and `--turn-kp`.

## Future gyro/IMU fusion

The code now has a `WheelGyroOdometry` class. Today it uses wheels only. Later, a gyro can feed yaw rate into the same update loop and you can raise `--gyro-delta-weight` from 0.0 after calibration.

Recommended order:

1. Wheel odometry stable.
2. Gyro raw z-rate stable while sitting still.
3. Estimate gyro bias.
4. Feed bias-corrected z-rate into `WheelGyroOdometry.update(...)`.
5. Tune `--gyro-delta-weight`.
