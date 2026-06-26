# Project Roadmap

## Now

- Validate LD20 raw serial.
- Parse LD20 packets.
- Validate camera preview.
- Run no-motor safety bubble.
- Run single-scan LiDAR edge mapping.
- Run fixed-pose LiDAR room mapping.
- Test two-scan mapping with a manual measured forward move.
- Run contour + LiDAR range association.

## Next

- Calibrate LiDAR yaw offset so `0 degrees` means robot-front.
- Calibrate camera FOV and yaw offset.
- Add a simple emergency-stop script that refuses motion if front zone is blocked.
- Add wheel encoders.
- Add differential-drive odometry.
- Replace manual two-scan pose deltas with encoder odometry so scans accumulate while moving.

## Later

- Object detection with a lightweight model.
- Sensor fusion: object label from camera + range from LiDAR + robot motion from odometry.
- Local planner that outputs safe velocity commands.
- ROS 2 bridge if the custom stack gets too big.
