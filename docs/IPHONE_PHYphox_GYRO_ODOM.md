# iPhone phyphox gyro odometry scaffold

This is a temporary bridge so the robot can use an iPhone running phyphox as a yaw-rate gyro before a real onboard IMU is installed.

## Phone setup

1. Install/open **phyphox** on the iPhone.
2. Open the **Gyroscope** experiment.
3. Open the phyphox menu and enable **Allow Remote Access**.
4. Leave the phone on the same Wi-Fi/network as the Raspberry Pi.
5. Note the URL shown by phyphox, such as `http://192.168.1.42`.
6. Mount the phone firmly on the robot so it cannot slide. Start with the phone lying flat, screen up, with the top of the phone pointing forward.

## First Pi connection test

```bash
cd ~/roboticbase
source .venv/bin/activate

python scripts/gyro0_phyphox_watch.py \
  --phone-gyro-url http://YOUR_PHONE_IP \
  --buffer z \
  --start \
  --calibrate-s 2
```

Turn the robot gently left and right by hand.

- If left turns make the printed yaw rate positive, keep the default sign.
- If left turns make the printed yaw rate negative, add `--invert`.

## Moving LiDAR + odom map using phone gyro

Start gently. Keep the robot on the floor, clear the area, and keep the phone awake with phyphox visible.

```bash
python scripts/stage10_lidar_phone_gyro_odom_drive_map.py \
  --phone-gyro-url http://YOUR_PHONE_IP \
  --phone-gyro-buffer z \
  --phone-gyro-weight 0.65 \
  --target-distance-m 0.50 \
  --power 10 \
  --min-points 420 \
  --save data/captures/lidar_phone_gyro_odom_drive_map.png \
  --enable-motors \
  --i-understand-this-will-drive-the-robot
```

If the heading correction steers the wrong way, stop and retry with:

```bash
--phone-gyro-invert
```

If Wi-Fi is flaky and you would rather stop than fall back to wheel-only heading, add:

```bash
--require-phone-gyro
```

## Notes

- This is not meant to be permanent. Wi-Fi latency/dropouts make a phone gyro worse than a real wired IMU.
- The phone gyro helps heading drift/slip, but encoder distance still controls forward position.
- Keep the robot still during the calibration countdown.
- The default scaffold uses buffer `z`, which is usually yaw for a flat-mounted phone. Different phone mounting may need `x` or `y`.
