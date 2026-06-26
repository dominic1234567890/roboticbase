# LD20 LiDAR Notes

The LD20 sends binary UART packets automatically after stable operation.

Useful packet facts:

- Header byte: `0x54`
- VerLen byte: `0x2c`
- 12 points per packet
- One point = 2-byte distance in millimeters + 1-byte confidence/intensity
- Start and end angles are in 0.01 degrees
- Timestamp is in milliseconds and wraps
- CRC is CRC-8 with polynomial `0x4D`, initial value `0x00`, no reflection, xorout `0x00`

This repo's parser lives in:

```text
src/tcr_minibot/sensors/lidar_ld20.py
```

## Common issues

### No `/dev/ttyUSB0`

Try:

```bash
lsusb
dmesg | tail -30
```

Then unplug/replug the CP2102 adapter.

### Permission denied

```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

### Raw bytes but parser prints nothing

Try:

```bash
python scripts/stage2_lidar_packets.py --skip-crc
```

If `--skip-crc` works, packet alignment is okay but CRC checking needs investigation.

### Raw bytes do not contain `54 2c`

Try a different baud rate:

```bash
python scripts/stage1_lidar_raw.py --baud 115200
python scripts/stage1_lidar_raw.py --baud 230400
```
