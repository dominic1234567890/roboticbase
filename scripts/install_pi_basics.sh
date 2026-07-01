#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  git \
  python3-venv \
  python3-pip \
  python3-serial \
  python3-opencv \
  python3-matplotlib \
  python3-yaml \
  python3-gpiozero \
  python3-lgpio \
  v4l-utils \
  minicom

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
python -m pip install -r requirements.txt

echo "Done. Activate with: source .venv/bin/activate"
echo "If /dev/ttyUSB0 permission fails, run: sudo usermod -a -G dialout $USER && sudo reboot"
