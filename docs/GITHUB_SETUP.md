# GitHub Setup

From the folder on your Pi or computer:

```bash
git init
git add .
git commit -m "Initial mini Pi5 robot repo"
```

Then create an empty GitHub repo and connect it:

```bash
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/trashcan-mini-pi5-robot.git
git push -u origin main
```

Recommended branches:

```bash
git checkout -b lidar-bringup
git checkout -b camera-bringup
git checkout -b sensor-fusion
git checkout -b motor-safety
```
