# Isaac Sim 5.1 — Docker setup (headless + WebRTC streaming)

Reproducible Isaac Sim launch for **native Ubuntu 22.04/24.04** hosts with an RTX
GPU. We use the **official** NVIDIA image (`nvcr.io/nvidia/isaac-sim:5.1.0`) — no
custom build — wrapped in a compose file so it's a pull-and-go on any machine.

> This is separate from the repo's top-level `docker/docker-compose.yml`, which is
> the native-Linux ROS 2 / Gazebo dev stack. Don't mix them.

## Why headless + streaming
Isaac Sim renders on the GPU box and streams the viewport to a thin **WebRTC
client** — good for a laptop GPU and for remote access.

> ⚠️ **Blackwell (RTX 50-series) + driver 595.x:** the RTX renderer
> (`libcarb.scenerenderer-rtx`) segfaults on startup **even in headless mode** —
> headless does *not* dodge it. You must downgrade the host driver to the **580.xx**
> branch first. See **[`DRIVER_DOWNGRADE.md`](https://github.com/PavanSandaka/project-drishti/blob/main/docker/isaac-sim/DRIVER_DOWNGRADE.md)**.

## One-time host prereqs
1. **NVIDIA driver** present (`nvidia-smi` works) — RTX GPU with RT cores required.
2. **NVIDIA Container Toolkit:**
   ```bash
   sudo apt-get install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   # verify GPU passthrough:
   docker run --rm --gpus all nvcr.io/nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi
   ```
3. **NGC login** (needed to pull the image):
   ```bash
   docker login nvcr.io   # Username: $oauthtoken   Password: <your NGC API key>
   ```

## Quick start
```bash
git fetch && git checkout feat/isaac-sim-docker
docker pull nvcr.io/nvidia/isaac-sim:5.1.0     # ~15–20 GB, first time only
./docker/isaac-sim/start_isaac.sh
```
Wait for **`Isaac Sim Full Streaming App is loaded.`** (first run is slow — it's
building the shader cache, not hung).

## Viewing it — WebRTC Streaming Client
Download the **Isaac Sim WebRTC Streaming Client** (separate small app from NVIDIA),
then connect:
- **Same machine:** connect to `127.0.0.1`.
- **From another machine on the LAN:** connect to the host's IP, and open
  **TCP 49100 + UDP 47998** on the host firewall (UDP is required for WebRTC media;
  opening only TCP fails).

## Interactive / debugging
```bash
docker compose -f docker/isaac-sim/docker-compose.yml run --rm isaac-sim bash
# then inside:  ./runheadless.sh -v
```

## Notes & gotchas
- **VRAM:** 8 GB is fine for mobile-robot + sensors + ROS 2 bridge at moderate scene
  complexity (use "RTX – Real-Time", not path tracing). Isaac Lab / humanoid RL wants
  more — keep parallel envs small.
- **Cache lives on the host** at `~/docker/isaac-sim/` (owned by uid 1234). Safe to
  delete to reclaim space; next launch just rebuilds it.
- **Driver 595.x on Blackwell crashes the RTX renderer even headless** — downgrade to
  the 580.xx branch is required, not optional. Full steps in
  [`DRIVER_DOWNGRADE.md`](https://github.com/PavanSandaka/project-drishti/blob/main/docker/isaac-sim/DRIVER_DOWNGRADE.md).

## Next step (why this is in project-drishti)
Once streaming works, enable the **ROS 2 bridge** (`isaacsim.ros2.bridge`) and publish
LiDAR/odom into ROS 2 Jazzy — the on-ramp to running slam_toolbox / Nav2 against Isaac
Sim instead of Gazebo for the ranger SLAM/Nav track (see `docs/ROADMAP.md`).
