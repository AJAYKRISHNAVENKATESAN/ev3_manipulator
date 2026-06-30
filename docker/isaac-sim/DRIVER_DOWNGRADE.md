# NVIDIA driver downgrade — 595 → 580 (Isaac Sim 5.1 on Blackwell)

**Why:** Isaac Sim 5.1 segfaults in the RTX renderer (`libcarb.scenerenderer-rtx` /
`librtx.scenedb`) on the **RTX 5050 (Blackwell, sm_120)** under driver **595.71.05** —
even in headless mode. The 595.x branch breaks the RTX renderer / CUDA detect on
RTX 50-series. NVIDIA's tested driver for Isaac Sim 5.1 is the **580.xx production
branch**. On **Blackwell (RTX 50-series) you MUST use the open kernel module flavor** —
`nvidia-driver-580-open`. The proprietary kernel module does **not** support Blackwell
and won't load. (The 595 currently running is almost certainly already `-open` — Step 0
confirms.)

> ⚠️ Removing the running driver stops the display server — run these steps from a text
> console (TTY), not inside the desktop session, and reboot when done.

---

## Step 0 — Diagnose first

Capture how 595 was installed before changing anything:

```bash
nvidia-smi | head -5                      # confirm 595.71.05 + GPU
ubuntu-drivers devices                    # what Ubuntu recommends / sees
dpkg -l | grep -iE 'nvidia|libnvidia' | awk '{print $2, $3}'   # installed driver pkgs
modinfo nvidia | grep -iE 'license|version'   # "Dual MIT/GPL" + "(open)" = open kernel module
apt-cache policy nvidia-driver-580-open    # is 580-OPEN available? from which repo?
ls /usr/bin/nvidia-uninstall 2>/dev/null && echo "→ .run installer was used" || echo "→ apt install (good)"
```

- If `nvidia-uninstall` exists → the driver came from NVIDIA's `.run` file (different
  removal path: run `sudo nvidia-uninstall` first).
- If `apt-cache policy nvidia-driver-580-open` shows a candidate → skip Step 1.
- If it shows `Candidate: (none)` → do Step 1 to add the PPA.

---

## Step 1 — Make 580 available (only if Step 0 showed no candidate)

```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get update
apt-cache policy nvidia-driver-580-open    # should now show a 580.xx candidate
```

---

## Step 2 — Drop to a TTY and stop the display manager

The driver can't be swapped while X/Wayland is using it.

```bash
# Switch to a text console: press  Ctrl + Alt + F3   then log in.
sudo systemctl isolate multi-user.target   # stops the desktop / display manager
```

---

## Step 3 — Purge 595, install 580

```bash
# Remove the current driver (this does NOT touch nvidia-container-toolkit):
sudo apt-get purge -y 'nvidia-driver-*' 'nvidia-dkms-*' 'libnvidia-*' 'nvidia-kernel-*'
sudo apt-get autoremove -y

# Install the OPEN 580 branch (required for Blackwell):
sudo apt-get install -y nvidia-driver-580-open

# Pin it so an unattended-upgrade doesn't jump back to 595:
sudo apt-mark hold nvidia-driver-580-open
```

If `apt` complains about held/conflicting `libnvidia-*` versions, run
`sudo apt-get install -f` and re-run the install line.

---

## Step 4 — Reboot and verify

```bash
sudo reboot
```

After it comes back up (desktop should load normally):

```bash
nvidia-smi        # Driver Version should read 580.xx, GPU = RTX 5050
```

Then re-verify GPU passthrough into Docker (container toolkit was left intact):

```bash
docker run --rm --gpus all nvcr.io/nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi
```

---

## Step 5 — Resume Isaac Sim

```bash
cd ~/<path>/project-drishti
git checkout feat/isaac-sim-docker
./docker/isaac-sim/start_isaac.sh
```

Wait for **`Isaac Sim Full Streaming App is loaded.`** — on 580 the RTX renderer
should no longer segfault. Connect the WebRTC Streaming Client to `127.0.0.1`.

---

## Rollback (if 580 misbehaves)

```bash
sudo apt-mark unhold nvidia-driver-580-open
sudo apt-get purge -y 'nvidia-driver-*' 'libnvidia-*'
sudo ubuntu-drivers autoinstall   # restores the 595-open the machine shipped with
sudo reboot
```
