#!/usr/bin/env bash
# One-shot launcher for Isaac Sim 5.1 (headless + WebRTC livestream).
#
# Prepares the host cache directories (owned by the container's uid 1234), then
# starts the container. First run downloads/builds the shader cache and can take
# several minutes — that is normal, not a hang. Wait for:
#     "Isaac Sim Full Streaming App is loaded."
# then connect with the Isaac Sim WebRTC Streaming Client (see README.md).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CACHE_ROOT="${HOME}/docker/isaac-sim"

# Isaac Sim 5.1 runs as uid:gid 1234 inside the container, so the mounted host
# dirs must be owned by 1234 or the container can't write its cache. Create them
# already owned by 1234 (install -d) instead of a recursive chown on every launch —
# the latter gets slow as the shader cache grows to many GB.
echo "[isaac] preparing cache dirs under ${CACHE_ROOT} (uid 1234) — may prompt for sudo ..."
for d in cache/main/ov cache/main/warp cache/computecache config \
         data/documents data/Kit logs pkg; do
  sudo install -d -o 1234 -g 1234 "${CACHE_ROOT}/${d}"
done

echo "[isaac] starting Isaac Sim (Ctrl-C to stop) ..."
docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up
