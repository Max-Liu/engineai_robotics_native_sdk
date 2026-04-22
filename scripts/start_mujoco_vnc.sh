#!/bin/bash
# Run MuJoCo simulation inside engineai_robotics_env container and view via browser (noVNC).
# Designed for macOS + Docker Desktop where GPU passthrough is unavailable.
#
# Usage:
#   ./scripts/start_mujoco_vnc.sh [robot_name] [width] [height]
#
# Examples:
#   ./scripts/start_mujoco_vnc.sh pm01_edu           # 1280x720 (default)
#   ./scripts/start_mujoco_vnc.sh pm01_edu 1920 1080 # higher resolution (slower)
#   MUJOCO_RENDER_THREADS=6 ./scripts/start_mujoco_vnc.sh pm01_edu
#   VNC_SCALE=0.75 ./scripts/start_mujoco_vnc.sh pm01_edu
#   DISPLAY_DEPTH=16 ./scripts/start_mujoco_vnc.sh pm01_edu
#   ./scripts/start_mujoco_vnc.sh --stop             # stop everything

set -e

ROBOT=${1:-pm01_edu}
DISP_W=${2:-1920}
DISP_H=${3:-1080}
DISPLAY_DEPTH=${DISPLAY_DEPTH:-24}
CONTAINER=engineai_robotics_env
VNC_PORT=5900
PROXY_PORT=6080
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROXY_SCRIPT="$SCRIPT_DIR/vnc_proxy.py"

# ── Stop mode ────────────────────────────────────────────────────────────────
if [ "$1" = "--stop" ]; then
  echo "[INFO] Stopping MuJoCo simulation and VNC services..."
  pkill -f vnc_proxy.py 2>/dev/null || true
  docker exec "$CONTAINER" bash -c "
    pkill engineai_robotics_simulation_mujoco 2>/dev/null || true
    pkill x11vnc 2>/dev/null || true
    pkill Xvfb 2>/dev/null || true
  " 2>/dev/null || true
  echo "[INFO] All stopped."
  exit 0
fi

# ── Pre-flight checks ─────────────────────────────────────────────────────────
if ! docker ps --filter "name=${CONTAINER}" --format "{{.Names}}" | grep -q "^${CONTAINER}$"; then
  echo "[ERROR] Container '${CONTAINER}' is not running."
  echo "        Start it first: cd docker/dockers/engineai_robotics_env && docker compose up -d"
  exit 1
fi

if [ ! -f "$PROXY_SCRIPT" ]; then
  echo "[ERROR] Proxy script not found: $PROXY_SCRIPT"
  exit 1
fi

if ! python3 -c "import websockets" 2>/dev/null; then
  echo "[INFO] Installing websockets Python package..."
  pip3 install -q websockets
fi

# ── Install required tools in container (idempotent) ─────────────────────────
echo "[INFO] Checking required tools in container..."
docker exec -u root "$CONTAINER" bash -c "
  missing=''
  which Xvfb   > /dev/null 2>&1 || missing=\"\$missing xvfb\"
  which x11vnc > /dev/null 2>&1 || missing=\"\$missing x11vnc\"
  [ -d /usr/share/novnc ]        || missing=\"\$missing novnc\"
  if [ -n \"\$missing\" ]; then
    echo '[INFO] Installing:\$missing'
    apt-get update -q && apt-get install -y -q \$missing
  fi
" 2>/dev/null

# ── Tear down any previous session ───────────────────────────────────────────
echo "[INFO] Clearing previous session..."
pkill -f vnc_proxy.py 2>/dev/null || true
lsof -ti :${PROXY_PORT} | xargs kill -9 2>/dev/null || true
docker exec "$CONTAINER" bash -c "
  pkill engineai_robotics_simulation_mujoco 2>/dev/null || true
  pkill x11vnc 2>/dev/null || true
  pkill Xvfb 2>/dev/null || true
  sleep 1
" 2>/dev/null || true

# ── Start Xvfb ────────────────────────────────────────────────────────────────
echo "[INFO] Starting Xvfb :99 at ${DISP_W}x${DISP_H}x${DISPLAY_DEPTH}..."
docker exec -d "$CONTAINER" bash -c \
  "Xvfb :99 -screen 0 ${DISP_W}x${DISP_H}x${DISPLAY_DEPTH} -ac +extension GLX +render -noreset > /tmp/xvfb.log 2>&1"
sleep 1

# ── Start MuJoCo ─────────────────────────────────────────────────────────────
NUM_CORES=$(docker exec "$CONTAINER" nproc)
RENDER_THREADS=${MUJOCO_RENDER_THREADS:-$((NUM_CORES - 2))}
if [ "$RENDER_THREADS" -lt 1 ]; then
  RENDER_THREADS=$NUM_CORES
fi
if [ "$RENDER_THREADS" -gt "$NUM_CORES" ]; then
  RENDER_THREADS=$NUM_CORES
fi
VNC_SCALE=${VNC_SCALE:-1}
VNC_WAIT_MS=${VNC_WAIT_MS:-5}
VNC_DEFER_MS=${VNC_DEFER_MS:-5}
SCALE_FLAG=""
if [ "$VNC_SCALE" != "1" ] && [ "$VNC_SCALE" != "1.0" ]; then
  SCALE_FLAG="-scale ${VNC_SCALE}"
fi

echo "[INFO] Starting MuJoCo (robot=${ROBOT}, llvmpipe x${RENDER_THREADS}/${NUM_CORES} threads)..."
docker exec -d "$CONTAINER" bash -c "
  DISPLAY=:99 \
  LIBGL_ALWAYS_SOFTWARE=1 \
  MESA_GL_VERSION_OVERRIDE=4.5COMPAT \
  GALLIUM_DRIVER=llvmpipe \
  LP_NUM_THREADS=${RENDER_THREADS} \
  /Users/max/Project/engineai_robotics_native_sdk/scripts/run_mujoco.sh ${ROBOT} > /tmp/mujoco.log 2>&1
"
sleep 3

if docker exec "$CONTAINER" bash -c "grep -q 'ERROR' /tmp/mujoco.log 2>/dev/null"; then
  echo "[ERROR] MuJoCo failed to start:"
  docker exec "$CONTAINER" cat /tmp/mujoco.log
  exit 1
fi

# ── Start x11vnc ──────────────────────────────────────────────────────────────
# -scale 0.75     : optional server-side downscale before VNC encoding
# -noxdamage      : avoid XDAMAGE glitches under software rendering
# -wait 5 -defer 5: 5 ms poll interval for low latency
# -noscr          : disable scrolling copy optimisation (unreliable with llvmpipe)
echo "[INFO] Starting x11vnc on port ${VNC_PORT} (scale=${VNC_SCALE}, wait=${VNC_WAIT_MS}ms, defer=${VNC_DEFER_MS}ms)..."
docker exec -d "$CONTAINER" bash -c "
  x11vnc -display :99 \
    -nopw -listen 0.0.0.0 -rfbport ${VNC_PORT} \
    -forever -shared \
    -noxdamage -wait ${VNC_WAIT_MS} -defer ${VNC_DEFER_MS} \
    -noscr -nocursorshape \
    ${SCALE_FLAG} \
    > /tmp/x11vnc.log 2>&1
"
sleep 2

# ── Start WebSocket proxy on Mac ───────────────────────────────────────────────
# vnc_proxy.py: asyncio WebSocket server that serves noVNC HTML and bridges
# WebSocket frames directly to x11vnc via `docker exec socat`.
# This is one hop fewer than the previous websockify-in-container approach.
echo "[INFO] Starting WebSocket proxy on Mac (port ${PROXY_PORT})..."
python3 "$PROXY_SCRIPT" "$PROXY_PORT" > /tmp/vnc_proxy.log 2>&1 &
sleep 2

if ! lsof -ti :${PROXY_PORT} > /dev/null 2>&1; then
  echo "[ERROR] Proxy failed to start. Log:"
  cat /tmp/vnc_proxy.log
  exit 1
fi

# ── Open browser ──────────────────────────────────────────────────────────────
echo "[INFO] Opening browser..."
open "http://localhost:${PROXY_PORT}/vnc.html?resize=scale"

echo ""
echo "================================================"
echo " MuJoCo simulation is running!"
echo ""
echo " Browser : http://localhost:${PROXY_PORT}/vnc.html"
echo "           Click 'Connect' (no password)"
echo " Robot   : ${ROBOT}"
echo " Display : ${DISP_W}x${DISP_H}x${DISPLAY_DEPTH}  Renderer: llvmpipe x${RENDER_THREADS}/${NUM_CORES}  VNC scale: ${VNC_SCALE}"
echo ""
echo " To stop : ./scripts/start_mujoco_vnc.sh --stop"
echo "================================================"
