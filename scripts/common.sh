#!/usr/bin/env bash
# Shared helpers for the PX4 + AirSim + ROS 2 demo scripts.
# Source this file from the other scripts; do not execute it directly.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
PX4_BUILD="$ROOT/px4/PX4-Autopilot/build/px4_sitl_default"
AGENT="$ROOT/third_party/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
RVIZ_CONFIG="$WS/install/airsim_ros2_bridge/share/airsim_ros2_bridge/config/airsim.rviz"
ROS_SETUP=/opt/ros/humble/setup.bash

BLOCKS_EXE='D:\AirSim\Blocks\Blocks\WindowsNoEditor\Blocks.exe'
BLOCKS_DIR='D:\AirSim\Blocks\Blocks\WindowsNoEditor'
BLOCKS_LOG='D:\AirSim\blocks_abs.log'

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }

source_ros() {
    # ROS setup scripts read variables that may be unset; nounset would abort.
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
    set -u
}

pid_alive() {
    [[ -n "${1:-}" && -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null
}

stop_pidfile() {
    local file=$1
    if pid_alive "$file"; then
        local pid
        pid=$(cat "$file")
        log "stopping $(basename "$file" .pid) (pid $pid)"
        kill "$pid" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.2
        done
        kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$file"
}

stop_name() {
    local name=$1 pid
    for pid in $(pgrep -x "$name" 2>/dev/null || true); do
        log "stopping $name (pid $pid)"
        kill "$pid" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.2
        done
        kill -9 "$pid" 2>/dev/null || true
    done
}

stop_launch() {
    local pattern=$1 pid
    for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
        log "stopping launch process (pid $pid)"
        kill "$pid" 2>/dev/null || true
    done
}

# AirSim port state:
#   ready     - RPC 41451 listening, simulator_mavlink 4560 listening
#               (Blocks is waiting for PX4; PX4 may be started now)
#   connected - RPC 41451 listening, 4560 established
#               (PX4 is already attached; a different PX4 cannot attach)
#   rpc       - only 41451 listening (AirSim is not in accept mode)
#   down      - nothing listening
#
# Never connect to 4560 to test it: AirSim accepts the first TCP peer as its
# SITL client, so a probe consumes the connection.
blocks_state() {
    local state
    state=$(powershell.exe -NoProfile -NonInteractive -Command "
        \$rpc = Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue
        \$sitlListen = Get-NetTCPConnection -LocalPort 4560 -State Listen -ErrorAction SilentlyContinue
        \$sitlConnected = Get-NetTCPConnection -LocalPort 4560 -State Established -ErrorAction SilentlyContinue
        if (\$rpc -and \$sitlListen) { 'ready' }
        elseif (\$rpc -and \$sitlConnected) { 'connected' }
        elseif (\$rpc) { 'rpc' }
        else { 'down' }
    " 2>/dev/null | tr -d '\r' | head -n 1)
    printf '%s\n' "${state:-down}"
}

# Blocks is usable for the rest of the stack (including "PX4 already attached").
blocks_ready() {
    local state
    state=$(blocks_state)
    [[ "$state" == "ready" || "$state" == "connected" ]]
}

# Blocks can accept a *new* PX4 connection.
blocks_accepting_px4() {
    [[ "$(blocks_state)" == "ready" ]]
}
