#!/usr/bin/env bash
# Stop the ROS 2 / PX4 / DDS processes started by these scripts.
# AirSim Blocks is left running by default; pass --blocks to close it too.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

stop_launch '[r]os2 launch px4_keyboard_control'
stop_launch '[p]x4_keyboard_control/(keyboard_velocity|velocity_control)'

stop_name offboard_square
stop_launch '[r]os2 launch px4_offboard_demo'
stop_pidfile /tmp/offboard.pid

stop_name airsim_sensors
stop_launch '[r]os2 launch airsim_ros2_bridge'
stop_pidfile /tmp/bridge.pid

stop_name rviz2
stop_pidfile /tmp/rviz.pid

stop_name px4
stop_pidfile /tmp/px4_sitl.pid
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0

stop_name MicroXRCEAgent
stop_pidfile /tmp/uxrce_agent.pid

if [[ "${1:-}" == "--blocks" ]]; then
    log "closing AirSim Blocks on Windows"
    powershell.exe -NoProfile -NonInteractive -Command \
        "Get-Process -Name Blocks -ErrorAction SilentlyContinue | Stop-Process -Force" \
        >/dev/null 2>&1 || true
    for _ in $(seq 1 15); do
        [[ "$(blocks_state)" == "down" ]] && break
        sleep 1
    done
    log "AirSim state after close: $(blocks_state)"
fi

log "stopped (logs in /tmp/{px4_sitl,bridge,rviz,offboard,uxrce_agent}.log)"
