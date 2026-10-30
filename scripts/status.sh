#!/usr/bin/env bash
# Print a one-screen status summary of the simulation.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

echo "== processes =="
for name in MicroXRCEAgent px4 airsim_sensors rviz2 offboard_square joy_node; do
    if pgrep -x "$name" >/dev/null 2>&1; then
        printf '  %-18s running  %s\n' "$name" "$(pgrep -d, -x "$name")"
    else
        printf '  %-18s -        not running\n' "$name"
    fi
done

# Controller node names are longer than the 15-character /proc comm limit, so
# match their installed path instead of using pgrep -x.
for target in px4_keyboard_control/keyboard_velocity \
              px4_keyboard_control/velocity_control \
              px4_gamepad_control/gamepad_control; do
    pids=$(pgrep -d, -f "/$target" 2>/dev/null || true)
    if [[ -n "$pids" ]]; then
        printf '  %-18s running  %s\n' "${target##*/}" "$pids"
    else
        printf '  %-18s -        not running\n' "${target##*/}"
    fi
done

echo "== AirSim =="
echo "  state: $(blocks_state) (ready=listening for PX4, connected=PX4 attached)"
powershell.exe -NoProfile -NonInteractive -Command \
    "Get-NetTCPConnection -LocalPort 41451,4560 -ErrorAction SilentlyContinue |
     Select-Object LocalAddress,LocalPort,RemoteAddress,RemotePort,State,OwningProcess |
     Format-Table -AutoSize" 2>/dev/null || true

echo "== DDS agent =="
ss -lunp 2>/dev/null | grep 8888 || echo "  UDP 8888 not listening"

echo "== ROS topics =="
source_ros
timeout 8 ros2 topic list 2>/dev/null | grep -E '^/(fmu|airsim)' || true
