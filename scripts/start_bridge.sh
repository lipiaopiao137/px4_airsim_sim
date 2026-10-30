#!/usr/bin/env bash
# Start the AirSim -> ROS 2 sensor bridge without RViz (use start_rviz.sh).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

source_ros
stop_pidfile /tmp/bridge.pid
stop_name airsim_sensors
stop_launch '[r]os2 launch airsim_ros2_bridge'

rm -f /tmp/bridge.log
nohup ros2 launch airsim_ros2_bridge sensors.launch.py rviz:=false \
    > /tmp/bridge.log 2>&1 &
echo $! > /tmp/bridge.pid

for _ in $(seq 1 30); do
    if grep -q "AirSim connected" /tmp/bridge.log 2>/dev/null; then
        log "bridge ready: AirSim connected, /airsim/* publishing"
        exit 0
    fi
    if ! kill -0 "$(cat /tmp/bridge.pid)" 2>/dev/null; then
        tail -n 30 /tmp/bridge.log || true
        die "bridge exited during startup"
    fi
    sleep 1
done

tail -n 30 /tmp/bridge.log || true
die "bridge did not connect to AirSim within 30 s (is Blocks running?)"
