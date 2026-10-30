#!/usr/bin/env bash
# Run the offboard square demo in the foreground.  Any extra arguments are
# passed to ros2 launch, e.g.:
#   ./scripts/start_offboard.sh altitude:=2.0 leg_seconds:=5.0
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

source_ros
ros2 topic list 2>/dev/null | grep -qx "/fmu/out/vehicle_status" \
    || die "PX4 topics are missing; start scripts/start_px4.sh first"

stop_name offboard_square
stop_launch '[r]os2 launch px4_offboard_demo'
# Only one offboard controller may stream setpoints at a time.
stop_launch '[r]os2 launch px4_keyboard_control'
stop_launch '[p]x4_keyboard_control/(keyboard_velocity|velocity_control)'
: > /tmp/offboard.log

log "starting offboard demo; press Ctrl-C to stop"
set -o pipefail
ros2 launch px4_offboard_demo offboard_square.launch.py "$@" 2>&1 | tee /tmp/offboard.log
