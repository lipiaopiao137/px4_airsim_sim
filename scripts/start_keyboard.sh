#!/usr/bin/env bash
# Fly the vehicle manually with keyboard input.
#
# The offboard controller runs in the background; the keyboard node runs in the
# foreground because only a foreground `ros2 run` inherits the terminal stdin
# that it reads (ros2 launch pipes stdin, so key presses would be lost).
#
# Extra arguments configure the controller via ros2 launch, e.g.:
#   ./scripts/start_keyboard.sh takeoff_altitude:=2.0 cmd_timeout:=0.5
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

source_ros
ros2 topic list 2>/dev/null | grep -qx "/fmu/out/vehicle_status" \
    || die "PX4 topics are missing; start scripts/start_px4.sh first"

stop_launch '[r]os2 launch px4_keyboard_control'
stop_launch '[p]x4_keyboard_control/(keyboard_velocity|velocity_control)'
stop_launch '[r]os2 launch px4_gamepad_control'
stop_launch '[p]x4_gamepad_control/gamepad_control'
# Only one offboard controller may stream setpoints at a time.
stop_name offboard_square
stop_launch '[r]os2 launch px4_offboard_demo'

: > /tmp/keyboard.log
: > /tmp/velocity_control.log

ros2 launch px4_keyboard_control velocity_control.launch.py "$@" \
    > /tmp/velocity_control.log 2>&1 &
CTRL_PID=$!
cleanup() {
    # launch only tears down its children on SIGINT, not SIGTERM.
    kill -INT "$CTRL_PID" 2>/dev/null
    wait "$CTRL_PID" 2>/dev/null
    pkill -f '/px4_keyboard_control/velocity_control' 2>/dev/null
}
trap cleanup EXIT INT TERM

# Give the controller time to create its publishers before keys can arrive.
for _ in $(seq 1 20); do
    grep -q "keyboard velocity control ready" /tmp/velocity_control.log 2>/dev/null && break
    sleep 0.2
done
if ! grep -q "keyboard velocity control ready" /tmp/velocity_control.log 2>/dev/null; then
    tail -n 20 /tmp/velocity_control.log || true
    die "controller did not start (see /tmp/velocity_control.log)"
fi

log "controller running (pid $CTRL_PID); press i to take off, g to land, q to quit"
set -o pipefail
ros2 run px4_keyboard_control keyboard_velocity 2>&1 | tee /tmp/keyboard.log
