#!/usr/bin/env bash
# Fly the vehicle manually with an Xbox gamepad.
#
# joy_node publishes sensor_msgs/Joy on /joy; gamepad_control maps the sticks to
# PX4 offboard velocity setpoints and uses RB / LB for takeoff and landing. The
# node runs in the background; this script tails its log in the foreground so
# Ctrl+C stops the controller and joy_node together.
#
# Extra arguments configure the controller via ros2 launch, e.g.:
#   ./scripts/start_gamepad.sh rc_control_mode:=JAPAN takeoff_altitude:=2.0
#
# Controller selection (joy 3.3.0 / SDL):
#   JOY_DEVICE_ID=0 ./scripts/start_gamepad.sh
#   JOY_DEVICE_NAME="Xbox Wireless Controller" ./scripts/start_gamepad.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

source_ros
ros2 topic list 2>/dev/null | grep -qx "/fmu/out/vehicle_status" \
    || die "PX4 topics are missing; start scripts/start_px4.sh first"

stop_launch '[r]os2 launch px4_gamepad_control'
stop_launch '[p]x4_gamepad_control/gamepad_control'
stop_launch '[r]os2 launch px4_keyboard_control'
stop_launch '[p]x4_keyboard_control/(keyboard_velocity|velocity_control)'
# Only one offboard controller may stream setpoints at a time.
stop_name offboard_square
stop_launch '[r]os2 launch px4_offboard_demo'
# Restart joy_node so /joy uses the deadzone and rate this controller expects.
stop_name joy_node

if ! compgen -G "/dev/input/event*" > /dev/null; then
    log "WARNING: no /dev/input device found; connect the Xbox controller first"
    log "  Windows (admin): usbipd list, then usbipd attach --wsl --busid <BUSID>"
fi

: > /tmp/joy.log
: > /tmp/gamepad.log

CTRL_PID=""
JOY_PID=""
cleanup() {
    if [[ -n "$CTRL_PID" ]]; then
        kill -INT "$CTRL_PID" 2>/dev/null
        wait "$CTRL_PID" 2>/dev/null
    fi
    pkill -f '/px4_gamepad_control/gamepad_control' 2>/dev/null
    if [[ -n "$JOY_PID" ]]; then
        kill "$JOY_PID" 2>/dev/null
        wait "$JOY_PID" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

joy_args=(-p device_id:="${JOY_DEVICE_ID:-0}" -p deadzone:=0.0 -p autorepeat_rate:=20.0)
if [[ -n "${JOY_DEVICE_NAME:-}" ]]; then
    joy_args+=(-p device_name:="$JOY_DEVICE_NAME")
fi
ros2 run joy joy_node --ros-args "${joy_args[@]}" > /tmp/joy.log 2>&1 &
JOY_PID=$!

joy_ready=""
for _ in $(seq 1 25); do
    kill -0 "$JOY_PID" 2>/dev/null || break
    if ros2 topic list 2>/dev/null | grep -qx "/joy"; then
        joy_ready=1
        break
    fi
    sleep 0.2
done
if ! kill -0 "$JOY_PID" 2>/dev/null; then
    tail -n 20 /tmp/joy.log || true
    die "joy_node did not start (see /tmp/joy.log)"
fi
if [[ -z "$joy_ready" ]]; then
    log "WARNING: /joy is not published; joy_node sees no controller (see /tmp/joy.log)"
fi

ros2 launch px4_gamepad_control gamepad_control.launch.py "$@" \
    > /tmp/gamepad.log 2>&1 &
CTRL_PID=$!

# Give the controller time to create its publishers and subscriptions.
for _ in $(seq 1 25); do
    grep -q "gamepad ready" /tmp/gamepad.log 2>/dev/null && break
    sleep 0.2
done
if ! grep -q "gamepad ready" /tmp/gamepad.log 2>/dev/null; then
    tail -n 20 /tmp/gamepad.log || true
    die "controller did not start (see /tmp/gamepad.log)"
fi

log "gamepad running (pid $CTRL_PID); RB take off, LB land, Ctrl+C to stop"
log "logs: /tmp/joy.log /tmp/gamepad.log"
tail -n +1 -f /tmp/gamepad.log
