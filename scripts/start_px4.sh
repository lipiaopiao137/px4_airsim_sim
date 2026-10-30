#!/usr/bin/env bash
# Start PX4 SITL in daemon mode and wait for the AirSim simulator link.
#
# Order matters: AirSim must already be listening on 4560.  PX4 is the TCP
# client here ("Waiting for simulator to accept connection"), AirSim is the
# server.  PX4 runs with -d so it does not start the interactive pxh shell;
# a redirected stdin otherwise floods the log with "pxh>" prompts.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

state=$(blocks_state)
for _ in $(seq 1 30); do
    [[ "$state" == "ready" || "$state" == "connected" ]] && break
    sleep 1
    state=$(blocks_state)
done

case "$state" in
    ready)
        ;;
    connected)
        die "AirSim is still connected to an old PX4 on 4560; close Blocks, run scripts/start_blocks.sh, then re-run this script"
        ;;
    rpc)
        die "Blocks RPC is up but 4560 is not listening; close Blocks and restart it"
        ;;
    *)
        die "AirSim is not ready after 30 s; run scripts/start_blocks.sh first"
        ;;
esac
pgrep -x MicroXRCEAgent >/dev/null 2>&1 || die "MicroXRCEAgent is not running; run scripts/start_agent.sh first"
[[ -x "$PX4_BUILD/bin/px4" ]] || die "PX4 binary not found: $PX4_BUILD/bin/px4"

stop_pidfile /tmp/px4_sitl.pid
stop_name px4
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0 /tmp/px4_sitl.log

cd "$PX4_BUILD"
nohup env PX4_SIM_MODEL=none_iris ./bin/px4 -d > /tmp/px4_sitl.log 2>&1 &
sleep 1
PX4_PID=$(pgrep -n -x px4 2>/dev/null || true)
[[ -n "$PX4_PID" ]] || { tail -n 30 /tmp/px4_sitl.log || true; die "PX4 failed to start"; }
echo "$PX4_PID" > /tmp/px4_sitl.pid
log "PX4 started (pid $PX4_PID), waiting for simulator and home"

deadline=$((SECONDS + 60))
while (( SECONDS < deadline )); do
    if grep -q "Simulator connected on TCP port 4560" /tmp/px4_sitl.log 2>/dev/null \
       && grep -q "Ready for takeoff" /tmp/px4_sitl.log 2>/dev/null; then
        log "PX4 ready: simulator connected, home set, ready for takeoff"
        exit 0
    fi
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        tail -n 40 /tmp/px4_sitl.log || true
        die "PX4 exited during startup"
    fi
    sleep 1
done

tail -n 40 /tmp/px4_sitl.log || true
die "PX4 did not connect to AirSim within 60 s (was TCP 4560 probed?)"
