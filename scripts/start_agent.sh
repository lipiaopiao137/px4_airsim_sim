#!/usr/bin/env bash
# Start the Micro-XRCE-DDS-Agent (UDP 8888) in the background.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

if pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
    log "MicroXRCEAgent already running (pid $(pgrep -n -x MicroXRCEAgent))"
    exit 0
fi

[[ -x "$AGENT" ]] || die "agent not built: $AGENT"

nohup "$AGENT" udp4 -p 8888 > /tmp/uxrce_agent.log 2>&1 &
echo $! > /tmp/uxrce_agent.pid

sleep 1
if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
    tail -n 20 /tmp/uxrce_agent.log || true
    die "MicroXRCEAgent failed to start"
fi

log "MicroXRCEAgent started (pid $(pgrep -n -x MicroXRCEAgent)), UDP 8888"
