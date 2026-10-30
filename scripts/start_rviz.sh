#!/usr/bin/env bash
# Start RViz2 with the saved 1270x720 layout.
#
# WSL2 exposes no /dev/dri on this machine, so RViz uses Mesa llvmpipe.
# LIBGL_ALWAYS_SOFTWARE=1 avoids the "failed to create GL context" path.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

source_ros
[[ -f "$RVIZ_CONFIG" ]] || die "RViz config not found: $RVIZ_CONFIG"

stop_pidfile /tmp/rviz.pid
stop_name rviz2

export LIBGL_ALWAYS_SOFTWARE=1
rm -f /tmp/rviz.log
# setsid detaches RViz into its own session.  A bare `nohup ... &` exits on its
# own a few seconds after the GL context is up when the calling shell ends.
setsid nohup rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz.log 2>&1 < /dev/null &

for _ in $(seq 1 20); do
    if pgrep -x rviz2 >/dev/null 2>&1; then
        pgrep -n -x rviz2 > /tmp/rviz.pid
        log "RViz2 started (pid $(cat /tmp/rviz.pid)), layout from $RVIZ_CONFIG"
        exit 0
    fi
    sleep 1
done

tail -n 30 /tmp/rviz.log || true
die "RViz2 failed to start"
