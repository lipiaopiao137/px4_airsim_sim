#!/usr/bin/env bash
# Print real publish rates for the /airsim/* topics.
#
# `ros2 topic hz` is unreliable on this machine: with RViz2 + bridge + PX4 all
# running it regularly reports nothing at all while the data is in fact flowing.
# This runs the C++ topic_rates node, which subscribes to every /airsim topic
# and counts messages instead.
#
#   ./scripts/topic_rates.sh [seconds]      # default 20 s
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

source_ros

WATCH_SECONDS="${1:-20}"
DURATION=$(printf '%.1f' "$WATCH_SECONDS")
log "counting /airsim/* messages for ${DURATION}s (start RViz/bridge first)"

ros2 run airsim_ros2_bridge topic_rates --ros-args -p duration_s:="${DURATION}"
