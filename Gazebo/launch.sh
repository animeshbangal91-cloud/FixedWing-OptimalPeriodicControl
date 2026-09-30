#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME="$HOME/stallion_sitl"
SESSION=stallion
MISSION_ARGS=""
NO_ATTACH=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mission)
            [[ $# -ge 2 && -f "$2" ]] || { echo "--mission requires an existing JSON file"; exit 1; }
            printf -v MISSION_ARGS ' --mission %q' "$(realpath "$2")"
            shift 2 ;;
        --no-attach) NO_ATTACH=1; shift ;;
        *) echo "Usage: $0 [--mission mission.json] [--no-attach]"; exit 1 ;;
    esac
done
if [[ -n "$MISSION_ARGS" ]] && pgrep -x px4 >/dev/null; then
    echo "Stop the existing simulation before starting a new energy mission."
    exit 1
fi
printf -v start 'bash %q; exec bash' "$RUNTIME/start_px4.sh"
TUNING_ARGS=""
if [[ -f "$HERE/tuning.json" ]]; then
    printf -v TUNING_ARGS ' --tuning-config %q' "$HERE/tuning.json"
fi
printf -v stream 'sleep 15; flock -n /tmp/px4-fixedwing-demo.lock %q -u %q --model-config %q --results %q%s; exec bash' \
    "$HOME/venvs/px4/bin/python" "$HERE/../fixedwing_demo/stream_mission.py" \
    "$RUNTIME/model_report.json" "$HERE/results" "$TUNING_ARGS$MISSION_ARGS"
printf -v gui 'source %q; export GZ_IP=127.0.0.1; LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb gz sim -g -v 3; exec bash' "$RUNTIME/gz_env.sh"
if [[ ! -f "$RUNTIME/model_report.json" || ! -f "$HOME/stallion_build/libStallionAerodynamics.so" ]]; then
    echo "Run first: bash $HERE/setup.sh"
    exit 1
fi
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    if pgrep -x px4 >/dev/null || pgrep -f '[g]z sim.* -s ' >/dev/null; then
        echo "Stop the existing PX4/Gazebo simulation before launching the separate Stallion model."
        exit 1
    fi
    tmux new-session -d -s "$SESSION" -n px4 -c "$RUNTIME" "$start"
    tmux new-window -t "$SESSION":1 -n stream -c "$RUNTIME" "$stream"
    tmux new-window -t "$SESSION":2 -n gazebo -c "$RUNTIME" "$gui"
elif ! pgrep -x px4 >/dev/null; then
    # Restart stopped demo windows only when they have returned to an idle shell.
    for window in 0 1 2; do
        current=$(tmux display-message -p -t "$SESSION:$window" '#{pane_current_command}')
        if [[ "$current" != bash ]]; then
            echo "Window $window is running $current. Stop it before restarting this session."
            exit 1
        fi
    done
    if pgrep -f '[g]z sim.* -s ' >/dev/null; then
        echo "A Gazebo server is still running. Stop it before resetting the model."
        exit 1
    fi
    tmux respawn-window -k -t "$SESSION":0 -c "$RUNTIME" "$start"
    tmux respawn-window -k -t "$SESSION":1 -c "$RUNTIME" "$stream"
    tmux respawn-window -k -t "$SESSION":2 -c "$RUNTIME" "$gui"
fi
tmux select-window -t "$SESSION":1
echo "Stallion: wait for READY, then Ctrl+B 0: commander mode offboard; commander arm"
if [[ "$NO_ATTACH" == 1 ]]; then exit 0; fi
if [[ -n "${TMUX:-}" ]]; then tmux switch-client -t "$SESSION"; else tmux attach -t "$SESSION"; fi
