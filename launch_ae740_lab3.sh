#!/usr/bin/env bash
# AE 740 Lab 3 - Team 4 simulator launcher
# Opens Terminator with a tmux 2x2 layout:
#   Gazebo | Crazyswarm2
#   Controller | Flight commands
set -euo pipefail

ROOT="$HOME/ae740_2026/ae740_crazyflie_sim_team_4"
SESSION="ae740_lab3"

for command in terminator tmux; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "Missing $command. Install with: sudo apt install $command" >&2
        exit 1
    fi
done

for required in "$ROOT/ae740_venv/bin/activate" \
                "$ROOT/ros2_ws/install/setup.bash" \
                "$ROOT/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_target_tracking.sh"; do
    if [[ ! -f "$required" ]]; then
        echo "Missing required file: $required" >&2
        exit 1
    fi
done

# Reattach without starting duplicate Gazebo/server/controller processes.
if tmux has-session -t "=$SESSION" 2>/dev/null; then
    exec terminator -e "tmux attach-session -t $SESSION"
fi

# Configure one shell per pane. PYTHONNOUSERSITE and PYTHONPATH are the
# fixes established for this machine's user-level setuptools/NumPy conflicts.
setup="cd '$ROOT' && source ae740_venv/bin/activate && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && export PYTHONNOUSERSITE=1 && export PYTHONPATH=\"\$VIRTUAL_ENV/lib/python3.10/site-packages:\$PYTHONPATH\""

# Create a detached session, then split into four panes.
tmux new-session -d -s "$SESSION" -n lab3
pane_gazebo="$(tmux display-message -p -t "$SESSION:0" '#{pane_id}')"
pane_server="$(tmux split-window -h -P -F '#{pane_id}' -t "$pane_gazebo")"
pane_controller="$(tmux split-window -v -P -F '#{pane_id}' -t "$pane_gazebo")"
pane_commands="$(tmux split-window -v -P -F '#{pane_id}' -t "$pane_server")"
tmux select-layout -t "$SESSION:0" tiled

# Enter setup commands in all panes.
for pane in "$pane_gazebo" "$pane_server" "$pane_controller" "$pane_commands"; do
    tmux send-keys -t "$pane" "$setup" C-m
done

# Start in course-recommended order; delays allow launch processes to begin.
# Always check server/controller output before issuing takeoff.
tmux send-keys -t "$pane_gazebo" "printf '\\n=== GAZEBO SITL ===\\n'; cd '$ROOT/crazyflie-firmware' && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_target_tracking.sh -t 0 -p 1 -m crazyflie" C-m
sleep 8
tmux send-keys -t "$pane_server" "printf '\\n=== CRAZYSWARM2 SERVER ===\\n'; cd '$ROOT/ros2_ws' && QT_QPA_PLATFORM=xcb ros2 launch crazyflie launch.py backend:=cflib" C-m
sleep 10
tmux send-keys -t "$pane_controller" "printf '\\n=== GEOMETRIC CONTROLLER ===\\n'; cd '$ROOT/ros2_ws' && ros2 run controller_pkg crazyflie_geometric_controller" C-m

tmux send-keys -t "$pane_commands" "printf '\\n=== FLIGHT COMMANDS: wait for Gazebo/server/controller readiness ===\\n'" C-m
tmux select-pane -t "$pane_commands"

exec terminator -e "tmux attach-session -t $SESSION"
