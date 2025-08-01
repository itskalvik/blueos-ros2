#!/bin/bash

echo "Setting up tmux session..."

# Start tmux server
tmux -f /etc/tmux.conf start-server

# Kill any existing ROS2 session
tmux kill-session -t ROS2 2>/dev/null || true

# Create a new detached session with a persistent shell that keeps running
tmux new-session -d -s 'ROS2' -n 'main' 'bash -c "echo \"ROS2 Terminal Ready\"; while true; do sleep 1; done"'

# Split the screen into a 2x2 matrix with persistent shells
tmux split-window -v -t ROS2:main 'bash -c "echo \"Pane 1 Ready\"; while true; do sleep 1; done"'
tmux split-window -h -t ROS2:main.0 'bash -c "echo \"Pane 2 Ready\"; while true; do sleep 1; done"'
tmux select-pane -t ROS2:main.0
tmux split-window -h -t ROS2:main.0 'bash -c "echo \"Pane 3 Ready\"; while true; do sleep 1; done"'

# Send initial commands to each pane
tmux send-keys -t ROS2:main.0 'echo "Pane 0 - ROS2 Terminal"' C-m
tmux send-keys -t ROS2:main.1 'echo "Pane 1 - ROS2 Terminal"' C-m
tmux send-keys -t ROS2:main.2 'echo "Pane 2 - ROS2 Terminal"' C-m
tmux send-keys -t ROS2:main.3 'echo "Pane 3 - ROS2 Terminal"' C-m

echo "tmux session created successfully"
tmux list-sessions 