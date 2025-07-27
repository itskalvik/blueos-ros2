#!/bin/bash

echo "Starting.."
[ ! -e /var/run/nginx.pid ] && nginx&

# Create a new tmux session
tmux -f /etc/tmux.conf start-server
tmux new -d -s "ROS2"

# Split the screen into a 2x2 matrix
tmux split-window -v
tmux split-window -h
tmux select-pane -t 0
tmux split-window -h

tmux send-keys -t 0
tmux send-keys -t 1 
tmux send-keys -t 2 
tmux send-keys -t 3 

function create_service {
    tmux new -d -s "$1" || true
    SESSION_NAME="$1:0"
    # Set all necessary environment variables for the new tmux session
    for NAME in $(compgen -v | grep MAV_); do
        VALUE=${!NAME}
        tmux setenv -t $SESSION_NAME -g $NAME $VALUE
    done
    tmux send-keys -t $SESSION_NAME "$2" C-m
}

create_service 'ttyd' 'ttyd -p 88 sh -c "/usr/bin/tmux attach -t ROS2 || /usr/bin/tmux new -s user_terminal"'

echo "Done!"
sleep infinity