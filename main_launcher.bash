#!/bin/bash

DIR_SCRIPT="${0%/*}"

${DIR_SCRIPT}/as2_launch.bash

# if inside a tmux session detach before attaching to the session
if [ -n "$TMUX" ]; then
    tmux switch-client -t tello
else
    tmux attach -t tello:0
fi