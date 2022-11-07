#!/bin/bash

tmux ls | grep -Po "tello" | xargs -I % sh -c 'tmux kill-session -t %'
