WINDOW_ID=0

function new_session() {
    SESSION=$1
    # Kill any previous session (-t -> target session, -a -> all other sessions )
    tmux kill-session -t $SESSION 2> /dev/null
    # Create new session  (-2 allows 256 colors in the terminal, -s -> session name, -d -> not attach to the new session)
    tmux -2 new-session -d -s $SESSION
    if [ -n "$2" ]; then
       cmd="tmux set-env -t $SESSION $2 $3"
       echo "Setting up $SESSION : $cmd"
       $cmd
    fi
}

function new_window() {
  if [ $WINDOW_ID -eq 0 ]; then
    # send-keys writes the string into the sesssion (-t -> target session , C-m -> press Enter Button)
    tmux rename-window -t $SESSION:0 "$1"
    tmux send-keys -t $SESSION:0 "$2" C-m

  else
    tmux new-window -t $SESSION:$WINDOW_ID -n "$1"
    tmux send-keys -t $SESSION:$WINDOW_ID "$2" C-m
  fi
  WINDOW_ID=$((WINDOW_ID+1))
}
