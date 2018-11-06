#!/bin/bash
tmux new -s "remote" -d "/bin/bash/"
tmux run-shell -t "remote:0" "start.bash"
tmux attach -t "remote" -d

