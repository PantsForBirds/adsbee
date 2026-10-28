#!/bin/bash
# stop.sh: stop this task's A/B loop and its console logger only (not the websocket logger).
cd ~/t0879 && touch STOP
for p in $(pgrep -f "[a]b79\.sh") $(pgrep -f "[b]lock79\.sh") $(pgrep -f "[c]on79\.py"); do kill $p; done
sleep 2; pgrep -af "[a]b79\.sh|[c]on79\.py|[b]lock79\.sh" || echo stopped
