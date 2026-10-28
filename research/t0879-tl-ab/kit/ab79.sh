#!/bin/bash
# ab79.sh SECS START_INDEX [END_INDEX]: TL-offset A/B on the lab image, until ~/t0879/STOP exists. Schedule in sched.txt (re-read
# each block): tokens are TL offsets in mV (400, 500, ...) or LEARN (TL learning run, then a block at the learned value).
cd ~/t0879 || exit 1
SECS=${1:-1800}; i=${2:-0}; END=${3:-1000000}
while [ ! -e STOP ] && [ $i -lt $END ]; do
  read -r -a SCHED < sched.txt
  k=${SCHED[$((i % ${#SCHED[@]}))]}
  tag=$(printf "b%03d-%s" $i $k)
  bash block79.sh $k $tag $SECS >> runs/ab.log 2>&1
  i=$((i + 1))
done
echo "$(date -u +%FT%T) stopped at index $i" >> runs/ab.log
