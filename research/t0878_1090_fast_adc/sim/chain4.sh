#!/bin/bash
# Sensitivity variants (run alongside chain3's tail). $1 = variant list.
cd "$(dirname "$0")"; PY=../venv/bin/python; D=../data
for v in "$@"; do
  set -- $v
  export NOISE_JSON=$D/$1/noise.json DET_FLOOR_DB=$2 VIDEO_FC=$3
  $PY noise.py 60 > $D/$1/noise.out 2>&1
  $PY run.py $D/$1/main.json 300 -2 24 1 > $D/$1/main.out 2>&1
  $PY a_sweep.py $D/$1/a_sweep.json 200 > $D/$1/a_sweep.out 2>&1
  $PY flash.py $D/$1/flash.json 200 > $D/$1/flash.out 2>&1
done
