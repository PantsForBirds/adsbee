#!/bin/bash
# t-0878 full model run, nominal front end (detector floor +3 dB over in-band noise, 8 MHz video), then sensitivity
# runs. Outputs in ../data (nominal) and ../data/<variant>.
cd "$(dirname "$0")"; PY=../venv/bin/python; D=../data
$PY noise.py 120 > $D/noise.out 2>&1
$PY noise_pio.py 40 > $D/noise_pio.out 2>&1
$PY run.py $D/main.json 600 -2 24 1 > $D/main.out 2>&1
$PY a_sweep.py $D/a_sweep.json 400 > $D/a_sweep.out 2>&1
$PY flash.py $D/flash.json 400 > $D/flash.out 2>&1
$PY timing.py $D/timing.json 400 > $D/timing.out 2>&1
$PY overlap.py $D/overlap.json 240 > $D/overlap.out 2>&1
$PY iq.py $D/iq.json 400 > $D/iq.out 2>&1
RATE=100 $PY run.py $D/rate100.json 300 -2 24 1 > $D/rate100.out 2>&1
for v in "fl0 0 8e6" "fl6 6 8e6"; do
  set -- $v
  export NOISE_JSON=$D/$1/noise.json DET_FLOOR_DB=$2 VIDEO_FC=$3
  $PY noise.py 60 > $D/$1/noise.out 2>&1
  $PY run.py $D/$1/main.json 300 -2 24 1 > $D/$1/main.out 2>&1
  $PY a_sweep.py $D/$1/a_sweep.json 200 > $D/$1/a_sweep.out 2>&1
  $PY flash.py $D/$1/flash.json 200 > $D/$1/flash.out 2>&1
done
echo done
