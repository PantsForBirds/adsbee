# t-0878: 1090 next rev, fast ADC + RP2350 (desk study)

Simulation of the 1090U receive chain (AD8313 log detector, MCP6566 comparator with tracking LEVEL, exact PIO
preamble matcher model, #398 firmware demod) against ADC / flash / I/Q alternatives. Report: `report/index.html`.

Run (Python 3.11, numpy, scipy, numba): `sim/chain3.sh` (nominal + sensitivity), then `report/analysis.py` and
`report/gen_report.py`. Paths in the scripts point at the original task folder (/workspace/agents/worker-6/t0878);
adjust `D`/`HERE` if you run them from here. `sim/softdemod_t0870.py` is the t-0870 Python port of the soft demod.
