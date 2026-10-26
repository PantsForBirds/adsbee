# t-0877: 1090U RF front end review (desk study)

- `index.html`: the report (self-contained).
- `ana/sim.py`: time-domain model of the 1090U chain (noise, SAW cascade, AD8313, FAST/LEVEL/D7 network, MCP6566,
  cycle-level PIO preamble matcher ring, soft demod port + #398 retry). `ana/ref.py`: matched-filter and LR2021-like
  reference receivers. `ana/softdemod.py`: worker-2's port of mode_s_soft_demod.cc (t-0870).
- `results/`: sweep outputs used by the report (`sweep3.json` is the corrected scoring; `sweep_v1_bug.json` counted
  captures starting 13-48 cycles early as lost and is kept only for reference).
- Rebuild: paths in `make_report.py` point at /workspace/agents/worker-5/t0877/. Needs numpy, scipy, numba.
