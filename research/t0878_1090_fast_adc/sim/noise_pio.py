"""Noise-only PIO match rates for (TL offset, mismatch budget) pairs, matcher restarted after each match."""
import numpy as np, model as M, json, sys
rng=np.random.default_rng(11); n=132*240; R=50; NB=int(sys.argv[1])
NF=json.load(open(__import__('os').environ.get('NOISE_JSON', '../noise.json')))['nf_mean']
CFG=[(o,b) for o in (300,600,900,1200,1500) for b in (4,6,8,10,12)]
cnt={c:0 for c in CFG}; T=0
for k in range(NB):
  det=M.detector(M.add_noise_filter(np.zeros((R,n),complex),rng))
  for o in sorted({c[0] for c in CFG}):
    c=M.comparator_chain(det,NF,o)
    for b in sorted({c[1] for c in CFG}):
      for r in range(R):
        c48=c[r,::5]; t=300
        while True:
          m,e=M.matcher_from(c48,t,len(c48)-500,M.PATTERN,b)
          if m<0: break
          cnt[(o,b)]+=1; t=m+1
  T+=R*(n-1500)/M.FS
out={f'{o}_{b}':v/T for (o,b),v in cnt.items()}; out['seconds']=T
json.dump(out,open('../noise_pio.json','w'),indent=1); print(out)
