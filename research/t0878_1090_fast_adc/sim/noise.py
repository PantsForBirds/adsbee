"""Noise-only trigger rates: PIO matches/s per TL offset, soft preamble triggers/s per threshold and sample rate."""
import numpy as np, model as M, json, sys
rng=np.random.default_rng(7)
n=132*240; R=50; NB=int(sys.argv[1]) if len(sys.argv)>1 else 6
OFFS=[0,300,600,900,1200,1500,2000]; FSS=[8,10,12,20]; THR=np.arange(4,62,2)
dets=[]; nfm=[]
pio={o:dict(matches=0,edges=0) for o in OFFS}; soft={f:{int(t):0 for t in THR} for f in FSS}; T=0
for b in range(NB):
  det=M.detector(M.add_noise_filter(np.zeros((R,n),complex),rng)); nfm.append(det[:,1000:].mean())
  nf=float(np.mean(nfm))
  for o in OFFS:
    c=M.comparator_chain(det,nf,o)
    for r in range(R):
      c48=c[r,::5]; t=300
      while True:
        m,e=M.matcher_from(c48,t,len(c48)-500,M.PATTERN); pio[o]['edges']+=e
        if m<0: break
        pio[o]['matches']+=1; t=m+1
  for f in FSS:
    x,ph,st=M.adc_path(det,f,None,rng); spc=f//2
    for r in range(R):
      P=np.concatenate([[0],np.cumsum(x[r])]); sc=M.preamble_scores(P,1500//st,len(P)-20*spc,spc)
      for t in THR:
        # count separated crossings (dead time one chip)
        k=np.flatnonzero(sc>t); cnt=0; last=-10**9
        for kk in k:
          if kk-last>spc: cnt+=1
          last=kk
        soft[f][int(t)]+=cnt
  T+=R*(n-1500)/M.FS
out=dict(seconds=T,nf_mean=float(np.mean(nfm)),pio={o:{k:v/T for k,v in d.items()} for o,d in pio.items()},soft={f:{t:v/T for t,v in d.items()} for f,d in soft.items()})
json.dump(out,open(__import__('os').environ.get('NOISE_JSON', '../noise.json'),'w'),indent=1); print(json.dumps(out,indent=1))
