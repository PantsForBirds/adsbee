import numpy as np, sim, json, sys
from multiprocessing import Pool
L=np.arange(-98,-77,1.0)
VAR={
 'base':{},
 'C17_10p':dict(c17=10e-12),
 'C17_22p':dict(c17=22e-12),
 'C17_47p':dict(c17=47e-12),
 'X2_TA3003':dict(filt='tfs+ta3003'),
 'fastcomp':dict(tau_c=1e-9,hyst=3e-3),
 'noD7':dict(d7=False),
 'revD_V1':dict(r11=1e3,d7=False),
 'NF+3dB':dict(nf_db=3.8),
 'X2_TA3003+C17_22p':dict(filt='tfs+ta3003',c17=22e-12),
 'budget6':dict(budget=6),
 'budget8':dict(budget=8),
 'maj1':dict(mmode=1,budget=1),
 'maj0':dict(mmode=1,budget=0),
}
TLS=[0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2]
def job(a):
    name,tl=a
    p=sim.rev('V3'); p.update(VAR[name]); p['tl_offset']=tl
    r=sim.curve(p,L,160,seed=abs(hash((name,tl)))%10000)
    fm,fc,duty=sim.false_match_rate(p,np.random.default_rng(1),ms=3.0,nt=6)
    return dict(name=name,tl=tl,p90=float(sim.p90(r)),duty=duty,fm=fm,cand=fc,
                curve=[(x['p_dbm'],x['ok']/x['n'],x['nomatch']/x['n'],x['blocked']/x['n']) for x in r])
import os
out=json.load(open('../sweep1.json'))
done={(r['name'],r['tl']) for r in out}
GRID={'C17_10p':[0.5,0.4],'C17_22p':[0.4,0.3],'C17_47p':[0.3,0.2],'X2_TA3003+C17_22p':[0.4,0.3],'revD_V1':[1.0,1.1],'fastcomp':[1.0],'base':[0.85,0.9,1.0],'X2_TA3003':[1.0]}
jobs=[(n,t) for n in GRID for t in GRID[n] if (n,t) not in done]
print(len(jobs),'jobs',flush=True)
with Pool(7) as P:
    for res in P.imap_unordered(job,jobs):
        out.append(res); print(res['name'],res['tl'],'p90 %.2f duty %.3f fm/s %.1f'%(res['p90'],res['duty'],res['fm']),flush=True)
        json.dump(out,open('../sweep.json','w'))
