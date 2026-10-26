import numpy as np, sim, json
from multiprocessing import Pool
L=np.arange(-98,-77,1.0)
VAR={'base':{},'C17_10p':dict(c17=10e-12),'C17_22p':dict(c17=22e-12),'X2_TA3003':dict(filt='tfs+ta3003'),
 'X2_TA3003+C17_22p':dict(filt='tfs+ta3003',c17=22e-12),'maj0':dict(mmode=1,budget=0),'NF+3dB':dict(nf_db=3.8),
 'fastcomp':dict(tau_c=1e-9,hyst=3e-3),'revD_V1':dict(r11=1e3,d7=False),
 'cw-95':dict(cw_dbm=-95),'cw-92':dict(cw_dbm=-92)}
GRID={'base':[0.6,0.7,0.8,0.9,1.0],'C17_10p':[0.5,0.6,0.7],'C17_22p':[0.4,0.5],'X2_TA3003':[0.8,0.9],'X2_TA3003+C17_22p':[0.4,0.5],
 'maj0':[0.6,0.7],'NF+3dB':[0.8,0.9],'fastcomp':[0.8,0.9],'revD_V1':[0.9,1.0],'cw-95':[0.5,0.6],'cw-92':[0.5,0.6]}
def job(a):
    name,tl=a
    p=sim.rev('V3'); p.update(VAR[name]); p['tl_offset']=tl
    r=sim.curve(p,L,200,seed=abs(hash((name,tl,3)))%10000)
    return dict(name=name,tl=tl,p90=float(sim.p90(r)),nf=r[0]['nf_v'],curve=[(x['p_dbm'],x['ok']/x['n'],x['nomatch']/x['n'],x['blocked']/x['n']) for x in r])
jobs=[(n,t) for n in GRID for t in GRID[n]]
out=[]
with Pool(7) as P:
    for res in P.imap_unordered(job,jobs):
        out.append(res); print(res['name'],res['tl'],'p90 %.2f'%res['p90'],flush=True)
        json.dump(out,open('../sweep3.json','w'))
