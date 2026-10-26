import numpy as np, sim, json
from multiprocessing import Pool
L=np.arange(-98,-81,1.0)
def job(tl):
    p=sim.rev('V3'); p['tl_offset']=tl; p['genie']=True
    r=sim.curve(p,L,200,seed=int(tl*100)); return tl,float(sim.p90(r)),[(x['p_dbm'],x['ok']/x['n']) for x in r]
with Pool(3) as P:
    res=P.map(job,[0.3,0.6,0.8,1.0,1.2])
for r in res: print(r[0],round(r[1],2))
json.dump(res,open('../genie.json','w'))
