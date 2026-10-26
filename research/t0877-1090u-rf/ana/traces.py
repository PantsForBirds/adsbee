import numpy as np, sim, json
out={}
for name,kw in [('tl600',dict()),('tl1000',dict(tl_offset=1.0))]:
    p=sim.rev('V3'); p.update(kw); p['lead_us']=6.0
    rng=np.random.default_rng(11)
    r=sim.run_level(p,-88,1,rng,return_traces=True)
    d,f,l,c=r['traces']
    i0=int(3e-6*sim.FS); i1=int(20e-6*sim.FS)
    sl=slice(i0,i1,2)
    out[name]=dict(t=((np.arange(len(d))[sl]-int(6e-6*sim.FS))/sim.FS*1e6).round(3).tolist(),
                   det=d[sl].round(4).tolist(),fast=f[sl].round(4).tolist(),level=l[sl].round(4).tolist(),comp=c[sl].tolist(),ok=r['ok'])
    print(name,r['ok'],r['nomatch'])
json.dump(out,open('../traces.json','w'))
