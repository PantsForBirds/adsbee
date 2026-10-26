import numpy as np, sim, json
rng=np.random.default_rng(4); out={}
for off in [0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.35,1.5]:
    p=sim.rev('V3'); p['tl_offset']=off
    d,h=sim.noise_stats(p,rng,us=300,nt=4)
    out[off]=dict(duty=d,hist=(h/h.sum()).tolist() if h.sum() else None)
    print(off,round(d,4),flush=True)
json.dump(out,open('../duty.json','w'))
