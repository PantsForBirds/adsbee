import numpy as np, sim, json
p=sim.rev('V3'); p['lead_us']=30.0
rng=np.random.default_rng(7)
r=sim.run_level(p,-200,1,rng,return_traces=True)
d,f,l,c=r['traces']; i=int(10e-6*sim.FS)
o=dict(nf_v=r['nf_v'],det_rms_mV=float(d[i:].std()*1e3),fast_rms_mV=float(f[i:].std()*1e3),level_rms_mV=float(l[i:].std()*1e3),
       level_minus_nf_mV=float((l[i:].mean()-r['nf_v'])*1e3),duty=float(c[i:].mean()))
p['pwm_ripple']=0.031
o['pwm_ripple_levelbias_mVpp']=31.0; o['pwm_ripple_level_mVpp']=31.0*10/110
print(o); json.dump(o,open('../scope.json','w'))
