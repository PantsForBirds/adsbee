# Comparator HI statistics in noise-only stretches of real captures (worker-2 t0870 live-sky dumps, read-only).
import sys, numpy as np, json
sys.path.insert(0,'/workspace/agents/worker-2/t0870/ana')
import dumps, softdemod as sd
D=dumps.load('/workspace/agents/worker-2/t0870/data/runs/ab011-d125-shadow.log.gz')
print('loaded',len(D),flush=True)
duty=[];hist=np.zeros(60);rssi=[];n=0
for d in D:
    if d['nwords']<40: continue
    r=sd.demod(d['s'])
    if not r or r['len']!=56 or not r['crc_ok']: continue
    s=d['s']; a=r['offset']+56*12+12*8
    tail=s[a:]
    if len(tail)<300: continue
    duty.append(tail.mean()); rssi.append(d['rssi'])
    x=np.diff(np.concatenate([[0],tail,[0]]).astype(int)); st=np.where(x==1)[0]; en=np.where(x==-1)[0]
    for L in en-st: hist[min(L,59)]+=1
    n+=1
    if n>=4000: break
duty=np.array(duty)
out=dict(frames=len(duty),mean=float(duty.mean()),median=float(np.median(duty)),p10=float(np.percentile(duty,10)),p90=float(np.percentile(duty,90)),zero=float((duty==0).mean()),runhist=hist.tolist(),tail_samples=int(len(tail)),rssi_med=float(np.median(rssi)))
print(json.dumps({k:v for k,v in out.items() if k!='runhist'}))
print('run hist', hist[:16].astype(int))
json.dump(out,open('../data_realnoise.json','w'))
