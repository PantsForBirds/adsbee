# AD8313 input match (C11=C18=3 pF in series -> 1.5 pF, L6=10 nH across inputs) at 1090 MHz.
# Chip modelled as Rin || Cin (incl. pads); 900 MHz eval values back-solve to ~330 ohm || 2.5 pF.
import numpy as np, json
f=np.linspace(0.9e9,1.3e9,401); w=2*np.pi*f
res={}
for Rin in [250,330,450]:
  for Cin in [1.5,2.0,2.5]:
    Y=1/Rin+1j*w*Cin*1e-12+1/(1j*w*10e-9)
    Zl=1/Y; Zin=Zl+1/(1j*w*1.5e-12)
    G=(Zin-50)/(Zin+50)
    # voltage across chip for 1 W available from 50 ohm vs across matched 50 ohm load
    I=2*np.sqrt(50)/(Zin+50)  # source EMF 2*sqrt(50*P)
    Vc=np.abs(I*Zl); gain=20*np.log10(Vc/np.sqrt(50))
    i=np.argmin(abs(f-1.09e9))
    res[f'{Rin}/{Cin}']=dict(gain=float(gain[i]),rl=float(-20*np.log10(abs(G[i]))),fpk=float(f[np.argmax(gain)]/1e6),gpk=float(gain.max()))
    print(Rin,Cin,'gain@1090 %.1f dB  RL %.1f dB  peak %.0f MHz %.1f dB'%(gain[i],-20*np.log10(abs(G[i])),f[np.argmax(gain)]/1e6,gain.max()))
json.dump(res,open('../match.json','w'))
