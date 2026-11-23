import numpy as np, pandas as pd, sys
from scipy.signal import welch
R='/mnt/user-data/uploads/SKR-Pico-Robot/runs/'
for arg in sys.argv[1:]:
    r,lab=arg.split('=')
    d=pd.read_csv(R+r+'/data.csv'); run=d[d['mode'].to_numpy()==6]
    q=run.q_dps.to_numpy(); t=run.t_s.to_numpy(); u=run.u_mm_s2.to_numpy()/1000; th=run.pitch_deg.to_numpy(); v=run.v_left_mm_s.to_numpy()
    f,P=welch(q,fs=250,nperseg=2048); df=f[1]
    tt=t>t[0]+10
    print(f"{lab:18s}",' '.join(f"{a}-{b}Hz {np.sqrt(P[(f>=a)&(f<b)].sum()*df):.1f}" for a,b in [(1.5,3),(3,3.8),(3.8,5.2),(5.2,8),(8,30)]),'| u std',u[tt].std().round(2),'| pitch std',th[tt].std().round(2),'| v std',v[tt].std().round(1))
