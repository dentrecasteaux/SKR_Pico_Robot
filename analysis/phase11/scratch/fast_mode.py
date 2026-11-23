"""Phase 11.3: add the 4-5 Hz mode. Joint fit over this evening's step runs."""
import json, numpy as np
from numba import njit
from scipy.optimize import differential_evolution
import phase11_model as m
G=9.80665; R2D=57.29577951308232
RUNS={'2026-11-23_203054_balance':[44.1,4.28,5.85,4.71],'2026-11-23_204006_balance':[51.4,4.37,6.89,6.48],
      '2026-11-23_204308_balance':[58.8,4.45,7.92,8.26],'2026-11-23_204602_balance':[66.1,4.54,8.96,10.03],
      '2026-11-23_204928_balance':[73.5,4.63,10.0,11.8],'2026-11-23_205340_balance':[51.4,4.37,6.89,6.48]}
@njit(cache=True)
def sim(l,tau,wz,zz,wp,zp,kind,w2,z2,c2,K,kickU,kickS,T,pre,Tc,sub):
    # kind 0: none; 1: compliant drive a=H(s)u; 2: second dipole on the reading (zero w2*c2.. see below)
    h=Tc/sub; nd=int(round(tau/h)); nsteps=int(round((T+pre)/Tc))
    buf=np.zeros(nd+1); bi=0
    th=0.0; om=0.0; w=0.0; wd=0.0; v=0.0; x=0.0; xa=0.0; va=0.0; xc=0.0; vc=0.0; y2=0.0; y2d=0.0
    c=(wp/wz)**2
    out=np.zeros((3,nsteps)); k0=int(round(pre/Tc)); kN=k0+int(round(kickS/Tc))
    for k in range(nsteps):
        thm=c*th+w; qm=c*om+wd
        if kind==2:
            thm+=y2; qm+=y2d
        u=K[0]*thm+K[1]*qm+K[2]*v+K[3]*x
        if k>=k0 and k<kN: u+=kickU
        if u>6.0: u=6.0
        if u<-6.0: u=-6.0
        v+=u*Tc; x+=v*Tc
        out[0,k]=k*Tc-pre; out[1,k]=qm*R2D; out[2,k]=u
        for s in range(sub):
            buf[bi]=u; bi=(bi+1)%(nd+1); ud=buf[bi]
            if kind==1:
                vc+=ud*h; xc+=vc*h
                a=w2*w2*(xc-xa)+2*z2*w2*(vc-va)
                va+=a*h; xa+=va*h
            else:
                a=ud
            wdd=-2*zp*wp*wd-wp*wp*w+c*((2*zz*wz-2*zp*wp)*om+(wz*wz-wp*wp)*th)
            wd+=wdd*h; w+=wd*h
            al=(G*th-a)/l
            if kind==2:
                # body-flex resonance excited by the wheel acceleration, seen by the IMU
                y2dd=-w2*w2*y2-2*z2*w2*y2d+c2*a
                y2d+=y2dd*h; y2+=y2d*h
            om+=al*h; th+=om*h
    return out
DATA={}
def data():
    if not DATA:
        for r in RUNS: DATA[r]=m.kick_ensemble(r)
    return DATA
def run_cost(p,r,K,T=1.5):
    t,mean,n=data()[r]
    o=sim(p['l'],p['tau'],2*np.pi*p['fz'],p['zz'],2*np.pi*p['fp'],p['zp'],p['kind'],2*np.pi*p.get('f2',5.0),p.get('z2',0.05),p.get('c2',0.0),
          np.asarray(K,float),2.0,0.04,T+0.004,0.1,0.002,8)
    sel=t<=T
    rq=np.interp(t[sel],o[0],o[1])-mean[0][sel]; ru=(np.interp(t[sel],o[0],o[2])-mean[2][sel])*20
    return np.mean(rq**2)+np.mean(ru**2)
def total(p): return np.mean([run_cost(p,r,K) for r,K in RUNS.items()])
BASE=['l','tau','fz','zz','fp','zp']; BB=[(0.03,0.3),(0.0,0.04),(0.8,6),(0.0,1.0),(0.8,15),(0.0,1.5)]
def fitkind(kind,extra,eb,seed=0):
    names=BASE+extra
    f=lambda x: total({**dict(zip(names,x)),'kind':kind})
    r=differential_evolution(f,BB+eb,seed=seed,maxiter=300,popsize=20,tol=1e-8)
    p={**dict(zip(names,map(float,r.x))),'kind':kind}; return p,float(np.sqrt(r.fun))
if __name__=='__main__':
    import sys
    res={}
    for kind,extra,eb in [(0,[],[]),(1,['f2','z2'],[(2,30),(0.0,1.5)]),(2,['f2','z2','c2'],[(2,15),(0.0,1.0),(-2,2)])]:
        p,c=fitkind(kind,extra,eb); res[kind]=(p,c)
        print('kind',kind,{k:round(v,4) for k,v in p.items()},'rms',round(c,2),'| per run',[round(np.sqrt(run_cost(p,r,K)),1) for r,K in RUNS.items()],flush=True)
    json.dump({str(k):v for k,v in res.items()},open('fast_fit.json','w'),indent=1)
