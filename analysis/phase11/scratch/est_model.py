"""Model v2: rigid pendulum + delay + the IMU's complementary filter, with the
accelerometer seeing the IMU's own acceleration and ACCEL_COMP subtracting u/g."""
import numpy as np, json
from numba import njit
from scipy.optimize import differential_evolution
import phase11_model as m
G=9.80665; R2D=57.29577951308232; TAU_F=0.5
@njit(cache=True)
def sim(l,tau,h,comp,wf,zf,kf,K,kickU,kickS,T,pre,Tc,sub,gate):
    hh=Tc/sub; nd=int(round(tau/hh)); nsteps=int(round((T+pre)/Tc))
    buf=np.zeros(nd+1); bi=0
    th=0.0; om=0.0; v=0.0; x=0.0; est=0.0; ph=0.0; phd=0.0
    out=np.zeros((3,nsteps)); k0=int(round(pre/Tc)); kN=k0+int(round(kickS/Tc))
    u=0.0; a=0.0; al=0.0
    for k in range(nsteps):
        # estimator runs at the IMU rate (here: once per control step, 2 ms)
        thg=th+ph
        Aimu=a+h*al
        accp=thg-(Aimu-comp*u)/G          # accelerometer pitch (rad)
        dev=abs((Aimu-comp*u)/G)
        kk=Tc/(TAU_F+Tc)
        if gate==0 or dev<0.15:
            est=est+kk*(accp-(thg+est))   # est = estimate - true (gyro part exact)
        thm=thg+est; qm=om+phd
        u=K[0]*thm+K[1]*qm+K[2]*v+K[3]*x
        if k>=k0 and k<kN: u+=kickU
        if u>6.0: u=6.0
        if u<-6.0: u=-6.0
        v+=u*Tc; x+=v*Tc
        out[0,k]=k*Tc-pre; out[1,k]=qm*R2D; out[2,k]=u
        for s in range(sub):
            buf[bi]=u; bi=(bi+1)%(nd+1); a=buf[bi]
            al=(G*th-a)/l
            if wf>0:
                phdd=-wf*wf*ph-2*zf*wf*phd+kf*a
                phd+=phdd*hh; ph+=phd*hh
            om+=al*hh; th+=om*hh
    return out
RUNS={  # run: (gains, accel comp)
 '2026-11-23_161333_balance':([36.7,4.19,4.81,2.94],1),
 '2026-11-23_173835_balance':([36.7,4.19,4.81,2.94],1),
 '2026-11-23_203054_balance':([44.1,4.28,5.85,4.71],1),
 '2026-11-23_204006_balance':([51.4,4.37,6.89,6.48],1),
 '2026-11-23_204308_balance':([58.8,4.45,7.92,8.26],1),
 '2026-11-23_204602_balance':([66.1,4.54,8.96,10.03],1),
 '2026-11-23_204928_balance':([73.5,4.63,10.0,11.8],1),
 '2026-11-23_212700_balance':([58.8,4.45,7.92,8.26],0)}
DATA={}
def data(r):
    if r not in DATA: DATA[r]=m.kick_ensemble(r)
    return DATA[r]
def simp(p,K,comp,T=1.6):
    return sim(p['l'],p['tau'],p['h'],float(comp),2*np.pi*p.get('ff',0.0) if p.get('ff') else 0.0,p.get('zf',0.05),p.get('kf',0.0),np.asarray(K,float),2.0,0.04,T,0.1,0.002,8,int(p.get('gate',0)))
def run_cost(p,r,T=1.5):
    K,comp=RUNS[r]; t,mean,n=data(r); o=simp(p,K,comp,T+0.004); sel=t<=T
    rq=np.interp(t[sel],o[0],o[1])-mean[0][sel]; ru=(np.interp(t[sel],o[0],o[2])-mean[2][sel])*20
    return np.mean(rq**2)+np.mean(ru**2)
def total(p,runs): return np.mean([run_cost(p,r) for r in runs])
if __name__=='__main__':
    import sys
    runs=list(RUNS)
    for lab,names,b,extra in [('est',['l','tau','h'],[(0.03,0.3),(0,0.04),(-0.1,0.4)],{}),
                              ('est+flex',['l','tau','h','ff','zf','kf'],[(0.03,0.3),(0,0.04),(-0.1,0.4),(1,12),(0,1),(-0.5,0.5)],{})]:
        f=lambda x: total({**dict(zip(names,x)),**extra},runs)
        r=differential_evolution(f,b,seed=0,maxiter=200,popsize=20,tol=1e-8)
        p={**dict(zip(names,map(float,r.x))),**extra}
        print(lab,{k:round(v,4) for k,v in p.items()},'rms',round(np.sqrt(r.fun),2),'| per run',[round(np.sqrt(run_cost(p,x)),1) for x in runs],flush=True)
        json.dump(p,open(f'est_{lab}.json','w'))
