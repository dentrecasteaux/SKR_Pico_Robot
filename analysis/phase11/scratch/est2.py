"""Model v3: rigid pendulum + delay + complementary filter with IMU-height accel and ACCEL_COMP
   + joint-compliance pole/zero pair on what the IMU sees."""
import numpy as np, json
from numba import njit
from scipy.optimize import differential_evolution
import phase11_model as m
G=9.80665; R2D=57.29577951308232; TAU_F=0.5
@njit(cache=True)
def sim(l,tau,h,comp,wz,zz,wp,zp,K,kickU,kickS,T,pre,Tc,sub):
    hh=Tc/sub; nd=int(round(tau/hh)); nsteps=int(round((T+pre)/Tc))
    buf=np.zeros(nd+1); bi=0
    th=0.0; om=0.0; v=0.0; x=0.0; est=0.0; w=0.0; wd=0.0
    use=wz>0; c=(wp/wz)**2 if use else 1.0
    out=np.zeros((3,nsteps)); k0=int(round(pre/Tc)); kN=k0+int(round(kickS/Tc))
    u=0.0; a=0.0; al=0.0; kk=Tc/(TAU_F+Tc)
    for k in range(nsteps):
        if use:
            thg=c*th+w; qm=c*om+wd
        else:
            thg=th; qm=om
        accp=thg-(a+h*al-comp*u)/G
        est=est+kk*(accp-(thg+est))
        thm=thg+est
        u=K[0]*thm+K[1]*qm+K[2]*v+K[3]*x
        if k>=k0 and k<kN: u+=kickU
        if u>6.0: u=6.0
        if u<-6.0: u=-6.0
        v+=u*Tc; x+=v*Tc
        out[0,k]=k*Tc-pre; out[1,k]=qm*R2D; out[2,k]=u
        for s in range(sub):
            buf[bi]=u; bi=(bi+1)%(nd+1); a=buf[bi]
            al=(G*th-a)/l
            if use:
                wdd=-2*zp*wp*wd-wp*wp*w+c*((2*zz*wz-2*zp*wp)*om+(wz*wz-wp*wp)*th)
                wd+=wdd*hh; w+=wd*hh
            om+=al*hh; th+=om*hh
    return out
from est_model import RUNS
DATA={}
def data(r):
    if r not in DATA: DATA[r]=m.kick_ensemble(r)
    return DATA[r]
def simp(p,K,comp,T=1.6):
    return sim(p['l'],p['tau'],p['h'],float(comp),2*np.pi*p.get('fz',0.0),p.get('zz',0.0),2*np.pi*p.get('fp',1.0),p.get('zp',0.0),np.asarray(K,float),2.0,0.04,T,0.1,0.002,8)
def run_cost(p,r,T=1.5):
    K,comp=RUNS[r]; t,mean,n=data(r); o=simp(p,K,comp,T+0.004); sel=t<=T
    rq=np.interp(t[sel],o[0],o[1])-mean[0][sel]; ru=(np.interp(t[sel],o[0],o[2])-mean[2][sel])*20
    return np.mean(rq**2)+np.mean(ru**2)
NAMES=['l','tau','h','fz','zz','fp','zp']; B=[(0.05,0.2),(0,0.03),(0,0.35),(0.8,8),(0,1),(0.8,12),(0,1.5)]
def fit(runs,seed=0,maxiter=200):
    f=lambda x: np.mean([run_cost(dict(zip(NAMES,x)),r) for r in runs])
    r=differential_evolution(f,B,seed=seed,maxiter=maxiter,popsize=20,tol=1e-8)
    return dict(zip(NAMES,map(float,r.x))),float(np.sqrt(r.fun))
if __name__=='__main__':
    import sys
    sets={'evening':[r for r in RUNS if r[11:13]>='20' or r[11:13]=='21'],'day':['2026-11-23_161333_balance','2026-11-23_173835_balance']}
    out={}
    for name in sys.argv[1:]:
        p,c=fit(sets[name]); out[name]=p
        print(name,{k:round(v,4) for k,v in p.items()},'rms',round(c,2),'| per run',{r[11:17]:round(np.sqrt(run_cost(p,r)),1) for r in RUNS},flush=True)
        json.dump(p,open(f'est2_{name}.json','w'))
