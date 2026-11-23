import numpy as np, json
from scipy.optimize import differential_evolution
from est2_lin import modes
S1=[44.1,4.28,5.85,4.71];S2=[51.4,4.37,6.89,6.48];S3=[58.8,4.45,7.92,8.26];S4=[66.1,4.54,8.96,10.03];S5=[73.5,4.63,10,11.8]
TG=[(S1,1,[(2.58,0.20)]),(S2,1,[(2.88,0.31)]),(S3,1,[(4.4,0.03)]),(S4,1,[(4.8,0.02)]),(S5,1,[(5.15,0.015)]),(S3,0,[(2.12,0.08)])]
def err(p):
    e=0
    for K,c,tg in TG:
        md=[x for x in modes(p,K,c) if x[0]>0.8]
        if not md: return 1e6
        for f,z in tg:
            j=min(md,key=lambda q:abs(q[0]-f)); e+=((j[0]-f)/0.1)**2+((j[1]-z)/0.03)**2
        # no other mode may be less damped than 0.1 below what is seen, (unseen modes must be well damped)
        lowest=min(md,key=lambda q:q[1])
        if lowest[1]<0: e+=200
        if c==0 or K is S1 or K is S2:   # no fast mode seen: anything else must be >=0.2
            for q in md:
                if abs(q[0]-tg[0][0])>0.5 and q[1]<0.2: e+=((0.2-q[1])/0.03)**2
    return e
NAMES=['l','tau','h','fz','zz','fp','zp']; B=[(0.06,0.13),(0,0.025),(0,0.3),(0.8,5),(0,1),(0.6,5),(0,1.5)]
if __name__=='__main__':
    best=None
    for seed in (0,1):
        r=differential_evolution(lambda x: err(dict(zip(NAMES,x))),B,seed=seed,maxiter=150,popsize=15,tol=1e-9)
        print(seed,round(r.fun,1),dict(zip(NAMES,np.round(r.x,4))),flush=True)
        if best is None or r.fun<best.fun: best=r
    p=dict(zip(NAMES,map(float,best.x))); json.dump(p,open('est2_poles.json','w'))
    for K,c,tg in TG: print(K,c,[(round(a,2),round(b,3)) for a,b in modes(p,K,c) if a>0.8],'target',tg)
