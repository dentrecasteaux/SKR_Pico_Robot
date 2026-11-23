import json,numpy as np
from scipy.optimize import differential_evolution
import poles_ct as pc
pe=json.load(open('model_fit.json'))['2026-11-23_203054_balance']['p']
S={1:[44.1,4.28,5.85,4.71],2:[51.4,4.37,6.89,6.48],3:[58.8,4.45,7.92,8.26],4:[66.1,4.54,8.96,10.03],5:[73.5,4.63,10,11.8]}
T={1:[(2.58,0.20)],2:[(2.88,0.32)],3:[(4.35,0.03),(3.8,0.35)],4:[(4.80,0.02)],5:[(5.15,0.015)]}
def err(p,extra):
    e=0
    for k,K in S.items():
        md=pc.modes(p,K,extra)
        if not md: return 1e6
        for f,z in T[k]:
            j=min(md,key=lambda m:abs(m[0]-f)); e+=((j[0]-f)/0.1)**2+((j[1]-z)/0.03)**2
        if any(m[1]<-0.02 for m in md): e+=100
    return e
def fit(extra,names,bounds,free_base=()):
    nm=list(names)+list(free_base)
    bb=list(bounds)+[{'l':(0.03,0.08),'fz':(1,3),'zz':(0,1),'fp':(0.6,2.5),'zp':(0,1)}[k] for k in free_base]
    f=lambda x: err({**pe,**dict(zip(nm,x))},extra)
    r=differential_evolution(f,bb,seed=1,maxiter=300,popsize=25,tol=1e-9)
    p={**pe,**dict(zip(nm,map(float,r.x)))}
    return p,r.fun
if __name__=='__main__':
    out={}
    for lab,extra,names,b,fb in [('delay',None,['dtau'],[(0,0.03)],()),
                              ('lag','lag',['T2','dtau'],[(0,0.05),(0,0.03)],()),
                              ('drive','drive',['f2','z2','dtau'],[(3,20),(0,1),(0,0.03)],()),
                              ('drive+base','drive',['f2','z2','dtau'],[(3,20),(0,1),(0,0.03)],('l','zz','zp'))]:
        p,e=fit(extra,names,b,fb); out[lab]=(p,e,extra)
        print(lab,'err',round(e,2),{k:round(v,4) for k,v in p.items() if k in names or k in fb})
        for k,K in S.items(): print('   s%d'%k,[(round(a,2),round(z,3)) for a,z in pc.modes(p,K,extra) if a>1],' target',T[k])
    json.dump(out,open('polefit.json','w'),indent=1)
