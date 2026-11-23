import numpy as np
G=9.80665; TAU_F=0.5
def cl_matrix(p,K,comp,Tc=0.002,sub=8):
    hh=Tc/sub; nd=int(round(p['tau']/hh))
    wf=2*np.pi*p['ff'] if p.get('ff') else 0.0; zf=p.get('zf',0); kf=p.get('kf',0); l=p['l']; h=p['h']
    # state: th, om, v, x, est, ph, phd, a, al, u_prev, buffer[nd+1] (order as in sim incl. bi rotation -> use list)
    n=10+nd+1
    def step(z):
        th,om,v,x,est,ph,phd,a,al,u=z[:10]; buf=list(z[10:])
        thg=th+ph; Aimu=a+h*al; accp=thg-(Aimu-comp*u)/G; kk=Tc/(TAU_F+Tc)
        est=est+kk*(accp-(thg+est)); thm=thg+est; qm=om+phd
        u=K[0]*thm+K[1]*qm+K[2]*v+K[3]*x
        v=v+u*Tc; x=x+v*Tc
        for s in range(sub):
            buf=buf[1:]+[u]      # oldest first; delayed value = buf[0] after push (nd steps ago)
            a=buf[0]; al=(G*th-a)/l
            if wf>0:
                phdd=-wf*wf*ph-2*zf*wf*phd+kf*a; phd=phd+phdd*hh; ph=ph+phd*hh
            om=om+al*hh; th=th+om*hh
        return np.array([th,om,v,x,est,ph,phd,a,al,u]+buf)
    M=np.zeros((n,n))
    for j in range(n):
        z=np.zeros(n); z[j]=1; M[:,j]=step(z)
    return M
def modes(p,K,comp):
    ev=np.linalg.eigvals(cl_matrix(p,K,comp)); ev=ev[np.abs(ev)>1e-9]
    s=np.log(ev.astype(complex))/0.002
    s=[z for z in s if z.imag>1e-6 and abs(z)<2*np.pi*20]
    return sorted([(abs(z)/(2*np.pi),-z.real/abs(z)) for z in s])
