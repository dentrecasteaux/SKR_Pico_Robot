import numpy as np
G=9.80665; TAU_F=0.5
def cl_matrix(p,K,comp,Tc=0.002,sub=8):
    hh=Tc/sub; nd=int(round(p['tau']/hh)); l=p['l']; h=p['h']
    wz=2*np.pi*p['fz']; wp=2*np.pi*p['fp']; zz=p['zz']; zp=p['zp']; c=(wp/wz)**2
    n=10+nd+1
    def step(z):
        th,om,v,x,est,w,wd,a,al,u=z[:10]; buf=list(z[10:])
        thg=c*th+w; qm=c*om+wd
        accp=thg-(a+h*al-comp*u)/G; kk=Tc/(TAU_F+Tc)
        est=est+kk*(accp-(thg+est)); thm=thg+est
        u=K[0]*thm+K[1]*qm+K[2]*v+K[3]*x
        v=v+u*Tc; x=x+v*Tc
        for s in range(sub):
            buf=buf[1:]+[u]; a=buf[0]; al=(G*th-a)/l
            wdd=-2*zp*wp*wd-wp*wp*w+c*((2*zz*wz-2*zp*wp)*om+(wz*wz-wp*wp)*th)
            wd=wd+wdd*hh; w=w+wd*hh
            om=om+al*hh; th=th+om*hh
        return np.array([th,om,v,x,est,w,wd,a,al,u]+buf)
    M=np.zeros((n,n))
    for j in range(n):
        z=np.zeros(n); z[j]=1; M[:,j]=step(z)
    return M
def modes(p,K,comp):
    ev=np.linalg.eigvals(cl_matrix(p,K,comp)); ev=ev[np.abs(ev)>1e-9]
    s=np.log(ev.astype(complex))/0.002
    if any(z.real>1e-3 for z in s): unstable=True
    s=[z for z in s if z.imag>1e-6 and abs(z)<2*np.pi*20]
    return sorted([(abs(z)/(2*np.pi),-z.real/abs(z)) for z in s])
