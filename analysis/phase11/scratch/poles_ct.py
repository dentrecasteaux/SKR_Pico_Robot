import numpy as np
from numpy.polynomial import polynomial as Pn
G=9.80665
def pade(tau,n=4):
    # coefficients of e^{-s tau} Pade (n,n), highest power first via numpy poly1d
    from math import factorial
    c=[factorial(2*n-k)*factorial(n)/(factorial(2*n)*factorial(k)*factorial(n-k)) for k in range(n+1)]
    num=np.poly1d([c[k]*(-tau)**k for k in range(n+1)][::-1]); den=np.poly1d([c[k]*tau**k for k in range(n+1)][::-1])
    return num,den
def plant(p,extra=None):
    l=p['l']; wz,wp=2*np.pi*p['fz'],2*np.pi*p['fp']; c=(wp/wz)**2
    N=np.poly1d([-1,0])*c*np.poly1d([1,2*p['zz']*wz,wz*wz]); D=np.poly1d([l,0,-G])*np.poly1d([1,2*p['zp']*wp,wp*wp])
    if extra=='drive':
        w2=2*np.pi*p['f2']; N=N*w2*w2; D=D*np.poly1d([1,2*p['z2']*w2,w2*w2])
    elif extra=='lag':
        D=D*np.poly1d([p['T2'],1])
    nd,dd=pade(p['tau']+0.001+p.get('dtau',0.0))
    return N*nd, D*dd
def poles(p,K,extra=None):
    N,D=plant(p,extra)
    Nc=np.poly1d([K[1],K[0],0]); Dc=np.poly1d([1,-K[2],-K[3]])
    ch=D*Dc-N*Nc
    r=ch.r
    return sorted([z for z in r if z.imag>=0 and abs(z)<2*np.pi*20],key=abs)
def modes(p,K,extra=None):
    return [(abs(z)/(2*np.pi), -z.real/abs(z)) for z in poles(p,K,extra) if z.imag>1e-6]
