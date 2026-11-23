#!/usr/bin/env python3
"""Phase 11.1: balance model fitted to the kick tests, and LQR gains checked on it.

Model (all linear, small angles):
  * pendulum on the wheel axle:      l·θ'' = g·θ − a,   a = commanded wheel
    acceleration delayed by tau (stepper, IMU filter, loop timing);
  * a lightly damped "dipole" between the body and the IMU reading: a pole
    pair (fp, zp) and a zero pair (fz, zz), DC gain 1. It stands for the
    compliant joints of the prototype; its damping is what handling changes.
  * the firmware controller exactly: u = KP·θm + KD·qm + KV·v + KX·x + kick,
    v and x being the integrals of u, run at 500 Hz.

Usage (from this folder; needs numpy and scipy, numba optional for speed):
  python3 phase11_model.py fit  RUN [RUN ...]   fit the model to each run's kicks
  python3 phase11_model.py check                closed-loop poles for known gain sets
  python3 phase11_model.py steps                LQR gains and the stepping plan
RUN is a runs/ folder name, e.g. 2026-11-23_161333_balance.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.optimize import differential_evolution
from scipy.signal import cont2discrete

G = 9.80665
R2D = 57.29577951308232
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.environ.get("R2W_RUNS", os.path.join(HERE, "..", "..", "..", "runs"))
MODEL_FILE = os.path.join(HERE, "model_fit.json")
CURRENT = np.array([36.7, 4.19, 4.81, 2.94])        # KP KD KV KX


# ---------------------------------------------------------------- data
def kick_ensemble(run: str, pre: float = 0.1, post: float = 1.6):
    """Average response to the BAL_KICK pulses of one run (sign-aligned).
    Returns t, mean [q deg/s, pitch deg, u m/s2, v m/s], number of kicks."""
    import csv
    cols = {"t_s": [], "q_dps": [], "pitch_deg": [], "u_mm_s2": [], "v_left_mm_s": [], "flags": []}
    with open(os.path.join(RUNS, run, "data.csv")) as fh:
        for row in csv.DictReader(fh):
            for k in cols:
                cols[k].append(float(row[k]))
    q, th = np.array(cols["q_dps"]), np.array(cols["pitch_deg"])
    u, v = np.array(cols["u_mm_s2"]) / 1000, np.array(cols["v_left_mm_s"]) / 1000
    kick = (np.array(cols["flags"]).astype(int) & 0x80) > 0
    on = np.flatnonzero(np.diff(kick.astype(int)) == 1) + 1
    n0, n1 = int(pre / 0.004), int(post / 0.004)
    segs = []
    for j, i in enumerate(on):
        if i - n0 < 0 or i + n1 >= len(q) or (j + 1 < len(on) and on[j + 1] < i + n1):
            continue
        s = np.sign(u[i] - u[i - 1]) or 1.0
        b = slice(i - n0, i)
        segs.append([s * (x[i - n0:i + n1] - x[b].mean()) for x in (q, th, u, v)])
    return np.arange(-n0, n1) * 0.004, np.array(segs).mean(0), len(segs)


# ---------------------------------------------------------------- simulation
def _sim_py(l, tau, wz, zz, wp, zp, K, kickU, kickS, T, pre, Tc, sub, amax):
    h = Tc / sub
    nd = int(round(tau / h))
    nsteps = int(round((T + pre) / Tc))
    buf = np.zeros(nd + 1)
    bi = 0
    th = om = w = wd = v = x = 0.0
    use = wz > 0
    c = (wp / wz) ** 2 if use else 1.0
    out = np.zeros((4, nsteps))
    k0 = int(round(pre / Tc))
    kN = k0 + int(round(kickS / Tc))
    for k in range(nsteps):
        if use:
            thm = c * th + w
            qm = c * om + wd
        else:
            thm = th
            qm = om
        u = K[0] * thm + K[1] * qm + K[2] * v + K[3] * x
        if k0 <= k < kN:
            u += kickU
        u = min(amax, max(-amax, u))
        v += u * Tc
        x += v * Tc
        out[0, k] = k * Tc - pre
        out[1, k] = qm * R2D
        out[2, k] = u
        out[3, k] = thm * R2D
        for _ in range(sub):
            buf[bi] = u
            bi = (bi + 1) % (nd + 1)
            a = buf[bi]
            if use:
                wdd = -2 * zp * wp * wd - wp * wp * w + c * ((2 * zz * wz - 2 * zp * wp) * om + (wz * wz - wp * wp) * th)
                wd += wdd * h
                w += wd * h
            om += (G * th - a) / l * h
            th += om * h
    return out


try:
    from numba import njit
    _sim = njit(cache=True)(_sim_py)
except ImportError:          # slower but fine for single runs
    _sim = _sim_py


def omega(p):
    return 2 * np.pi * p.get("fz", 0.0), 2 * np.pi * p.get("fp", 1.0)


def simulate(p, K=CURRENT, T=1.6, kickU=2.0, kickS=0.04, pre=0.1):
    wz, wp = omega(p)
    o = _sim(p["l"], p["tau"], wz, p.get("zz", 0.0), wp, p.get("zp", 0.0),
             np.asarray(K, float), kickU, kickS, T, pre, 0.002, 8, 6.0)
    return o[0], o[1], o[2], o[3]


# ---------------------------------------------------------------- fitting
NAMES = ["l", "tau", "fz", "zz", "fp", "zp"]
BOUNDS = [(0.03, 0.3), (0.0, 0.04), (0.8, 6), (0.0, 1.0), (0.8, 15), (0.0, 1.5)]


def cost(p, t, m, T=1.5):
    ts, q, u, _ = simulate(p, T=T + 0.004)
    sel = t <= T
    rq = np.interp(t[sel], ts, q) - m[0][sel]
    ru = (np.interp(t[sel], ts, u) - m[2][sel]) * 20      # 1 m/s2 weighted like 20 deg/s
    return np.mean(rq ** 2) + np.mean(ru ** 2)


def fit(run):
    t, m, n = kick_ensemble(run)
    r = differential_evolution(lambda x: cost(dict(zip(NAMES, x)), t, m), BOUNDS,
                               seed=0, maxiter=400, popsize=30, tol=1e-8)
    return dict(zip(NAMES, map(float, r.x))), float(np.sqrt(r.fun)), n


# ---------------------------------------------------------------- linear analysis
def closed_loop_poles(p, K, Tc=0.002, sub=40):
    """Poles of the discrete closed loop (ZOH at 500 Hz, delay tau), as s = ln(z)/Tc."""
    l = p["l"]
    wz, wp = omega(p)
    n = 6     # [x, v, th, om, w, wd]
    A = np.zeros((n, n))
    A[0, 1] = 1
    A[2, 3] = 1
    A[3, 2] = G / l
    Bd = np.zeros((n, 1))
    Bd[3, 0] = -1 / l
    C = np.zeros((4, n))
    if wz > 0:
        c = (wp / wz) ** 2
        zz, zp = p["zz"], p["zp"]
        A[4, 5] = 1
        A[5, 4] = -wp * wp
        A[5, 5] = -2 * zp * wp
        A[5, 2] = c * (wz * wz - wp * wp)
        A[5, 3] = c * (2 * zz * wz - 2 * zp * wp)
        C[0, 2], C[0, 4], C[1, 3], C[1, 5] = c, 1, c, 1
    else:
        C[0, 2], C[1, 3] = 1, 1
        A[4, 4] = A[5, 5] = -1e3      # unused states, parked far away
    C[2, 1] = C[3, 0] = 1
    h = Tc / sub
    m = int(round(p["tau"] / h))
    Ad, Bdd, *_ = cont2discrete((A, Bd, np.eye(n), np.zeros((n, 1))), h)
    dim = n + m
    M = np.zeros((dim, dim))
    for j in range(dim):
        z = np.zeros(dim)
        z[j] = 1
        u = float(np.asarray(K) @ (C @ z[:n]))
        xs, line = z[:n].copy(), list(z[n:])
        for _ in range(sub):
            line = [u] + line
            a = line.pop() if m > 0 else u
            xs = Ad @ xs + Bdd[:, 0] * a
        v = z[1] + u * Tc
        xs[0], xs[1] = z[0] + v * Tc, v
        M[:, j] = np.concatenate([xs, line])
    ev = np.linalg.eigvals(M)
    ev = ev[np.abs(ev) > 1e-9]
    s = np.log(ev.astype(complex)) / Tc
    return sorted([z for z in s if z.imag >= 0 and abs(z) < 2 * np.pi * 30 and z.real > -900], key=abs)


def min_damping(p, K):
    s = closed_loop_poles(p, K)
    if any(z.real > 1e-6 for z in s):
        return -1.0
    return min(-z.real / abs(z) for z in s if abs(z) > 1e-3)


def ring(p, K):
    c = [z for z in closed_loop_poles(p, K) if 1.0 < abs(z) / (2 * np.pi) < 4 and z.imag > 1e-6]
    return (abs(c[0]) / (2 * np.pi), -c[0].real / abs(c[0])) if c else (float("nan"), float("nan"))


def lqr_rigid(l, qx, qv, qth, qom, r):
    """Textbook LQR on the rigid pendulum; returns firmware order [KP, KD, KV, KX]."""
    A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, G / l, 0]], float)
    B = np.array([[0], [1], [0], [-1 / l]], float)
    X = solve_continuous_are(A, B, np.diag([qx, qv, qth, qom]), np.array([[r]]))
    K = (B.T @ X)[0] / r
    return np.array([-K[2], -K[3], -K[0], -K[1]])


def load_models():
    fits = json.load(open(MODEL_FILE))
    settled, handled = fits["2026-11-23_161333_balance"]["p"], fits["2026-11-23_172551_balance"]["p"]
    variants = {"settled": settled, "handled": handled,
                "delay +4 ms": {**settled, "tau": settled["tau"] + 0.004},
                "l +20 %": {**settled, "l": settled["l"] * 1.2},
                "l -20 %": {**settled, "l": settled["l"] * 0.8},
                "joints looser": {**settled, "zz": 0.15, "zp": 0.05}}
    return settled, handled, variants


def robust_min(K, variants):
    return min(min_damping(p, K) for p in variants.values())


# ---------------------------------------------------------------- commands
def cmd_fit(runs):
    fits = json.load(open(MODEL_FILE)) if os.path.exists(MODEL_FILE) else {}
    for run in runs:
        p, rms, n = fit(run)
        f, z = ring(p, CURRENT)
        fits[run] = {"p": p, "rms": rms, "kicks": n}
        print(f"{run}: {n} kicks  l {p['l'] * 1000:.1f} mm  delay {p['tau'] * 1000:.1f} ms  "
              f"pole {p['fp']:.2f} Hz ζ {p['zp']:.2f}  zero {p['fz']:.2f} Hz ζ {p['zz']:.2f}  "
              f"fit rms {rms:.1f}  -> predicted ring {f:.2f} Hz ζ {z:.3f}")
    json.dump(fits, open(MODEL_FILE, "w"), indent=1)


def cmd_check():
    settled, handled, variants = load_models()
    sets = {"current": CURRENT, "KD 4.6 (run 170714)": [36.7, 4.6, 4.81, 2.94],
            "redesign (fell, run 125339)": [30.55, 2.19, 4.81, 2.94],
            "KV 6 KX 5 (run 130117)": [36.7, 4.19, 6, 5]}
    for name, K in sets.items():
        fs, zs = ring(settled, K)
        fh, zh = ring(handled, K)
        print(f"{name:28s} settled {fs:.2f} Hz ζ {zs:5.2f} | handled {fh:.2f} Hz ζ {zh:5.2f} | "
              f"worst case ζ {robust_min(K, variants):5.2f}")


def cmd_steps():
    settled, handled, variants = load_models()
    target = np.round(lqr_rigid(0.075, 10, 1, 300, 0.3, 0.1), 1)
    target[1] = round(float(lqr_rigid(0.075, 10, 1, 300, 0.3, 0.1)[1]), 2)
    print("LQR target (rigid model l 75 mm, Q = diag(10, 1, 300, 0.3), R = 0.1):", target)
    for i, a in enumerate(np.linspace(0, 1, 6)):
        K = CURRENT + (target - CURRENT) * a
        K = np.array([round(K[0], 1), round(K[1], 2), round(K[2], 2), round(K[3], 2)])
        fs, zs = ring(settled, K)
        fh, zh = ring(handled, K)
        t, q, u, th = simulate(settled, K=K, T=3)
        late = np.flatnonzero(np.abs(th) > 0.1)
        print(f"step {i}: KP {K[0]:5.1f} KD {K[1]:4.2f} KV {K[2]:5.2f} KX {K[3]:5.2f} | "
              f"ring settled {fs:.2f} Hz ζ {zs:.2f}, handled {fh:.2f} Hz ζ {zh:.2f} | "
              f"worst case ζ {robust_min(K, variants):5.2f} | kick settles {t[late[-1]]:.1f} s, peak u {abs(u).max():.1f}")


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("fit", "check", "steps"):
        print(__doc__)
        sys.exit(1)
    {"fit": lambda: cmd_fit(sys.argv[2:]), "check": cmd_check, "steps": cmd_steps}[sys.argv[1]]()
