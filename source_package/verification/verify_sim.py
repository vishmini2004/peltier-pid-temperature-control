"""Pure-Python mirror of matlab/simulate_loop.m, used to verify the numbers in the report.
Plant:  tau*dx/dt = -x - K*(u_delayed - d_u)        x = T_surface - T_amb   (deg C)
Sensor: T_meas = quantise(T_surface delayed by L)
PID (reverse/cooling acting): e = T_meas - T_ref, u = sat(Kp e + I + D), clamping anti-windup,
        derivative on measurement with first-order filter Tf.
"""
import math, json, sys

P = dict(Tamb=25.0, K=15.0, tau=180.0, L=5.0, dt=0.1, Ts=1.0, q=0.0625,
         umin=0.0, umax=1.0)

def simulate(gains, T_end, ref_fn, dist_fn, P=P, mode="pid", u_ff=None, onoff_band=0.5,
             noise=0.0, seed=1, out_dist_fn=None):
    Kp, Ki, Kd, Tf = gains
    K, tau, L, dt, Ts = P["K"], P["tau"], P["L"], P["dt"], P["Ts"]
    a = math.exp(-dt / tau)
    nd = int(round(L / dt))
    n = int(round(T_end / dt)) + 1
    x = 0.0                                  # deviation from ambient
    buf = [P["Tamb"]] * nd                   # transport delay: exactly nd plant steps (as simulate_loop.m)
    I = 0.0; D = 0.0; y_prev = None; u = 0.0; relay = 0.0
    ctrl_every = int(round(Ts / dt))
    import random; rng = random.Random(seed)
    out = dict(t=[], T=[], Tm=[], r=[], u=[])
    for k in range(n):
        t = k * dt
        T = P["Tamb"] + x + (out_dist_fn(t) if out_dist_fn else 0.0)
        buf.append(T); Td = buf.pop(0)
        Tm = Td + (rng.gauss(0, noise) if noise else 0.0)
        if P["q"]:
            Tm = round(Tm / P["q"]) * P["q"]
        r = ref_fn(t)
        if k % ctrl_every == 0:
            e = Tm - r                         # cooling error: positive -> too warm -> cool more
            if mode == "open":
                u = u_ff(t)
            elif mode == "onoff":
                if e > onoff_band: relay = 1.0
                elif e < -onoff_band: relay = 0.0
                u = relay
            else:
                if y_prev is None: y_prev = Tm
                D = (Tf / (Tf + Ts)) * D + (Kd / (Tf + Ts)) * (Tm - y_prev)
                y_prev = Tm
                Pterm = Kp * e
                I_new = I + Ki * Ts * e
                v = Pterm + I_new + D
                if (v > P["umax"] and e > 0) or (v < P["umin"] and e < 0):
                    I_new = I                   # clamp: stop integrating into saturation
                I = I_new
                v = Pterm + I + D
                u = min(P["umax"], max(P["umin"], v))
        du = dist_fn(t)
        if k % 10 == 0:
            out["t"].append(round(t, 3)); out["T"].append(T); out["Tm"].append(Tm)
            out["r"].append(r); out["u"].append(u)
        # exact ZOH discretisation of first-order plant (input disturbance in duty units)
        x = a * x + (1 - a) * (-K * (u - du))
    return out

def metrics(o, r, t0=0.0, t1=None, T0=25.0):
    t, T = o["t"], o["T"]
    idx = [i for i, ti in enumerate(t) if ti >= t0 and (t1 is None or ti < t1)]
    step = r - T0
    yn = [(T[i] - T0) / step for i in idx]
    tt = [t[i] for i in idx]
    def first(th):
        for ti, y in zip(tt, yn):
            if y >= th: return ti
        return float("nan")
    tr = first(0.9) - first(0.1)
    os_ = max(0.0, (max(yn) - 1) * 100)
    band = 0.02 * abs(step)
    ts = float("nan")
    for ti, i in zip(reversed(tt), reversed(idx)):
        if abs(T[i] - r) > band:
            ts = ti - t0; break
    else:
        ts = 0.0
    if ts == tt[-1] - t0: ts = float("nan")      # never settled
    tail = [T[i] for i in idx if t[i] >= tt[-1] - 60]
    ess = r - sum(tail) / len(tail)
    peak_dev = max(abs(T[i] - r) for i in idx if (T[i] - r) * step > 0)  if any((T[i]-r)*step>0 for i in idx) else 0.0
    return dict(tr=tr, os=os_, ts=ts, ess=ess, undershoot_C=round(min(T[i] for i in idx) - r, 3))

def dist_metrics(o, r, td, t1=None, band=0.2):
    t, T, u = o["t"], o["T"], o["u"]
    idx = [i for i, ti in enumerate(t) if ti >= td and (t1 is None or ti < t1)]
    pk = max(T[i] - r for i in idx)
    rec = 0.0
    for i in reversed(idx):
        if abs(T[i] - r) > band:
            rec = t[i] - td; break
    iae = sum(abs(T[i] - r) for i in idx) * (t[1] - t[0])
    umax = max(u[i] for i in idx)
    return dict(peak=round(pk, 3), recovery=round(rec, 1), IAE=round(iae, 1), umax=round(umax, 3))

def margins(Kp, Ki, Kd, Tf, P=P):
    K, tau, L = P["K"], P["tau"], P["L"]
    def Lj(w):
        s = 1j * w
        C = Kp + Ki / s + Kd * s / (Tf * s + 1)
        G = K * complex(math.cos(-w * L), math.sin(-w * L)) / (tau * s + 1)
        return C * G
    ws = [10 ** (-5 + 6 * i / 200000) for i in range(200001)]
    wc = None; w180 = None; prev = None
    import cmath
    prev_ph = None; prev_mag = None
    ph_unw = 0; last = None
    for w in ws:
        v = Lj(w); mag = abs(v); ph = cmath.phase(v)
        if last is not None:
            d = ph - last
            if d > math.pi: ph_unw -= 2 * math.pi
            elif d < -math.pi: ph_unw += 2 * math.pi
        last = ph
        php = ph + ph_unw
        if prev_mag is not None and wc is None and prev_mag >= 1 > mag:
            wc = w; pm = 180 + math.degrees(php)
        if prev_ph is not None and w180 is None and prev_ph > -math.pi >= php:
            w180 = w; gm = 20 * math.log10(1 / mag)
        prev_mag = mag; prev_ph = php
    return dict(wc=wc, PM=round(pm, 1) if wc else None, w180=w180, GM_dB=round(gm, 1) if w180 else None,
                Ld=round(math.radians(pm) / wc, 1) if wc else None)

# ---------------------------------------------------------------------------
# Full verification run: reproduces every number quoted in the report.
# ---------------------------------------------------------------------------
def tuned_sets(P=P):
    K, tau, L = P["K"], P["tau"], P["L"]
    S = {}
    lam = 90.0; Kc = tau / (K * (lam + L)); Ti = min(tau, 4 * (lam + L)); S["PI_conservative"] = (Kc, Kc / Ti, 0.0, 2.0)
    lam = 20.0; Kc = tau / (K * (lam + L)); Ti = min(tau, 4 * (lam + L)); S["PI_SIMC"] = (Kc, Kc / Ti, 0.0, 2.0)
    S["PID_recommended"] = (Kc, Kc / Ti, Kc * L / 2, 2.0)
    lam = 30.0; Kc = (tau + L / 2) / (K * (lam + L / 2)); Ti = tau + L / 2; Td = tau * L / (2 * tau + L)
    S["PID_IMC_Rivera"] = (Kc, Kc / Ti, Kc * Td, 2.0)
    Kc = 1.2 * tau / (K * L); S["PID_ZN_reaction"] = (Kc, Kc / (2 * L), Kc * 0.5 * L, 2.0)
    return S

def u_sd(o, t0, t1):
    us = [u for t, u in zip(o["t"], o["u"]) if t0 <= t < t1]
    m = sum(us) / len(us); return (sum((v - m) ** 2 for v in us) / len(us)) ** 0.5

def osc_amp(o, t0, t1):
    T = [x for t, x in zip(o["t"], o["T"]) if t0 <= t < t1]; return (max(T) - min(T)) / 2

STD_REF = lambda t: 15.0
STD_DIST = lambda t: 0.2 if t >= 1200 else 0.0

def full_run():
    R = {}
    S = tuned_sets(); R["gains"] = {k: [round(v, 6) for v in g] for k, g in S.items()}
    rec = S["PID_recommended"]
    # 1. open-loop step (u = 1): check tau and K
    o = simulate(rec, 1200, STD_REF, lambda t: 0.0, mode="open", u_ff=lambda t: 1.0)
    T63 = 25 - 0.632 * 15
    t63 = next(t for t, T in zip(o["t"], o["T"]) if T <= T63)
    R["open_loop"] = dict(final=round(o["T"][-1], 2), t63=t63, T63=round(T63, 2), t_to_15=next(t for t, T in zip(o["t"], o["T"]) if T <= 15.0))
    # 2. design comparison (r = 15, load d_u = 0.2 at 1200 s), noise-free and with sensor noise
    R["designs"] = {}
    for k, g in S.items():
        o = simulate(g, 2400, STD_REF, STD_DIST)
        on = simulate(g, 2400, STD_REF, STD_DIST, noise=0.05)
        R["designs"][k] = dict(step=metrics(o, 15, 0, 1200), dist=dist_metrics(o, 15, 1200),
                               margins=margins(*g), u_sd_noise=round(u_sd(on, 800, 1190), 4),
                               osc_amp=round(osc_amp(o, 900, 1200), 3))
    # 3. before vs after control
    ba = {}
    ba["open_loop_feedforward"] = simulate(rec, 2400, STD_REF, STD_DIST, mode="open", u_ff=lambda t: (25 - 15) / 15)
    ba["P_only"] = simulate((rec[0], 0, 0, 2.0), 2400, STD_REF, STD_DIST)
    ba["on_off"] = simulate(rec, 2400, STD_REF, STD_DIST, mode="onoff", onoff_band=0.5)
    ba["PID_recommended"] = simulate(rec, 2400, STD_REF, STD_DIST)
    R["before_after"] = {}
    for k, o in ba.items():
        pre = [T for t, T in zip(o["t"], o["T"]) if 1100 <= t < 1200]
        last = [T for t, T in zip(o["t"], o["T"]) if t > 2250]      # last full on/off cycle (period 150 s)
        two = [T for t, T in zip(o["t"], o["T"]) if t > 2100]       # last two cycles, for peak-to-peak
        R["before_after"][k] = dict(T_before_load=round(sum(pre) / len(pre), 2), T_after_load=round(sum(last) / len(last), 2),
                                    ripple_pp=round(max(two) - min(two), 2), u_sd=round(u_sd(o, 1500, 2400), 3))
    with open("before_after.csv", "w") as f:
        f.write("t_s,series,T_C\n")
        for k, o in ba.items():
            for t, T in zip(o["t"], o["T"]):
                if t % 20 == 0: f.write(f"{int(t)},{k},{T:.3f}\n")
    # 4. gain sweep around the recommended set
    sw = {}
    for lab, g in [("Kp x0.5", (rec[0] * .5, rec[1], rec[2], 2)), ("Kp x2", (rec[0] * 2, rec[1], rec[2], 2)), ("Kp x4", (rec[0] * 4, rec[1], rec[2], 2)),
                   ("Ki x0.25", (rec[0], rec[1] * .25, rec[2], 2)), ("Ki x4", (rec[0], rec[1] * 4, rec[2], 2)), ("Ki x10", (rec[0], rec[1] * 10, rec[2], 2)),
                   ("Kd = 0", (rec[0], rec[1], 0, 2)), ("Kd x4", (rec[0], rec[1], rec[2] * 4, 2)), ("baseline", rec)]:
        o = simulate(g, 2400, STD_REF, STD_DIST); on = simulate(g, 2400, STD_REF, STD_DIST, noise=0.05)
        sw[lab] = dict(gains=[round(v, 5) for v in g], step=metrics(o, 15, 0, 1200), dist=dist_metrics(o, 15, 1200),
                       margins=margins(*g), u_sd_noise=round(u_sd(on, 800, 1190), 4))
    R["sweep"] = sw
    # 5. setpoint feasibility (T_amb 25, K 15, load d_u 0.2)
    fe = {}
    for r in [5, 10, 13, 15, 18, 20, 22]:
        o = simulate(rec, 2400, lambda t, r=r: r, STD_DIST)
        pre = [T for t, T in zip(o["t"], o["T"]) if 1100 <= t < 1200]; post = [T for t, T in zip(o["t"], o["T"]) if 2300 <= t]
        fe[r] = dict(u_ss=round((25 - r) / 15, 3), u_ss_load=round((25 - r) / 15 + 0.2, 3),
                     T_before=round(sum(pre) / len(pre), 2), T_after=round(sum(post) / len(post), 2))
    R["feasibility"] = fe
    # 6. setpoint staircase
    ref = lambda t: 20.0 if t < 800 else (15.0 if t < 1600 else (18.0 if t < 2400 else 22.0))
    o = simulate(rec, 3200, ref, lambda t: 0.0)
    R["staircase"] = {}
    for (a, b, r0, r1) in [(0, 800, 25, 20), (800, 1600, 20, 15), (1600, 2400, 15, 18), (2400, 3200, 18, 22)]:
        R["staircase"][f"{r0}->{r1}"] = metrics(o, r1, a, b, T0=r0)
    with open("staircase.csv", "w") as f:
        f.write("t_s,T_C,ref_C,duty\n")
        for t, T, r, u in zip(o["t"], o["T"], o["r"], o["u"]):
            if t % 10 == 0: f.write(f"{int(t)},{T:.3f},{r:.1f},{u:.3f}\n")
    # 7. robustness of the recommended set
    rb = {}
    for lab, ch in [("nominal", {}), ("K x0.7", {"K": 10.5}), ("K x1.3", {"K": 19.5}), ("tau x0.7", {"tau": 126.0}),
                    ("tau x1.3", {"tau": 234.0}), ("L = 10 s", {"L": 10.0}), ("L = 20 s", {"L": 20.0}), ("K x1.3, L = 20 s", {"K": 19.5, "L": 20.0})]:
        Pm = dict(P); Pm.update(ch)
        r_ = 25 - 0.8 * Pm["K"] if Pm["K"] < 15 else 15.0     # keep the setpoint reachable when K shrinks
        r_ = max(r_, 15.0)
        o = simulate(rec, 2400, lambda t, r_=r_: r_, STD_DIST, P=Pm)
        rb[lab] = dict(ref=r_, margins=margins(*rec, P=Pm), step=metrics(o, r_, 0, 1200), dist=dist_metrics(o, r_, 1200))
    R["robustness"] = rb
    return R

if __name__ == "__main__":
    R = full_run()
    with open("results.json", "w") as f: json.dump(R, f, indent=1, default=str)
    print(json.dumps(R, indent=1, default=str))
