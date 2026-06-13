"""Full signal analysis of the HOPPY digital twin.

Generates figures/signals.png with 8 subplots (4x2) covering the main signal
families (joint and Cartesian positions and velocities, GRF, torques and FSM):
  [0,0] joint positions  q3 (hip), q4 (knee)
  [0,1] Cartesian foot position  x, y, z  (data.site_xpos[foot_site])
  [1,0] ESTIMATED filtered joint velocity (Hoppy.qd_filt, lambda=10) vs raw
  [1,1] estimated CARTESIAN foot velocity = J(q).qd_filt (hip frame, the same
        encoder-emulation chain the controller uses) vs raw
  [2,0] vertical contact force (GRF) + reference at 0
  [2,1] control torques tau3, tau4 + lines at +/- tau_max (kT*N*IMAX)
  [3,0] FSM state  0=FLIGHT / 1=STANCE  (blue background=flight, green=stance)

Data (checked against the model, not assumed):
  - applied tau = rec["tau3"]/rec["tau4"] from control_step (tau = kT*N*i)
  - GRF = rec["grf"] (mj_contactForce normal; the foot_touch sensor reads 0
    because the site zone is smaller than the foot sphere)
  - FSM = rec["phase"] (1 stance, 0 flight)
  - filtered vel = Hoppy.qd_filt[0/1]; foot = h.d.site_xpos[h.fs]
The sim runs with vis=False (fast); visual geometry does not affect the dynamics.

Run:  python3 plot_signals.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")            # headless backend (only saves PNG)
import matplotlib.pyplot as plt

import twin
from controller import Hoppy, LAMBDA

HERE = os.path.dirname(os.path.abspath(__file__))
FIGDIR = os.path.join(HERE, "figures")
T_TOTAL = 5.0                    # seconds to simulate
FZ_CONTACT = 2.0                 # GRF threshold for "in stance" (N)


def run():
    """Simulate the twin (vis=False) and log every signal per step, aligned to
    the control instant."""
    h = Hoppy(dict(twin.DEFAULTS), mdl=twin)
    DT = h.DT
    hip_v, knee_v = h.vadr["theta3"], h.vadr["theta4"]
    n = int(T_TOTAL / DT)
    keys = ("t", "q3", "q4", "foot_x", "foot_y", "foot_z",
            "raw_hip", "raw_knee", "filt_hip", "filt_knee",
            "vest_x", "vest_z", "vraw_x", "vraw_z",
            "grf", "tau3", "tau4", "phase", "contact")
    log = {k: [] for k in keys}
    for _ in range(n):
        # state at the control instant (what control_step will read)
        raw_hip = h.d.qvel[hip_v]
        raw_knee = h.d.qvel[knee_v]
        foot = h.d.site_xpos[h.fs].copy()
        rec = h.step()                       # control_step (updates qd_filt) + mj_step
        # Cartesian foot velocity (hip frame): estimated = J.qd_filt
        # (the same encoder->filter->Jacobian chain as the controller) vs raw = J.qvel
        R2, _hip = h._hip_frame()
        Jhip = h._foot_jac_hip(R2)
        v_est = Jhip @ h.qd_filt
        v_raw = Jhip @ np.array([h.d.qvel[hip_v], h.d.qvel[knee_v]])
        log["t"].append(rec["t"])
        log["q3"].append(rec["q3"]);            log["q4"].append(rec["q4"])
        log["foot_x"].append(foot[0]); log["foot_y"].append(foot[1]); log["foot_z"].append(foot[2])
        log["raw_hip"].append(raw_hip);         log["raw_knee"].append(raw_knee)
        log["filt_hip"].append(h.qd_filt[0]);   log["filt_knee"].append(h.qd_filt[1])
        log["vest_x"].append(v_est[0]);         log["vest_z"].append(v_est[1])
        log["vraw_x"].append(v_raw[0]);         log["vraw_z"].append(v_raw[1])
        log["grf"].append(rec["grf"]);          log["phase"].append(rec["phase"])
        log["tau3"].append(rec["tau3"]);        log["tau4"].append(rec["tau4"])
        log["contact"].append(bool(rec["grf"] > FZ_CONTACT))
        if np.any(np.isnan(h.d.qpos)):
            print("WARNING: NaN detected, simulation truncated")
            break
    return {k: np.asarray(v) for k, v in log.items()}, DT


def _segments(mask):
    """Index ranges [(i0, i1), ...] of contiguous True runs in mask."""
    segs, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            segs.append((i, j)); i = j
        else:
            i += 1
    return segs


def main():
    D, DT = run()
    t = D["t"]
    segs = _segments(D["contact"])
    tmaxH = twin.kT * twin.NH * twin.IMAX     # hip max torque
    tmaxK = twin.kT * twin.NK * twin.IMAX     # knee max torque

    fig, ax = plt.subplots(4, 2, figsize=(13, 14))
    fig.suptitle("HOPPY digital twin: full signal analysis",
                 fontsize=14, fontweight="bold")

    def shade(a):                              # green band where GRF > 2 N (stance)
        for (i0, i1) in segs:
            a.axvspan(t[i0], t[min(i1, len(t) - 1)], color="green", alpha=0.15, lw=0)

    # [0,0] joint positions
    a = ax[0, 0]; shade(a)
    a.plot(t, D["q3"], color="C0", label="q3 hip")
    a.plot(t, D["q4"], color="C3", label="q4 knee")
    a.set_ylabel("joint angle (rad)"); a.set_title("Joint positions")
    a.legend(loc="best"); a.grid(alpha=0.3)

    # [0,1] Cartesian foot position (world)
    a = ax[0, 1]; shade(a)
    a.plot(t, D["foot_x"], color="C0", label="foot x")
    a.plot(t, D["foot_y"], color="C1", label="foot y")
    a.plot(t, D["foot_z"], color="C2", label="foot z (height)")
    a.set_ylabel("foot position (m)"); a.set_title("Cartesian foot position")
    a.legend(loc="best", fontsize=8); a.grid(alpha=0.3)

    # [1,0] filtered vs raw velocity
    a = ax[1, 0]; shade(a)
    a.plot(t, D["raw_hip"], color="gray", alpha=0.3, ls="--", label="raw qvel hip")
    a.plot(t, D["raw_knee"], color="dimgray", alpha=0.3, ls="--", label="raw qvel knee")
    a.plot(t, D["filt_hip"], color="C0", label="filtered hip")
    a.plot(t, D["filt_knee"], color="C1", label="filtered knee")
    a.set_ylabel("joint vel. (rad/s)")
    a.set_title("Estimated filtered velocity (lambda=10) vs raw")
    a.legend(loc="best", fontsize=7); a.grid(alpha=0.3)

    # [1,1] Cartesian foot velocity: estimated (J.qd_filt) vs raw (J.qvel)
    a = ax[1, 1]; shade(a)
    a.plot(t, D["vraw_x"], color="gray", alpha=0.3, ls="--", label="raw tangential")
    a.plot(t, D["vraw_z"], color="dimgray", alpha=0.3, ls="--", label="raw vertical")
    a.plot(t, D["vest_x"], color="C0", label="estimated tangential (J.qd_filt)")
    a.plot(t, D["vest_z"], color="C2", label="estimated vertical (J.qd_filt)")
    a.set_ylabel("foot Cartesian vel. (m/s)")
    a.set_title("Estimated Cartesian foot velocity (hip frame)")
    a.legend(loc="best", fontsize=7); a.grid(alpha=0.3)

    # [2,0] GRF
    a = ax[2, 0]; shade(a)
    a.plot(t, D["grf"], color="C2", label="vertical GRF")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("GRF (N)"); a.set_title("Contact force (GRF)")
    a.legend(loc="best"); a.grid(alpha=0.3)

    # [2,1] control torques + limits
    a = ax[2, 1]; shade(a)
    a.plot(t, D["tau3"], color="C0", label="tau hip")
    a.plot(t, D["tau4"], color="C3", label="tau knee")
    a.axhline(tmaxH, color="C0", ls=":", lw=1.0); a.axhline(-tmaxH, color="C0", ls=":", lw=1.0)
    a.axhline(tmaxK, color="C3", ls=":", lw=1.0); a.axhline(-tmaxK, color="C3", ls=":", lw=1.0)
    a.set_xlabel("time (s)"); a.set_ylabel("torque (N.m)")
    a.set_title("Control torques (+/- tau_max = kT.N.Imax dotted)")
    a.legend(loc="best", fontsize=8); a.grid(alpha=0.3)

    # [3,0] FSM state (blue background=flight, green=stance)
    a = ax[3, 0]
    for (i0, i1) in _segments(D["phase"] == 1):
        a.axvspan(t[i0], t[min(i1, len(t) - 1)], color="green", alpha=0.15, lw=0)
    for (i0, i1) in _segments(D["phase"] == 0):
        a.axvspan(t[i0], t[min(i1, len(t) - 1)], color="skyblue", alpha=0.12, lw=0)
    a.plot(t, D["phase"], color="k", drawstyle="steps-post", label="FSM")
    a.set_ylim(-0.2, 1.2); a.set_yticks([0, 1]); a.set_yticklabels(["FLIGHT", "STANCE"])
    a.set_xlabel("time (s)"); a.set_ylabel("FSM state")
    a.set_title("Finite state machine (FSM)")
    a.legend(loc="best", fontsize=8); a.grid(alpha=0.3)
    ax[3, 1].axis("off")

    os.makedirs(FIGDIR, exist_ok=True)
    out = os.path.join(FIGDIR, "signals.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("figure saved at:", out)

    # --- console metrics ---
    hops = sum(1 for (i0, i1) in segs if (i1 - i0) * DT >= 0.01)
    print("peak GRF = %.0f N" % D["grf"].max())
    print("tau_max hip = %.2f N.m  |  tau_max knee = %.2f N.m" % (tmaxH, tmaxK))
    print("hops detected = %d  (stance phases > 10 ms, GRF > %.0f N)" % (hops, FZ_CONTACT))


if __name__ == "__main__":
    main()
