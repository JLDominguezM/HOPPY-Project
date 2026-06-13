"""Final controller run of the abstract model + plots.

Uses the shared controller (controller.Hoppy), runs the simulation, prints
honest metrics and generates figures/results.png with 6 panels: body height,
foot force, joint torques, voltage (12 V saturation), real vs filtered velocity
and the limit cycle.

Run:  python3 control.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controller import simulate
from tune_eval import DEFAULTS

PARAMS = dict(DEFAULTS)   # faithful config + documented MuJoCo tuning in tune_eval.py


def figures(L):
    import os
    os.makedirs("figures", exist_ok=True)
    t = L["t"]
    ph = L["phase"]
    # body apices
    apex = []; cur = -9.9
    for p, z in zip(ph, L["body_z"]):
        if p == 0:
            cur = max(cur, z)
        elif cur > 0:
            apex.append(cur); cur = -9.9
    apex = np.array(apex)
    real = int(np.sum(L["foot_z"] > 0.02))  # informational only
    fig, ax = plt.subplots(3, 2, figsize=(12, 9))
    # 1) body height (sustained hopping)
    ax[0, 0].plot(t, L["body_z"], "b")
    ax[0, 0].grid(True)
    ax[0, 0].set_title(f"Body height ({len(apex)} hops, apex std={np.std(apex[3:])*1000:.0f} mm)")
    ax[0, 0].set_xlabel("time [s]"); ax[0, 0].set_ylabel("z hip [m]")
    # 2) foot reaction force (desired Bezier vs real contact)
    ax[0, 1].plot(t, L["Fz_des"], "r", lw=0.8, label="Fz desired (Bezier)")
    ax[0, 1].plot(t, L["grf"], "k", lw=0.8, label="real GRF (contact)")
    ax[0, 1].grid(True); ax[0, 1].legend(); ax[0, 1].set_title("Foot reaction force")
    ax[0, 1].set_xlabel("time [s]"); ax[0, 1].set_ylabel("N"); ax[0, 1].set_xlim(4, 6)
    # 3) joint torques
    ax[1, 0].plot(t, L["tau3"], "b", lw=0.7, label="hip")
    ax[1, 0].plot(t, L["tau4"], "r", lw=0.7, label="knee")
    ax[1, 0].grid(True); ax[1, 0].legend(); ax[1, 0].set_title("Joint torques")
    ax[1, 0].set_xlabel("time [s]"); ax[1, 0].set_ylabel("Nm"); ax[1, 0].set_xlim(4, 6)
    # 4) voltage with saturation
    ax[1, 1].plot(t, L["V3"], "b", lw=0.6, label="V hip")
    ax[1, 1].plot(t, L["V4"], "r", lw=0.6, label="V knee")
    ax[1, 1].axhline(12, color="k", ls="--", lw=0.8); ax[1, 1].axhline(-12, color="k", ls="--", lw=0.8)
    ax[1, 1].grid(True); ax[1, 1].legend(); ax[1, 1].set_title("Voltage (limit +/-12 V)")
    ax[1, 1].set_xlabel("time [s]"); ax[1, 1].set_ylabel("V"); ax[1, 1].set_xlim(4, 6)
    # 5) real vs filtered velocity
    ax[2, 0].plot(t, L["qd3_real"], "c", lw=0.6, label="real (qvel)")
    ax[2, 0].plot(t, L["qd3_filt"], "b", lw=0.9, label="filtered (encoder)")
    ax[2, 0].grid(True); ax[2, 0].legend(); ax[2, 0].set_title("Hip velocity: real vs filtered derivative")
    ax[2, 0].set_xlabel("time [s]"); ax[2, 0].set_ylabel("rad/s"); ax[2, 0].set_xlim(4, 5)
    # 6) limit cycle (body phase portrait)
    bz = L["body_z"]; bzv = np.gradient(bz, t)
    half = len(t) // 2
    ax[2, 1].plot(bz[half:], bzv[half:], "b", lw=0.5)
    ax[2, 1].grid(True); ax[2, 1].set_title("Limit cycle (body phase portrait)")
    ax[2, 1].set_xlabel("z [m]"); ax[2, 1].set_ylabel("dz/dt [m/s]")
    fig.tight_layout(); fig.savefig("figures/results.png", dpi=140)
    print("figures/results.png saved")


if __name__ == "__main__":
    L = simulate(PARAMS, t_total=8.0)
    ph = L["phase"]
    apex = []; cur = -9.9
    for p, z in zip(ph, L["body_z"]):
        if p == 0:
            cur = max(cur, z)
        elif cur > 0:
            apex.append(cur); cur = -9.9
    apex = np.array(apex)
    ss = L["t"] > 4.0
    print(f"Hops: {len(apex)}  apex std={np.std(apex[3:])*1000:.0f} mm (limit cycle)")
    print(f"Hip height (regime): [{L['body_z'][ss].min():.3f}, {L['body_z'][ss].max():.3f}] m")
    print(f"Max voltage |V|={max(np.abs(L['V3']).max(), np.abs(L['V4']).max()):.2f} V (limit 12)")
    print(f"Max current |i|={max(np.abs(L['i3']).max(), np.abs(L['i4']).max()):.2f} A (limit 30)")
    print(f"Peak GRF={L['grf'][ss].max():.0f} N")
    figures(L)
