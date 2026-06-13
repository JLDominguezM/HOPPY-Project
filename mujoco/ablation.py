"""Mechanical-model ablation.

Uses comparative simulations to show how each physical effect in the model
(armature, equivalent actuator damping, parallel knee spring and torque
saturation) affects the hop of the real URDF model with the paper's hybrid
controller (hoppy_urdf.FORWARD + controller.py).

Each variant turns off ONE effect and leaves the rest intact:
  full        : complete model (baseline)
  no_armature : armature=0 at hip and knee (no reflected inertia N^2*Ir)
  no_damping  : damping=0 at hip and knee (no actuator losses; the gantry
                damping j_damp is kept, it belongs to the structure)
  no_spring   : knee_stiff=0 (no parallel knee spring)
  no_sat      : VMAX/IMAX -> infinity (ideal actuator with no physical limits)

armature and damping are injected as constants in the XML (not as params),
so they are ablated by patching the XML that make_xml produces (monkeypatch).
Saturation lives in the controller (V<=12, i<=9.2), so it is ablated on the
Hoppy instance.

Outputs: figures/ablation.png + a metrics table on stdout.
Run:     python3 ablation.py
"""
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import controller
import hoppy_urdf as H

T_TOTAL = 6.0
ORIG_MAKE_XML = H.make_xml


def _xml_no_armature(xml):
    return re.sub(r'armature="[^"]*"', 'armature="0"', xml)


def _xml_no_damping(xml):
    # only theta3/theta4 (actuators); theta1/theta2 damping belongs to the gantry
    def zero(m):
        return re.sub(r'damping="[^"]*"', 'damping="0"', m.group(0))
    return re.sub(r'<joint name="theta[34]"[^>]*>', zero, xml)


def simula(nombre, params=None, xml_patch=None, sin_saturacion=False):
    p = dict(H.FORWARD, fast=True)
    if params:
        p.update(params)
    if xml_patch is not None:
        H.make_xml = lambda pp: xml_patch(ORIG_MAKE_XML(pp))
    try:
        h = controller.Hoppy(p, mdl=H)
    finally:
        H.make_xml = ORIG_MAKE_XML
    if sin_saturacion:
        h.VMAX, h.IMAX = 1e9, 1e9
    keys, L = None, {}
    for _ in range(int(T_TOTAL / h.DT)):
        rec = h.step()
        if keys is None:
            keys = list(rec)
            L = {k: [] for k in keys}
        for k in keys:
            L[k].append(rec[k])
        if np.any(np.isnan(h.d.qpos)):
            print(f"  {nombre}: NaN (diverged) at t={rec['t']:.2f}")
            break
    out = {k: np.array(v) for k, v in L.items()}
    out["nombre"] = nombre
    return out


def metricas(r):
    """Metrics over the steady regime (drops the first second)."""
    m = r["t"] >= 1.0
    t, fz, bz = r["t"][m], r["foot_z"][m], r["body_z"][m]
    vuelo = r["phase"][m] == 0
    # real foot clearance (above the foot sphere radius)
    clear = max(0.0, fz.max() - 0.016)
    # body excursion (peak to valley in the regime)
    exc = bz.max() - bz.min()
    # travel around the post
    th1 = r["theta1"][m]
    rate = (th1[-1] - th1[0]) / (t[-1] - t[0])
    # hops = stance->flight edges
    ph = r["phase"][m]
    saltos = int(((ph[1:] - ph[:-1]) == -1).sum())
    return dict(clear_cm=100 * clear, exc_cm=100 * exc, vuelo_pct=100 * vuelo.mean(),
                saltos=saltos, dtheta1=rate,
                tau_max=max(np.abs(r["tau3"][m]).max(), np.abs(r["tau4"][m]).max()),
                i_max=max(np.abs(r["i3"][m]).max(), np.abs(r["i4"][m]).max()))


VARIANTES = [
    ("full",        dict()),
    ("no_armature", dict(xml_patch=_xml_no_armature)),
    ("no_damping",  dict(xml_patch=_xml_no_damping)),
    ("no_spring",   dict(params=dict(knee_stiff=0.0))),
    ("no_sat",      dict(sin_saturacion=True)),
]
COLORES = {"full": "k", "no_armature": "tab:blue", "no_damping": "tab:orange",
           "no_spring": "tab:green", "no_sat": "tab:red"}


def main():
    runs, mets = [], []
    for nombre, kw in VARIANTES:
        print(f"simulating {nombre} ...")
        r = simula(nombre, **kw)
        runs.append(r)
        mets.append(metricas(r))

    # ---- table ----
    cab = f"{'variant':<14}{'foot clear':>12}{'excursion':>11}{'flight %':>9}" \
          f"{'hops':>8}{'dtheta1/dt':>11}{'tau_max':>9}{'i_max':>8}"
    print("\n" + cab)
    print("-" * len(cab))
    for r, mt in zip(runs, mets):
        print(f"{r['nombre']:<14}{mt['clear_cm']:>10.1f}cm{mt['exc_cm']:>9.1f}cm"
              f"{mt['vuelo_pct']:>8.0f}%{mt['saltos']:>8d}{mt['dtheta1']:>11.2f}"
              f"{mt['tau_max']:>9.2f}{mt['i_max']:>8.1f}")

    # ---- figure ----
    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    for r in runs:
        c = COLORES[r["nombre"]]
        lw = 2.2 if r["nombre"] == "full" else 1.3
        ax[0, 0].plot(r["t"], 100 * r["body_z"], c, lw=lw, label=r["nombre"])
        ax[0, 1].plot(r["t"], 100 * r["foot_z"], c, lw=lw, label=r["nombre"])
    ax[0, 0].set_ylabel("body height (cm)")
    ax[0, 0].set_title("Body: ablation changes the hop cycle")
    ax[0, 1].set_ylabel("foot height (cm)")
    ax[0, 1].set_title("Foot: real clearance (flight)")
    for a in ax[0]:
        a.set_xlabel("t (s)")
        a.grid(True, alpha=0.4)
        a.legend(fontsize=8, ncol=2)
        a.set_xlim(1.0, T_TOTAL)

    nombres = [r["nombre"] for r in runs]
    x = np.arange(len(nombres))
    cs = [COLORES[n] for n in nombres]
    ax[1, 0].bar(x, [mt["clear_cm"] for mt in mets], color=cs)
    ax[1, 0].set_xticks(x, nombres, rotation=15)
    ax[1, 0].set_ylabel("foot clearance (cm)")
    ax[1, 0].set_title("Hop height per variant")
    ax[1, 0].grid(True, axis="y", alpha=0.4)
    ax[1, 1].bar(x - 0.2, [mt["tau_max"] for mt in mets], 0.4, color=cs, label="|tau| max (N.m)")
    ax[1, 1].bar(x + 0.2, [mt["i_max"] for mt in mets], 0.4, color=cs, alpha=0.45,
                 label="|i| max (A)")
    ax[1, 1].axhline(H.kT * H.NK * H.IMAX,
                     color="r", ls="--", lw=1, label="physical tau max (kT.N.i_max)")
    ax[1, 1].set_xticks(x, nombres, rotation=15)
    ax[1, 1].set_yscale("log")
    ax[1, 1].set_title("Actuator effort (log): without saturation it blows up")
    ax[1, 1].grid(True, axis="y", alpha=0.4)
    ax[1, 1].legend(fontsize=8)

    fig.suptitle("Mechanical-model ablation: effect of armature, damping, "
                 "parallel spring and saturation (real URDF, hybrid controller)", y=0.99)
    fig.tight_layout()
    out = "figures/ablation.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\nsaved {out}")


if __name__ == "__main__":
    main()
