"""Integrator comparison: why the models use implicitfast instead of RK4.

A common recommendation is integrator="RK4" (+ Newton solver, iterations=50,
tolerance=1e-8). These models use integrator="implicitfast". This script runs
the SAME simulation (real URDF + hybrid controller, hoppy_urdf.FORWARD) with
both configurations and compares:

  - body and foot trajectory (is the hop the same?)
  - stability (NaN / divergence)
  - compute cost (wall-clock time)

Physical reason: implicitfast integrates the velocity-dependent terms IMPLICITLY
(joint damping and armature), which are exactly the effects this model adds, so
it stays stable with hard contact at 1 kHz. RK4 is explicit: with stiff contact
(small solref) and high damping it can need smaller steps to avoid ringing. The
MuJoCo documentation recommends implicit integrators for models with contacts
and advises against RK4 there.

Outputs: figuras/integrators.png + metrics on stdout.
Run:     python3 comparacion_integradores.py
"""
import re
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import controller
import hoppy_urdf as H

T_TOTAL = 6.0
ORIG_MAKE_XML = H.make_xml


def _xml_rk4(xml):
    # the exact RK4 configuration often recommended for reference
    return xml.replace(
        '<option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>',
        '<option timestep="0.001" integrator="RK4" gravity="0 0 -9.81" '
        'solver="Newton" iterations="50" tolerance="1e-8"/>')


def simula(nombre, xml_patch=None):
    p = dict(H.FORWARD, fast=True)
    if xml_patch is not None:
        H.make_xml = lambda pp: xml_patch(ORIG_MAKE_XML(pp))
    try:
        h = controller.Hoppy(p, mdl=H)
    finally:
        H.make_xml = ORIG_MAKE_XML
    keys, L = None, {}
    nan = False
    t0 = time.perf_counter()
    for _ in range(int(T_TOTAL / h.DT)):
        rec = h.step()
        if keys is None:
            keys = list(rec)
            L = {k: [] for k in keys}
        for k in keys:
            L[k].append(rec[k])
        if np.any(np.isnan(h.d.qpos)):
            nan = True
            break
    wall = time.perf_counter() - t0
    out = {k: np.array(v) for k, v in L.items()}
    out["nombre"], out["nan"], out["wall"] = nombre, nan, wall
    return out


def main():
    runs = [simula("implicitfast (ours)"),
            simula("RK4 + Newton/50/1e-8", _xml_rk4)]

    print(f"\n{'config':<24}{'stable':>8}{'foot clear':>13}{'hops':>8}{'wall':>9}")
    print("-" * 62)
    for r in runs:
        m = r["t"] >= 1.0
        clear = max(0.0, r["foot_z"][m].max() - 0.016) if m.any() else 0.0
        ph = r["phase"][m]
        saltos = int(((ph[1:] - ph[:-1]) == -1).sum())
        print(f"{r['nombre']:<24}{'NaN!' if r['nan'] else 'yes':>8}"
              f"{100*clear:>11.1f}cm{saltos:>8d}{r['wall']:>8.1f}s")

    fig, ax = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    for r, c in zip(runs, ("k", "tab:red")):
        ax[0].plot(r["t"], 100 * r["body_z"], c, lw=1.4, label=r["nombre"])
        ax[1].plot(r["t"], 100 * r["foot_z"], c, lw=1.4, label=r["nombre"])
    ax[0].set_ylabel("body height (cm)")
    ax[1].set_ylabel("foot height (cm)")
    ax[1].set_xlabel("t (s)")
    for a in ax:
        a.grid(True, alpha=0.4)
        a.legend()
    fig.suptitle("implicitfast vs RK4: same sim, same controller")
    fig.tight_layout()
    fig.savefig("figuras/integrators.png", dpi=150, bbox_inches="tight")
    print("\nsaved figuras/integrators.png")


if __name__ == "__main__":
    main()
