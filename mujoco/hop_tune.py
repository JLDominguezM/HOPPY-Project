"""hop_tune.py — mini random search del hop_controller (5 params clave).

Metrica = subida del cuerpo sobre el reposo asentado (cm) + bonus por vuelos sostenidos.
El modelo se construye UNA vez (los 5 params son del controlador, no del XML), asi que
cada eval es solo MjData + 8000 pasos. Guarda el mejor en hop_best.json.

Uso: python3 hop_tune.py [n_iter]
"""
import sys
import json
import numpy as np
import mujoco
import hoppy_urdf
from hop_controller import HopController

RANGES = dict(TAU_HIP=(3.0, 5.0), TAU_KNEE=(3.0, 5.0),
              T_crouch=(0.08, 0.25), T_push=(0.06, 0.20), q4_crouch=(-1.2, -0.7))

_M = mujoco.MjModel.from_xml_string(hoppy_urdf.make_xml(dict(hoppy_urdf.DEFAULTS, fast=True)))
_HIP = mujoco.mj_name2id(_M, mujoco.mjtObj.mjOBJ_BODY, "Link3")


def _flights(grf):
    air = grf < 2.0
    fl, i, n = 0, 0, len(air)
    while i < n:
        if air[i]:
            j = i
            while j < n and air[j]:
                j += 1
            if (j - i) > 40:          # vuelo sostenido > 40 ms
                fl += 1
            i = j
        else:
            i += 1
    return fl


def evaluate(p):
    d = mujoco.MjData(_M)
    h = HopController(p, _M, d)
    h.reset()
    z, g = [], []
    for _ in range(8000):
        rec = h.step()
        mujoco.mj_step(_M, d)
        if np.any(np.isnan(d.qpos)):
            return -999.0, 0.0, 0
        z.append(d.xpos[_HIP][2]); g.append(rec["grf"])
    z, g = np.array(z), np.array(g)
    z_rest = float(z[300:1500].min())          # punto bajo asentado
    rise = (float(z[3000:].max()) - z_rest) * 100.0
    fl = _flights(g)
    return rise + min(fl, 30) * 0.1, rise, fl   # fitness: subida + bonus vuelos


def main(n_iter=100):
    rng = np.random.default_rng(0)
    best = (-1e9, None, 0.0, 0)
    for k in range(n_iter):
        p = dict(hoppy_urdf.DEFAULTS); p["fast"] = True
        for kk, (lo, hi) in RANGES.items():
            p[kk] = float(rng.uniform(lo, hi))
        sc, rise, fl = evaluate(p)
        if sc > best[0]:
            best = (sc, {kk: round(p[kk], 4) for kk in RANGES}, rise, fl)
            print("iter %3d: rise=%.1f cm  vuelos=%d  %s" % (k, rise, fl, best[1]), flush=True)
    json.dump(dict(rise_cm=round(best[2], 2), vuelos=best[3], params=best[1]),
              open("hop_best.json", "w"), indent=2)
    print("=== BEST: rise=%.1f cm  vuelos=%d -> hop_best.json ===" % (best[2], best[3]))
    print(best[1])
    return best


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 100)
