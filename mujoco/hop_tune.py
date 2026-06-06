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
              T_crouch=(0.08, 0.25), T_push=(0.06, 0.20), q4_crouch=(-1.0, -0.6))

_M = mujoco.MjModel.from_xml_string(hoppy_urdf.make_xml(dict(hoppy_urdf.DEFAULTS, fast=True)))
_HIP = mujoco.mj_name2id(_M, mujoco.mjtObj.mjOBJ_BODY, "Link3")
_L4 = mujoco.mj_name2id(_M, mujoco.mjtObj.mjOBJ_BODY, "Link4")


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
    z, g, l4, knee = [], [], [], []
    for _ in range(8000):
        rec = h.step()
        mujoco.mj_step(_M, d)
        if np.any(np.isnan(d.qpos)):
            return -999.0, 0.0, 0
        z.append(d.xpos[_HIP][2]); g.append(rec["grf"])
        l4.append(d.xpos[_L4][2]); knee.append(float(d.qpos[3]))
    z, g, l4, knee = map(np.array, (z, g, l4, knee))
    z_rest = float(z[300:1500].min())          # punto bajo asentado
    rise = (float(z[3000:].max()) - z_rest) * 100.0
    fl = _flights(g)
    score = rise + min(fl, 30) * 0.1           # fitness: subida + bonus vuelos
    if float(l4.min()) < -0.01:                # Link4 atraviesa el piso -> castigo proporcional
        score -= 50.0 * abs(float(l4.min()))
    if float(np.mean(knee < -1.25)) > 0.10:    # rodilla pegada al limite -1.3 >10% del tiempo
        score -= 20.0
    return score, rise, fl


def _save(out, best):
    json.dump(dict(rise_cm=round(best[2], 2), vuelos=best[3], score=round(best[0], 2),
                   params=best[1]), open(out, "w"), indent=2)


def main(n_iter=100, seed=0, out="hop_best.json"):
    rng = np.random.default_rng(seed)
    best = (-1e9, None, 0.0, 0)
    for k in range(n_iter):
        p = dict(hoppy_urdf.DEFAULTS); p["fast"] = True
        for kk, (lo, hi) in RANGES.items():
            p[kk] = float(rng.uniform(lo, hi))
        sc, rise, fl = evaluate(p)
        if sc > best[0]:
            best = (sc, {kk: round(p[kk], 4) for kk in RANGES}, rise, fl)
            _save(out, best)
            print("iter %3d: rise=%.1f cm  vuelos=%d  score=%.1f  %s" % (k, rise, fl, sc, best[1]), flush=True)
        elif k % 10 == 0:
            print("iter %3d: best rise=%.1f cm (score=%.1f)" % (k, best[2], best[0]), flush=True)
    _save(out, best)
    print("=== BEST: rise=%.1f cm  vuelos=%d -> %s ===" % (best[2], best[3], out), flush=True)
    print(best[1], flush=True)
    return best


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=100)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="hop_best.json")
    a = ap.parse_args()
    main(n_iter=a.iters, seed=a.seed, out=a.out)
