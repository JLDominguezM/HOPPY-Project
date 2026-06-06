"""urdf_tune.py — busqueda aleatoria de la marcha para el HOPPY URDF real (hoppy_urdf).

El gait del twin NO transfiere (masas/inercias reales del CAD, 3.43 kg); aqui se
re-afina desde cero. Fitness por prioridad:
  1) subida del cuerpo (z_max - z0)  [primario; >0.02 m = "salta"]
  2) saltos sostenidos (nhop)        [secundario, solo si ya sube]
  3) estabilidad del apex (apex_std) [terciario]
  - penaliza NaN (-999) y pasar >80% en apoyo (no despega).

Random search SECUENCIAL: multiprocessing + MuJoCo da conflictos de contexto, y cada
eval es corta, asi que secuencial es lo robusto (ver RESTRICCIONES). Guarda incremental
urdf_best.json (top-10) y urdf_best_candidate.json si z>4cm. Interrumpible (Ctrl+C /
SIGTERM) guardando lo mejor. Timeout 30 s por evaluacion.

Uso:
  python3 urdf_tune.py --iters 500 --seed 42 --out urdf_best.json
  import urdf_tune; best = urdf_tune.run(n_iter=3, seed=42)
"""
import os
import sys
import json
import time
import signal
import argparse
import numpy as np

from controller import simulate
from verify import analyze
import hoppy_urdf

HERE = os.path.dirname(os.path.abspath(__file__))

RANGES = dict(
    kp_sw=(50, 400), kd_sw=(0.5, 8.0), krh=(0.0, 3.0), p_toe_z=(-0.35, -0.15),
    fz_scale=(0.5, 6.0), fx_scale=(0.0, 2.0), Tst=(0.05, 0.25), knee_stiff=(0.0, 0.5),
    blend=(0.01, 0.15), grf_liftoff=(1.0, 15.0), vx_d=(0.0, 1.5),
    q3_ref=(0.3, 0.8), q4_ref=(-1.2, -0.2),
)

T_SIM = 8.0            # 8 s = 8000 pasos por evaluacion
EVAL_TIMEOUT = 30      # s por evaluacion (red de seguridad)
_STOP = False


class _Timeout(Exception):
    pass


def _on_alarm(sig, frm):
    raise _Timeout()


def _on_stop(sig, frm):
    global _STOP
    _STOP = True
    sys.stderr.write("\n[senal recibida] termino tras esta evaluacion y guardo lo mejor...\n")
    sys.stderr.flush()


def evaluate(params):
    """Simula 8 s y devuelve metricas + score. NaN/timeout/excepcion -> score muy bajo."""
    try:
        signal.signal(signal.SIGALRM, _on_alarm)
        signal.alarm(EVAL_TIMEOUT)
    except (ValueError, AttributeError):
        pass  # SIGALRM solo en hilo principal/unix
    try:
        L = simulate(params, t_total=T_SIM, mdl=hoppy_urdf)
    except _Timeout:
        return dict(score=-999.0, z_cm=0.0, n_saltos=0, reason="timeout")
    except Exception as e:
        return dict(score=-999.0, z_cm=0.0, n_saltos=0, reason="err:%s" % type(e).__name__)
    finally:
        try:
            signal.alarm(0)
        except Exception:
            pass
    if L["_nan"]:
        return dict(score=-999.0, z_cm=0.0, n_saltos=0, reason="nan")
    bz = L["body_z"]
    z0 = float(bz[0])
    rise = float(bz.max()) - z0
    A = analyze(L)
    n_saltos = int(A["nhop"])
    apex_std = float(A["apex_std"])
    frac = float(A["frac"]) / 100.0
    grf_max = float(L["grf"].max())
    score = rise * 1000.0                              # primario: subida
    if rise > 0.02:
        score += min(n_saltos, 12) * 3.0              # secundario: saltos (solo si sube)
        score -= apex_std * 2000.0                     # terciario: estabilidad
    if frac > 0.80:
        score -= 200.0                                 # no despega
    return dict(score=float(score), z_cm=rise * 100.0, n_saltos=n_saltos,
                apex_std_mm=apex_std * 1000.0, grf_max=grf_max, frac_stance=round(frac, 3),
                params={k: float(params[k]) for k in RANGES})


def _sample(rng):
    p = dict(hoppy_urdf.DEFAULTS)
    p["fast"] = True                       # build sin mallas visuales (dinamica identica, carga rapida)
    for k, (lo, hi) in RANGES.items():
        p[k] = float(rng.uniform(lo, hi))
    return p


def _save(path, top):
    best = dict(top[0])
    best["top10"] = top[:10]
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(best, f, indent=2)
    os.replace(tmp, path)


def run(n_iter=200, seed=42, out=None):
    out = out or os.path.join(HERE, "urdf_best.json")
    cand = os.path.join(HERE, "urdf_best_candidate.json")
    rng = np.random.default_rng(seed)
    top = []
    best_score = -1e18
    t0 = time.time()
    for k in range(n_iter):
        if _STOP:
            break
        r = evaluate(_sample(rng))
        top.append(r)
        top.sort(key=lambda d: d["score"], reverse=True)
        top = top[:10]
        if top[0]["score"] > best_score:
            best_score = top[0]["score"]
            _save(out, top)
            if top[0].get("z_cm", 0.0) > 4.0:
                _save(cand, top)
        if k % 10 == 0:
            b = top[0]
            print("iter %d/%d: best_z=%.2f cm, saltos=%d, score=%.1f  (%.0fs)"
                  % (k, n_iter, b["z_cm"], b["n_saltos"], b["score"], time.time() - t0), flush=True)
            _save(out, top)
    _save(out, top)
    b = top[0]
    print("=== FIN (%d evals) === best_z=%.2f cm  saltos=%d  score=%.1f  -> %s"
          % (len(top) and (k + 1), b["z_cm"], b["n_saltos"], b["score"], out), flush=True)
    print("mejores params:", {kk: round(vv, 4) for kk, vv in b.get("params", {}).items()}, flush=True)
    return b


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=500)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default=os.path.join(HERE, "urdf_best.json"))
    a = ap.parse_args()
    signal.signal(signal.SIGINT, _on_stop)
    signal.signal(signal.SIGTERM, _on_stop)
    with open(os.path.join(HERE, "urdf_tune.pid"), "w") as _pf:
        _pf.write(str(os.getpid()))
    print("urdf_tune: %d iters, seed %d, pid %d, %.0fs/eval timeout, out=%s"
          % (a.iters, a.seed, os.getpid(), EVAL_TIMEOUT, a.out), flush=True)
    run(n_iter=a.iters, seed=a.seed, out=a.out)
