"""Re-afina el control del gemelo (twin.py) para SALTO ESTABLE + AVANCE.

Objetivo honesto (no matchea el MATLAB; la dinamica es la del robot del usuario):
empuje real (GRF carga), fase de vuelo real, cadera no se desploma al piso,
ciclo estable (apex consistente), avanza alrededor del poste, V<=12 / i<=9.2.
Busqueda aleatoria alrededor de los defaults.
"""
import numpy as np
from controller import simulate
from verify import analyze
import twin

RANGES = dict(
    spring_scale=(0.03, 0.5),  # resorte SUAVE para salto con vuelo (la rodilla flexiona/empuja)
    knee_damp=(0.0, 0.15),     # amortiguamiento de la rodilla
    kp_sw=(150, 600), kd_sw=(2, 10),
    krh=(0.0, 0.18), p_toe_z=(-0.26, -0.10), Tst=(0.12, 0.30),
    fz_scale=(0.8, 3.2), fx_scale=(0.4, 3.5), blend=(0.006, 0.03),
    grf_liftoff=(1.0, 5.0), j_damp=(0.0, 0.4), solref0=(0.006, 0.02),
    vx_d=(-1.2, 1.2),   # velocidad de avance deseada (Raibert) - el knob que faltaba
    kp_st=(0.0, 0.4), kd_st=(0.0, 0.3),   # autoridad de control en apoyo
    q3_ref=(-0.3, 0.5), q4_ref=(-0.8, 0.1),   # rodilla flexiona (cargar) y extiende (empujar/despegar)
)


def apexes(ph, hip):
    ap, cur = [], -9.0
    for p, z in zip(ph, hip):
        if p == 0:
            cur = max(cur, z)
        elif cur > -9:
            ap.append(cur); cur = -9.0
    return np.array(ap[1:]) if len(ap) > 1 else np.array([])


def score(L):
    """Score HONESTO via verify.analyze: premia AMPLITUD del salto (subir-bajar por
    empuje) y NO aletear (ratio clearance/subida<2), NO la altura absoluta del cuerpo
    (un cuerpo flotando alto con la pierna aleteando es autoengaño)."""
    A = analyze(L)
    if A["nan"]:
        return -1e9, {}
    rise = A["hip_rise"]; ratio = A["ratio"]; grf = A["grf_stance"]
    s = 0.0
    adv = abs(A["th1_net"]); clear = A["clear"]
    s += min(rise, 0.09) * 400                          # premia salto del cuerpo hasta ~9cm
    s += min(clear, 0.05) * 600                         # el PIE debe DESPEGAR (salto real)
    s -= A["apex_std"] * 1000                           # ciclo limite estable
    s -= max(0.0, ratio - 1.8) * 150                   # aletea -> castigo
    s += min(A["nhop"], 16) * 3                         # salto sostenido
    s += min(adv, 4.0) * 14                            # avance alrededor del poste
    if clear < 0.025: s -= 300                          # pie planchado = NO es salto real
    if adv < 0.5: s -= 150                              # debe avanzar
    if rise < 0.04: s -= 200                            # cuerpo debe subir
    if grf < 8:    s -= 300                            # debe CARGAR
    if ratio >= 2.0: s -= 300                          # debe NO aletear
    if rise < 0.03: s -= 300                           # debe subir de verdad
    if (A["q4_hi"] - A["q4_lo"]) < 0.20: s -= 120   # la rodilla debe FLEXIONAR (cargar resorte)
    if A["q4_lo"] < -1.28 or A["q4_hi"] > 0.28: s -= 80   # respeta el rango fisico de la rodilla
    if max(abs(A["th2_lo"]), abs(A["th2_hi"])) > 0.6:  s -= 100   # gantry no colapsa
    if A["Vmax"] > 12.01 or A["imax"] > 9.21:          s -= 100   # limites del motor
    info = dict(nhop=A["nhop"], rise=round(rise, 3), ratio=round(ratio, 2),
                apstd=round(A["apex_std"], 4), grf=round(grf, 1),
                adv=round(A["th1_net"], 2), q4=round(A["q4_lo"], 2),
                th2=round(max(abs(A["th2_lo"]), abs(A["th2_hi"])), 2))
    return s, info


def sample(rng, base, frac=None):
    """frac=None: muestreo global en RANGES. frac=f: local, +/- f del valor base."""
    p = dict(base)
    for k, (lo, hi) in RANGES.items():
        if frac is None:
            p[k] = float(rng.uniform(lo, hi))
        else:
            v = base[k]; span = (hi - lo) * frac
            p[k] = float(np.clip(rng.uniform(v - span, v + span), lo, hi))
    return p


def run_search(n=300, seed=0, t=4.0, base=None, frac=None):
    rng = np.random.default_rng(seed)
    base = base or dict(twin.DEFAULTS)
    best, best_s, best_i = dict(base), -1e18, None
    for k in range(n):
        p = sample(rng, base, frac=frac)
        try:
            L = simulate(p, t_total=t, mdl=twin)
        except Exception:
            continue
        s, info = score(L)
        if s > best_s:
            best_s, best, best_i = s, p, info
            print(f"[{k:3d}] score={s:7.1f} {info}")
    return best, best_s, best_i


if __name__ == "__main__":
    import sys
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    best, s, info = run_search(n=n, seed=seed, t=5.0)
    print("\n=== MEJOR ===", round(s, 1), info)
    print({k: round(v, 4) for k, v in best.items() if k in RANGES})
