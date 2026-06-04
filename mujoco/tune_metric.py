"""Funcion de score para afinar el salto hacia la referencia MATLAB.

score(params) corre la simulacion, exige que pasen los chequeos criticos de
verify (salto real, estable, avanza, sin colapso, dentro de limites) y entre
los que pasan premia la cercania a la referencia (amplitud 7.2 cm, 2.2 Hz,
avance ~1 rad/s, fraccion de apoyo 60%, pierna q4 a -2.27).

Mas alto = mejor. Los configs que NO saltan de verdad reciben score muy negativo.
"""
import numpy as np
from controller import simulate
from verify import analyze

# objetivos de la referencia MATLAB
REF = dict(amp=0.072, freq=2.21, th1rate=1.0, frac=60.0, q4min=-2.27, hip_lo=0.115)


def evaluate(params, t_total=8.0):
    L = simulate(params, t_total)
    A = analyze(L)
    t = L["t"]
    half = t > t[-1] * 0.5
    th1 = L["theta1"]
    # tasa de avance en regimen (rad/s, valor absoluto)
    th1rate = abs(th1[-1] - th1[half][0]) / (t[-1] - t[half][0] + 1e-9)

    # --- chequeos criticos (deben pasar; si no, no es un salto valido) ---
    crit = dict(
        no_nan=not A["nan"],
        sostenido=A["nhop"] >= 8,
        vuelo=A["n_flight"] >= 6,
        frac_ok=25 <= A["frac"] <= 75,
        sube=A["hip_rise"] > 0.04,
        carga=A["grf_stance"] > 8,
        doblada=A["q4_lo"] < -1.0 and A["q4_hi"] < -0.75,
        no_aletea=A["ratio"] < 2.0,
        no_colapsa=max(abs(A["th2_lo"]), abs(A["th2_hi"])) < 0.5,
        estable=A["apex_std"] < 0.012,
        avanza=th1rate > 0.3,
        volt=A["Vmax"] <= 12.05,
        corr=A["imax"] <= 30.05,
    )
    passed = all(crit.values())

    # --- distancia a la referencia (entre los que pasan) ---
    dist = (abs(A["hip_rise"] - REF["amp"]) * 100        # cm
            + abs(A["freq"] - REF["freq"]) * 1.5
            + abs(th1rate - REF["th1rate"]) * 1.0
            + abs(A["frac"] - REF["frac"]) / 10.0 * 0.5
            + abs(A["q4_lo"] - REF["q4min"]) * 0.5
            + abs(A["hip_lo"] - REF["hip_lo"]) * 100 * 0.3)

    if passed:
        score = 100.0 - dist
    else:
        # credito parcial: cuantos criticos pasa (para que el barrido tenga gradiente)
        score = -100.0 + 5.0 * sum(crit.values())

    return dict(score=float(score), passed=bool(passed),
                amp_cm=A["hip_rise"] * 100, freq=A["freq"], th1rate=th1rate,
                frac=A["frac"], q4min=A["q4_lo"], hip_lo=A["hip_lo"],
                apex_std_mm=A["apex_std"] * 1000, grf_peak=A["grf_peak"],
                Vmax=A["Vmax"], imax=A["imax"],
                fails=[k for k, v in crit.items() if not v])


if __name__ == "__main__":
    import json, sys
    p = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    print(json.dumps(evaluate(p), indent=1))
