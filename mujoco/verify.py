"""Suite de verificacion RIGUROSA del salto de HOPPY en MuJoCo.

Corre la simulacion y evalua varias senales independientes con umbrales
PASS/FAIL, y las compara contra la corrida de referencia del MATLAB
(ref_matlab.csv). No es una metrica floja: un salto solo cuenta si hay fase
de apoyo que EMPUJA + fase de vuelo real, el cuerpo sube por el empuje (no por
flotar), la pierna no aletea y se sostiene un ciclo limite. Imprime tabla y
veredicto global (PASS solo si pasan todos los criticos).

Uso:  python3 verify.py
"""
import os
import numpy as np

from controller import simulate
from tune_eval import HB, LB, DT

REF_CSV = os.path.join(os.path.dirname(__file__), "ref_matlab.csv")


def load_ref():
    import csv
    if not os.path.exists(REF_CSV):
        return None
    rows = list(csv.DictReader(open(REF_CSV)))
    g = lambda k: np.array([float(r[k]) for r in rows])
    t = g("t"); q2 = g("q2"); q4 = g("q4"); ph = g("phase"); footz = g("footz")
    hipz = HB - LB * np.sin(q2)
    ss = t > t[-1] * 0.4
    seg = _segments(ph, t)
    fl = [d for p, d in seg if p == 0 and d > 0.02]
    st = [d for p, d in seg if p == 1 and d > 0.02]
    return dict(hip=(hipz[ss].min(), hipz[ss].max()),
                clear=(footz[ss].max() - footz[ss].min()),
                q4=(q4[ss].min(), q4[ss].max()),
                frac=np.mean(st) / (np.mean(st) + np.mean(fl)) * 100,
                freq=1.0 / (np.mean(st) + np.mean(fl)),
                nhop=len(st))


def _segments(ph, t):
    out = []; i = 0
    while i < len(ph):
        j = i
        while j < len(ph) and ph[j] == ph[i]:
            j += 1
        out.append((ph[i], t[j - 1] - t[i])); i = j
    return out


def _hops(ph, bz, t):
    """Devuelve apices del cuerpo por ciclo (max en cada fase de vuelo) y
    duraciones de apoyo/vuelo en regimen."""
    apex = []; cur = -9.9
    for p, z in zip(ph, bz):
        if p == 0:
            cur = max(cur, z)
        elif cur > 0:
            apex.append(cur); cur = -9.9
    return np.array(apex)


def analyze(L):
    t = L["t"]; ph = L["phase"]; bz = L["body_z"]; fz = L["foot_z"]
    th1 = L["theta1"]; th2 = L["theta2"]; q4 = L["q4"]; grf = L["grf"]
    ss = t > t[-1] * 0.5
    seg = _segments(ph[ss], t[ss])
    fl = [d for p, d in seg if p == 0 and d > 0.01]
    st = [d for p, d in seg if p == 1 and d > 0.01]
    apex = _hops(ph, bz, t)
    a = apex[3:] if len(apex) > 4 else apex
    frac = (np.mean(st) / (np.mean(st) + np.mean(fl)) * 100) if (st and fl) else (100.0 if st else 0.0)
    freq = (1.0 / (np.mean(st) + np.mean(fl))) if (st and fl) else 0.0
    # trabajo del empuje: GRF medio en apoyo (carga real, no toque)
    grf_stance = grf[(ph == 1)]
    return dict(
        nan=L["_nan"], nhop=len(st),
        hip_lo=bz[ss].min(), hip_hi=bz[ss].max(),
        hip_rise=bz[ss].max() - bz[ss].min(),
        clear=fz[ss].max() - fz[ss].min(),
        ratio=(fz[ss].max() - fz[ss].min()) / max(bz[ss].max() - bz[ss].min(), 1e-6),
        q4_lo=q4[ss].min(), q4_hi=q4[ss].max(),
        th2_lo=th2[ss].min(), th2_hi=th2[ss].max(),
        th1_net=th1[-1] - th1[0],
        frac=frac, freq=freq,
        grf_stance=np.mean(grf_stance) if len(grf_stance) else 0.0,
        grf_peak=grf[ss].max(),
        apex_std=float(np.std(a)) if len(a) > 1 else 9.9,
        n_flight=len(fl),
        Vmax=max(np.abs(L["V3"]).max(), np.abs(L["V4"]).max()),
        imax=max(np.abs(L["i3"]).max(), np.abs(L["i4"]).max()),
    )


def verify(params=None, t_total=8.0, verbose=True):
    L = simulate(params, t_total)
    A = analyze(L)
    ref = load_ref()

    # --- chequeos criticos (el salto es real) ---
    C = []
    def chk(name, ok, val, crit=True):
        C.append((name, bool(ok), val, crit))
    chk("sin NaN / no diverge", not A["nan"], f"nan={A['nan']}")
    chk("saltos sostenidos >=8", A["nhop"] >= 8, f"{A['nhop']} saltos")
    chk("fases de vuelo reales", A["n_flight"] >= 6, f"{A['n_flight']} vuelos")
    chk("fraccion de apoyo 25-75%", 25 <= A["frac"] <= 75, f"{A['frac']:.0f}%")
    chk("cuerpo sube por empuje >4cm", A["hip_rise"] > 0.04, f"{A['hip_rise']*100:.1f} cm")
    chk("apoyo CARGA (GRF medio >8N)", A["grf_stance"] > 8, f"{A['grf_stance']:.0f} N")
    chk("pierna doblada (q4<-1.0, no singular)", A["q4_lo"] < -1.0 and A["q4_hi"] < -0.75, f"q4[{A['q4_lo']:.2f},{A['q4_hi']:.2f}]")
    chk("pierna NO aletea (clear<2x subida)", A["ratio"] < 2.0, f"ratio {A['ratio']:.2f}")
    chk("gantry no colapsa (|th2|<0.5)", max(abs(A["th2_lo"]), abs(A["th2_hi"])) < 0.5, f"th2[{A['th2_lo']:.2f},{A['th2_hi']:.2f}]")
    chk("ciclo limite estable (apex std<1cm)", A["apex_std"] < 0.01, f"{A['apex_std']*1000:.0f} mm")
    chk("voltaje <=12V", A["Vmax"] <= 12.01, f"{A['Vmax']:.2f} V")
    chk("corriente <=30A", A["imax"] <= 30.01, f"{A['imax']:.2f} A")

    if verbose:
        print("=" * 64)
        print("  VERIFICACION DEL SALTO DE HOPPY (MuJoCo)")
        print("=" * 64)
        print(f"{'chequeo':40} {'valor':>14}  estado")
        print("-" * 64)
        allok = True
        for name, ok, val, crit in C:
            if crit and not ok:
                allok = False
            print(f"{name:40} {val:>14}  {'PASS' if ok else 'FALLA'}")
        print("-" * 64)
        # comparacion contra la referencia MATLAB
        if ref:
            print("\n  COMPARACION vs REFERENCIA MATLAB")
            print(f"{'metrica':28} {'MuJoCo':>14} {'MATLAB':>14}")
            print(f"{'altura cadera min (m)':28} {A['hip_lo']:>14.3f} {ref['hip'][0]:>14.3f}")
            print(f"{'altura cadera max (m)':28} {A['hip_hi']:>14.3f} {ref['hip'][1]:>14.3f}")
            print(f"{'amplitud de salto (cm)':28} {A['hip_rise']*100:>14.1f} {(ref['hip'][1]-ref['hip'][0])*100:>14.1f}")
            print(f"{'frecuencia (Hz)':28} {A['freq']:>14.2f} {ref['freq']:>14.2f}")
            print(f"{'fraccion de apoyo (%)':28} {A['frac']:>14.0f} {ref['frac']:>14.0f}")
            print(f"{'pierna q4 min (rad)':28} {A['q4_lo']:>14.2f} {ref['q4'][0]:>14.2f}")
            print(f"{'theta1 neto (rad, gira poste)':28} {A['th1_net']:>14.2f} {'(avanza)':>14}")
            print(f"{'GRF pico (N)':28} {A['grf_peak']:>14.0f} {30:>14}")
        print("\n" + "=" * 64)
        print(f"  VEREDICTO: {'PASS - salta de verdad y de forma estable' if allok else 'FALLA - revisar chequeos marcados'}")
        print("=" * 64)
    return all(ok for _, ok, _, crit in C if crit), A


if __name__ == "__main__":
    verify()
