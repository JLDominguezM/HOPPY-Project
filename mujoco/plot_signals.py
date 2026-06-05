"""Figura de la Fase 5 de la rubrica: estimacion de velocidad por derivada
filtrada (lambda ~= 10) vs la velocidad cruda (qvel) de MuJoCo, para el GEMELO.

Genera figuras/encoder_signals.png (4 subplots 2x2):
  [0,0] angulos q3 (cadera) y q4 (rodilla)
  [0,1] voltajes de cadera y rodilla (ref +-12 V)
  [1,0] velocidad cadera: qvel cruda vs estimada filtrada
  [1,1] velocidad rodilla: qvel cruda vs estimada filtrada
Sombra verde = fase de apoyo (sensor de contacto foot_touch > 2 N).

Datos verificados contra el modelo compilado (no asumidos):
  - la velocidad filtrada vive en  Hoppy.qd_filt  (ndarray (2,), [hip=theta3, knee=theta4])
  - theta3 (cadera): indice 2 en qpos/qvel ;  theta4 (rodilla): indice 3
  - NO existe un sensor llamado 'Fz'. El sensor touch 'foot_touch' (sensordata[9]) lee
    0 N porque su zona (foot_site, size 0.004) no cubre la esfera de contacto del pie
    (radio 0.016); por eso el apoyo se detecta con el GRF real (mj_contactForce, el mismo
    que usa la FSM del controlador) via rec['grf'].
  - site del pie = 'foot_site'

Hoppy.step() avanza control + fisica y devuelve el log de control_step (que incluye
la cadera pero NO la velocidad de rodilla), por eso el loop tambien lee qd_filt[1] y
qvel[knee] directamente. control_step() NO recibe 'data' (usa self.d). simulate() si
da arrays paso a paso, pero tampoco registra la velocidad de rodilla.

Uso:  python3 plot_signals.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")            # backend sin pantalla (solo guarda PNG)
import matplotlib.pyplot as plt

import twin
from controller import Hoppy, LAMBDA

HERE = os.path.dirname(os.path.abspath(__file__))
FIGDIR = os.path.join(HERE, "figuras")
T_TOTAL = 5.0                    # segundos a simular
FZ_CONTACT = 2.0                 # umbral de fuerza para "en apoyo" (N)


def run():
    """Simula el gemelo y registra las senales paso a paso (todo en el mismo
    instante de control)."""
    h = Hoppy(dict(twin.DEFAULTS), mdl=twin)
    DT = h.DT
    hip_v, knee_v = h.vadr["theta3"], h.vadr["theta4"]          # 2, 3 (verificado)

    n = int(T_TOTAL / DT)
    keys = ("t", "q3", "q4", "raw_hip", "raw_knee", "filt_hip", "filt_knee",
            "V_hip", "V_knee", "i_hip", "i_knee", "contact")
    log = {k: [] for k in keys}
    for _ in range(n):
        # estado en el instante de control (lo que control_step va a leer)
        raw_hip = h.d.qvel[hip_v]
        raw_knee = h.d.qvel[knee_v]
        rec = h.step()                       # control_step (actualiza qd_filt) + mj_step
        log["t"].append(rec["t"])
        log["q3"].append(rec["q3"]);          log["q4"].append(rec["q4"])
        log["raw_hip"].append(raw_hip);       log["raw_knee"].append(raw_knee)
        log["filt_hip"].append(h.qd_filt[0]); log["filt_knee"].append(h.qd_filt[1])
        log["V_hip"].append(rec["V3"]);       log["V_knee"].append(rec["V4"])
        log["i_hip"].append(rec["i3"]);       log["i_knee"].append(rec["i4"])
        log["contact"].append(bool(rec["grf"] > FZ_CONTACT))
        if np.any(np.isnan(h.d.qpos)):
            print("AVISO: NaN detectado, simulacion truncada")
            break
    return {k: np.asarray(v) for k, v in log.items()}, DT


def _segments(mask):
    """Indices [(i0, i1), ...] de tramos contiguos True en mask."""
    segs, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def main():
    D, DT = run()
    t = D["t"]
    segs = _segments(D["contact"])
    af = LAMBDA * DT / (1.0 + LAMBDA * DT)

    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("HOPPY Gemelo Digital — Señales de Control y Estimación de Velocidad",
                 fontsize=14, fontweight="bold")

    def shade(a):                              # sombra verde en cada fase de apoyo
        for (i0, i1) in segs:
            a.axvspan(t[i0], t[min(i1, len(t) - 1)], color="green", alpha=0.15, lw=0)

    # [0,0] angulos de junta
    a = ax[0, 0]; shade(a)
    a.plot(t, D["q3"], color="C0", label="q3 cadera")
    a.plot(t, D["q4"], color="C3", label="q4 rodilla")
    a.set_ylabel("Ángulo de junta (rad)"); a.set_title("Ángulos de junta")
    a.legend(loc="best"); a.grid(alpha=0.3)

    # [0,1] voltajes de actuador
    a = ax[0, 1]; shade(a)
    a.plot(t, D["V_hip"], color="C0", label="V cadera")
    a.plot(t, D["V_knee"], color="C3", label="V rodilla")
    a.axhline(12, ls=":", color="k", lw=1.0); a.axhline(-12, ls=":", color="k", lw=1.0)
    a.set_ylabel("Voltaje (V)"); a.set_title("Voltaje de actuador (ref ±12 V)")
    a.legend(loc="best"); a.grid(alpha=0.3)

    # [1,0] velocidad cadera: cruda vs filtrada
    a = ax[1, 0]; shade(a)
    a.plot(t, D["raw_hip"], color="gray", alpha=0.4, label="qvel cruda (MuJoCo)")
    a.plot(t, D["filt_hip"], color="C0", label="qvel filtrada (estimada)")
    a.set_xlabel("Tiempo (s)"); a.set_ylabel("Vel. cadera (rad/s)")
    a.set_title("Velocidad cadera: cruda vs filtrada")
    a.legend(loc="best"); a.grid(alpha=0.3)
    a.text(0.97, 0.04,
           "λ = %.0f rad/s  |  Δt = %.0f ms  |  αf ≈ %.4f" % (LAMBDA, DT * 1000, af),
           transform=a.transAxes, ha="right", va="bottom", fontsize=9,
           bbox=dict(boxstyle="round", fc="white", ec="0.6", alpha=0.9))

    # [1,1] velocidad rodilla: cruda vs filtrada
    a = ax[1, 1]; shade(a)
    a.plot(t, D["raw_knee"], color="gray", alpha=0.4, label="qvel cruda (MuJoCo)")
    a.plot(t, D["filt_knee"], color="C1", label="qvel filtrada (estimada)")
    a.set_xlabel("Tiempo (s)"); a.set_ylabel("Vel. rodilla (rad/s)")
    a.set_title("Velocidad rodilla: cruda vs filtrada")
    a.legend(loc="best"); a.grid(alpha=0.3)

    os.makedirs(FIGDIR, exist_ok=True)
    out = os.path.join(FIGDIR, "encoder_signals.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("figura guardada en:", out)

    # --- metricas en consola ---
    v_max = float(max(np.abs(D["V_hip"]).max(), np.abs(D["V_knee"]).max()))
    i_max = float(max(np.abs(D["i_hip"]).max(), np.abs(D["i_knee"]).max()))
    hops = sum(1 for (i0, i1) in segs if (i1 - i0) * DT >= 0.01)   # apoyos > 10 ms
    print("V_max  = %.2f V" % v_max)
    print("I_max  = %.2f A" % i_max)
    print("saltos detectados = %d  (fases de apoyo > 10 ms, GRF > %.0f N)"
          % (hops, FZ_CONTACT))


if __name__ == "__main__":
    main()
