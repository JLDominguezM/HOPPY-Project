"""Análisis completo de señales del GEMELO HOPPY — Rúbrica Fase 5.

Genera figuras/señales_rubrica.png con 6 subplots (3x2) que cubren las 5 familias
de señales que pide la rúbrica:
  [0,0] posiciones articulares  q3 (cadera), q4 (rodilla)
  [0,1] posición cartesiana del pie  x, y, z  (data.site_xpos[foot_site])
  [1,0] velocidad ESTIMADA filtrada (Hoppy.qd_filt, lambda=10) vs qvel cruda (gris ---)
  [1,1] fuerza de contacto vertical (GRF)  + referencia en 0
  [2,0] torques de control  tau3, tau4  + lineas ±tau_max (kT*N*IMAX)
  [2,1] estado FSM  0=FLIGHT / 1=STANCE  (fondo azul=vuelo, verde=apoyo)

Datos (verificados contra el modelo, no asumidos):
  - tau aplicado = rec["tau3"]/rec["tau4"] de control_step (tau = kT*N*i)
  - GRF = rec["grf"] (mj_contactForce normal; el sensor touch foot_touch lee 0 por
    zona de sitio menor que la esfera del pie)
  - FSM = rec["phase"] (1 stance, 0 flight)
  - vel filtrada = Hoppy.qd_filt[0/1]; pie = h.d.site_xpos[h.fs]
La sim usa vis=False (rápida); la geometría visual no afecta la dinámica.

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
FZ_CONTACT = 2.0                 # umbral de GRF para "en apoyo" (N)


def run():
    """Simula el gemelo (vis=False) y registra todas las señales por paso, alineadas
    al instante de control."""
    h = Hoppy(dict(twin.DEFAULTS), mdl=twin)
    DT = h.DT
    hip_v, knee_v = h.vadr["theta3"], h.vadr["theta4"]
    n = int(T_TOTAL / DT)
    keys = ("t", "q3", "q4", "foot_x", "foot_y", "foot_z",
            "raw_hip", "raw_knee", "filt_hip", "filt_knee",
            "grf", "tau3", "tau4", "phase", "contact")
    log = {k: [] for k in keys}
    for _ in range(n):
        # estado en el instante de control (lo que control_step va a leer)
        raw_hip = h.d.qvel[hip_v]
        raw_knee = h.d.qvel[knee_v]
        foot = h.d.site_xpos[h.fs].copy()
        rec = h.step()                       # control_step (actualiza qd_filt) + mj_step
        log["t"].append(rec["t"])
        log["q3"].append(rec["q3"]);            log["q4"].append(rec["q4"])
        log["foot_x"].append(foot[0]); log["foot_y"].append(foot[1]); log["foot_z"].append(foot[2])
        log["raw_hip"].append(raw_hip);         log["raw_knee"].append(raw_knee)
        log["filt_hip"].append(h.qd_filt[0]);   log["filt_knee"].append(h.qd_filt[1])
        log["grf"].append(rec["grf"]);          log["phase"].append(rec["phase"])
        log["tau3"].append(rec["tau3"]);        log["tau4"].append(rec["tau4"])
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
            segs.append((i, j)); i = j
        else:
            i += 1
    return segs


def main():
    D, DT = run()
    t = D["t"]
    segs = _segments(D["contact"])
    tmaxH = twin.kT * twin.NH * twin.IMAX     # torque max cadera
    tmaxK = twin.kT * twin.NK * twin.IMAX     # torque max rodilla

    fig, ax = plt.subplots(3, 2, figsize=(13, 11))
    fig.suptitle("HOPPY Gemelo Digital — Análisis Completo de Señales (Rúbrica Fase 5)",
                 fontsize=14, fontweight="bold")

    def shade(a):                              # sombra verde donde GRF > 2 N (apoyo)
        for (i0, i1) in segs:
            a.axvspan(t[i0], t[min(i1, len(t) - 1)], color="green", alpha=0.15, lw=0)

    # [0,0] posiciones articulares
    a = ax[0, 0]; shade(a)
    a.plot(t, D["q3"], color="C0", label="q3 cadera")
    a.plot(t, D["q4"], color="C3", label="q4 rodilla")
    a.set_ylabel("Ángulo articular (rad)"); a.set_title("Posiciones articulares")
    a.legend(loc="best"); a.grid(alpha=0.3)

    # [0,1] posición cartesiana del pie (mundo)
    a = ax[0, 1]; shade(a)
    a.plot(t, D["foot_x"], color="C0", label="pie x")
    a.plot(t, D["foot_y"], color="C1", label="pie y")
    a.plot(t, D["foot_z"], color="C2", label="pie z (altura)")
    a.set_ylabel("Posición pie (m)"); a.set_title("Posición cartesiana del pie")
    a.legend(loc="best", fontsize=8); a.grid(alpha=0.3)

    # [1,0] velocidad filtrada vs cruda
    a = ax[1, 0]; shade(a)
    a.plot(t, D["raw_hip"], color="gray", alpha=0.3, ls="--", label="qvel cruda cadera")
    a.plot(t, D["raw_knee"], color="dimgray", alpha=0.3, ls="--", label="qvel cruda rodilla")
    a.plot(t, D["filt_hip"], color="C0", label="filtrada cadera")
    a.plot(t, D["filt_knee"], color="C1", label="filtrada rodilla")
    a.set_ylabel("Vel. articular (rad/s)")
    a.set_title("Velocidad estimada filtrada (λ=10) vs cruda")
    a.legend(loc="best", fontsize=7); a.grid(alpha=0.3)

    # [1,1] GRF
    a = ax[1, 1]; shade(a)
    a.plot(t, D["grf"], color="C2", label="GRF vertical")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("GRF (N)"); a.set_title("Fuerza de contacto (GRF)")
    a.legend(loc="best"); a.grid(alpha=0.3)

    # [2,0] torques de control + limites
    a = ax[2, 0]; shade(a)
    a.plot(t, D["tau3"], color="C0", label="τ cadera")
    a.plot(t, D["tau4"], color="C3", label="τ rodilla")
    a.axhline(tmaxH, color="C0", ls=":", lw=1.0); a.axhline(-tmaxH, color="C0", ls=":", lw=1.0)
    a.axhline(tmaxK, color="C3", ls=":", lw=1.0); a.axhline(-tmaxK, color="C3", ls=":", lw=1.0)
    a.set_xlabel("Tiempo (s)"); a.set_ylabel("Torque (N·m)")
    a.set_title("Torques de control (± τ_max = kT·N·Imax punteado)")
    a.legend(loc="best", fontsize=8); a.grid(alpha=0.3)

    # [2,1] estado FSM (fondo azul=vuelo, verde=apoyo)
    a = ax[2, 1]
    for (i0, i1) in _segments(D["phase"] == 1):
        a.axvspan(t[i0], t[min(i1, len(t) - 1)], color="green", alpha=0.15, lw=0)
    for (i0, i1) in _segments(D["phase"] == 0):
        a.axvspan(t[i0], t[min(i1, len(t) - 1)], color="skyblue", alpha=0.12, lw=0)
    a.plot(t, D["phase"], color="k", drawstyle="steps-post", label="FSM")
    a.set_ylim(-0.2, 1.2); a.set_yticks([0, 1]); a.set_yticklabels(["FLIGHT", "STANCE"])
    a.set_xlabel("Tiempo (s)"); a.set_ylabel("Estado FSM")
    a.set_title("Máquina de estados (FSM)")
    a.legend(loc="best", fontsize=8); a.grid(alpha=0.3)

    os.makedirs(FIGDIR, exist_ok=True)
    out = os.path.join(FIGDIR, "señales_rubrica.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("figura guardada en:", out)

    # --- métricas en consola ---
    hops = sum(1 for (i0, i1) in segs if (i1 - i0) * DT >= 0.01)
    print("GRF pico = %.0f N" % D["grf"].max())
    print("τ_max cadera = %.2f N·m  |  τ_max rodilla = %.2f N·m" % (tmaxH, tmaxK))
    print("saltos detectados = %d  (fases de apoyo > 10 ms, GRF > %.0f N)" % (hops, FZ_CONTACT))


if __name__ == "__main__":
    main()
