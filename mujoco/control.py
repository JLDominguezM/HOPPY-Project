"""Corrida final del controlador de HOPPY + graficas (rubrica M4-M6).

Usa el controlador compartido (controller.Hoppy), corre la simulacion, imprime
metricas honestas y genera figuras/resultados.png con 6 paneles:
altura del cuerpo, fuerza del pie, pares, voltaje (saturacion 12V), velocidad
real vs filtrada (Fase 5) y el ciclo limite.

Uso:  python3 control.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controller import simulate
from tune_eval import DEFAULTS

PARAMS = dict(DEFAULTS)   # config fiel + tuneo MuJoCo documentado en tune_eval.py


def figuras(L):
    import os
    os.makedirs("figuras", exist_ok=True)
    t = L["t"]
    ph = L["phase"]
    # apices del cuerpo
    apex = []; cur = -9.9
    for p, z in zip(ph, L["body_z"]):
        if p == 0:
            cur = max(cur, z)
        elif cur > 0:
            apex.append(cur); cur = -9.9
    apex = np.array(apex)
    real = int(np.sum(L["foot_z"] > 0.02))  # solo informativo
    fig, ax = plt.subplots(3, 2, figsize=(12, 9))
    # 1) altura del cuerpo (salto sostenido)
    ax[0, 0].plot(t, L["body_z"], "b")
    ax[0, 0].grid(True)
    ax[0, 0].set_title(f"Altura del cuerpo ({len(apex)} saltos, apex std={np.std(apex[3:])*1000:.0f} mm)")
    ax[0, 0].set_xlabel("Tiempo [s]"); ax[0, 0].set_ylabel("z cadera [m]")
    # 2) fuerza de reaccion del pie (deseada Bezier vs real contacto)
    ax[0, 1].plot(t, L["Fz_des"], "r", lw=0.8, label="Fz deseada (Bezier)")
    ax[0, 1].plot(t, L["grf"], "k", lw=0.8, label="GRF real (contacto)")
    ax[0, 1].grid(True); ax[0, 1].legend(); ax[0, 1].set_title("Fuerza de reaccion del pie")
    ax[0, 1].set_xlabel("Tiempo [s]"); ax[0, 1].set_ylabel("N"); ax[0, 1].set_xlim(4, 6)
    # 3) pares articulares
    ax[1, 0].plot(t, L["tau3"], "b", lw=0.7, label="cadera")
    ax[1, 0].plot(t, L["tau4"], "r", lw=0.7, label="rodilla")
    ax[1, 0].grid(True); ax[1, 0].legend(); ax[1, 0].set_title("Pares articulares")
    ax[1, 0].set_xlabel("Tiempo [s]"); ax[1, 0].set_ylabel("Nm"); ax[1, 0].set_xlim(4, 6)
    # 4) voltaje con saturacion (Fase 3)
    ax[1, 1].plot(t, L["V3"], "b", lw=0.6, label="V cadera")
    ax[1, 1].plot(t, L["V4"], "r", lw=0.6, label="V rodilla")
    ax[1, 1].axhline(12, color="k", ls="--", lw=0.8); ax[1, 1].axhline(-12, color="k", ls="--", lw=0.8)
    ax[1, 1].grid(True); ax[1, 1].legend(); ax[1, 1].set_title("Voltaje (limite +/-12 V)")
    ax[1, 1].set_xlabel("Tiempo [s]"); ax[1, 1].set_ylabel("V"); ax[1, 1].set_xlim(4, 6)
    # 5) velocidad real vs filtrada (Fase 5)
    ax[2, 0].plot(t, L["qd3_real"], "c", lw=0.6, label="real (qvel)")
    ax[2, 0].plot(t, L["qd3_filt"], "b", lw=0.9, label="filtrada (encoder)")
    ax[2, 0].grid(True); ax[2, 0].legend(); ax[2, 0].set_title("Velocidad de cadera: real vs derivada filtrada")
    ax[2, 0].set_xlabel("Tiempo [s]"); ax[2, 0].set_ylabel("rad/s"); ax[2, 0].set_xlim(4, 5)
    # 6) ciclo limite (retrato de fase del cuerpo)
    bz = L["body_z"]; bzv = np.gradient(bz, t)
    half = len(t) // 2
    ax[2, 1].plot(bz[half:], bzv[half:], "b", lw=0.5)
    ax[2, 1].grid(True); ax[2, 1].set_title("Ciclo limite (retrato de fase del cuerpo)")
    ax[2, 1].set_xlabel("z [m]"); ax[2, 1].set_ylabel("dz/dt [m/s]")
    fig.tight_layout(); fig.savefig("figuras/resultados.png", dpi=140)
    print("figuras/resultados.png guardada")


if __name__ == "__main__":
    L = simulate(PARAMS, t_total=8.0)
    ph = L["phase"]
    apex = []; cur = -9.9
    for p, z in zip(ph, L["body_z"]):
        if p == 0:
            cur = max(cur, z)
        elif cur > 0:
            apex.append(cur); cur = -9.9
    apex = np.array(apex)
    ss = L["t"] > 4.0
    print(f"Saltos: {len(apex)}  apex std={np.std(apex[3:])*1000:.0f} mm (ciclo limite)")
    print(f"Altura cadera regimen: [{L['body_z'][ss].min():.3f}, {L['body_z'][ss].max():.3f}] m")
    print(f"Voltaje max |V|={max(np.abs(L['V3']).max(), np.abs(L['V4']).max()):.2f} V (limite 12)")
    print(f"Corriente max |i|={max(np.abs(L['i3']).max(), np.abs(L['i4']).max()):.2f} A (limite 30)")
    print(f"GRF pico={L['grf'][ss].max():.0f} N")
    figuras(L)
