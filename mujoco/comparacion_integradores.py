"""Comparación de integradores - justificación de la nota de Fase 3.1.

La rúbrica recomienda integrator="RK4" (+ solver Newton, iterations=50,
tolerance=1e-8). Nuestros modelos usan integrator="implicitfast". Este script
corre la MISMA simulación (URDF real + controlador híbrido, hoppy_urdf.FORWARD)
con ambas configuraciones y compara:

  - trayectoria del cuerpo y del pie (¿el salto es el mismo?)
  - estabilidad (NaN / divergencia)
  - costo computacional (tiempo de pared)

Justificación física: implicitfast integra IMPLÍCITO los términos dependientes
de velocidad (damping articular y armature) - exactamente los efectos que la
rúbrica pide modelar - lo que lo hace estable con contacto duro a 1 kHz. RK4 es
explícito: con contacto rígido (solref pequeño) y damping alto puede requerir
pasos menores para no oscilar. La documentación de MuJoCo recomienda los
integradores implícitos para modelos con contactos y desaconseja RK4 ahí.

Salidas: figuras/integradores.png + métricas en stdout.
Correr:  python3 comparacion_integradores.py
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
    # config EXACTA recomendada por la rúbrica
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
    runs = [simula("implicitfast (nuestro)"),
            simula("RK4 + Newton/50/1e-8 (rúbrica)", _xml_rk4)]

    print(f"\n{'config':<34}{'estable':>8}{'pie despega':>13}{'saltos':>8}{'t pared':>9}")
    print("-" * 72)
    for r in runs:
        m = r["t"] >= 1.0
        clear = max(0.0, r["foot_z"][m].max() - 0.016) if m.any() else 0.0
        ph = r["phase"][m]
        saltos = int(((ph[1:] - ph[:-1]) == -1).sum())
        print(f"{r['nombre']:<34}{'NaN!' if r['nan'] else 'sí':>8}"
              f"{100*clear:>11.1f}cm{saltos:>8d}{r['wall']:>8.1f}s")

    fig, ax = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    for r, c in zip(runs, ("k", "tab:red")):
        ax[0].plot(r["t"], 100 * r["body_z"], c, lw=1.4, label=r["nombre"])
        ax[1].plot(r["t"], 100 * r["foot_z"], c, lw=1.4, label=r["nombre"])
    ax[0].set_ylabel("altura del cuerpo (cm)")
    ax[1].set_ylabel("altura del pie (cm)")
    ax[1].set_xlabel("t (s)")
    for a in ax:
        a.grid(True, alpha=0.4)
        a.legend()
    fig.suptitle("implicitfast vs RK4 (config recomendada) - misma sim, mismo controlador")
    fig.tight_layout()
    fig.savefig("figuras/integradores.png", dpi=150, bbox_inches="tight")
    print("\nguardado figuras/integradores.png")


if __name__ == "__main__":
    main()
