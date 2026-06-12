"""Ablación del modelo mecánico - Rúbrica Fase 2.2.

Demuestra con simulaciones comparativas la influencia de cada efecto físico del
modelo (armature, damping equivalente, resorte paralelo de rodilla y saturación
de torque) sobre el salto del URDF real con el controlador híbrido del paper
(hoppy_urdf.FORWARD + controller.py).

Cada variante apaga UN efecto y deja el resto intacto:
  full        : modelo completo (baseline)
  sin_armature: armature=0 en cadera y rodilla (sin inercia reflejada N^2*Ir)
  sin_damping : damping=0 en cadera y rodilla (sin pérdidas del actuador;
                el damping del gantry j_damp se conserva: es de la estructura)
  sin_resorte : knee_stiff=0 (sin resorte paralelo de rodilla)
  sin_satur   : VMAX/IMAX -> infinito (actuador ideal sin límites físicos)

armature y damping están inyectados como constantes en el XML (no como params),
así que se ablacionan parchando el XML que genera make_xml (monkeypatch).
La saturación vive en el controlador (Ec.18: V<=12, i<=9.2) -> se ablaciona
sobre la instancia de Hoppy.

Salidas: figuras/ablacion.png + tabla de métricas en stdout.
Correr:  python3 ablacion.py
"""
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import controller
import hoppy_urdf as H

T_TOTAL = 6.0
ORIG_MAKE_XML = H.make_xml


def _xml_sin_armature(xml):
    return re.sub(r'armature="[^"]*"', 'armature="0"', xml)


def _xml_sin_damping(xml):
    # solo theta3/theta4 (actuadores); el damping de theta1/theta2 es del gantry
    def zero(m):
        return re.sub(r'damping="[^"]*"', 'damping="0"', m.group(0))
    return re.sub(r'<joint name="theta[34]"[^>]*>', zero, xml)


def simula(nombre, params=None, xml_patch=None, sin_saturacion=False):
    p = dict(H.FORWARD, fast=True)
    if params:
        p.update(params)
    if xml_patch is not None:
        H.make_xml = lambda pp: xml_patch(ORIG_MAKE_XML(pp))
    try:
        h = controller.Hoppy(p, mdl=H)
    finally:
        H.make_xml = ORIG_MAKE_XML
    if sin_saturacion:
        h.VMAX, h.IMAX = 1e9, 1e9
    keys, L = None, {}
    for _ in range(int(T_TOTAL / h.DT)):
        rec = h.step()
        if keys is None:
            keys = list(rec)
            L = {k: [] for k in keys}
        for k in keys:
            L[k].append(rec[k])
        if np.any(np.isnan(h.d.qpos)):
            print(f"  {nombre}: NaN (divergió) en t={rec['t']:.2f}")
            break
    out = {k: np.array(v) for k, v in L.items()}
    out["nombre"] = nombre
    return out


def metricas(r):
    """Métricas del régimen estable (descarta el primer segundo)."""
    m = r["t"] >= 1.0
    t, fz, bz = r["t"][m], r["foot_z"][m], r["body_z"][m]
    vuelo = r["phase"][m] == 0
    # despegue real del pie (clearance sobre el radio de la esfera del pie)
    clear = max(0.0, fz.max() - 0.016)
    # excursión del cuerpo (pico-valle del régimen)
    exc = bz.max() - bz.min()
    # avance alrededor del poste
    th1 = r["theta1"][m]
    rate = (th1[-1] - th1[0]) / (t[-1] - t[0])
    # saltos = flancos de apoyo->vuelo
    ph = r["phase"][m]
    saltos = int(((ph[1:] - ph[:-1]) == -1).sum())
    return dict(clear_cm=100 * clear, exc_cm=100 * exc, vuelo_pct=100 * vuelo.mean(),
                saltos=saltos, dtheta1=rate,
                tau_max=max(np.abs(r["tau3"][m]).max(), np.abs(r["tau4"][m]).max()),
                i_max=max(np.abs(r["i3"][m]).max(), np.abs(r["i4"][m]).max()))


VARIANTES = [
    ("full",         dict()),
    ("sin_armature", dict(xml_patch=_xml_sin_armature)),
    ("sin_damping",  dict(xml_patch=_xml_sin_damping)),
    ("sin_resorte",  dict(params=dict(knee_stiff=0.0))),
    ("sin_satur",    dict(sin_saturacion=True)),
]
COLORES = {"full": "k", "sin_armature": "tab:blue", "sin_damping": "tab:orange",
           "sin_resorte": "tab:green", "sin_satur": "tab:red"}


def main():
    runs, mets = [], []
    for nombre, kw in VARIANTES:
        print(f"simulando {nombre} ...")
        r = simula(nombre, **kw)
        runs.append(r)
        mets.append(metricas(r))

    # ---- tabla ----
    cab = f"{'variante':<14}{'pie despega':>12}{'excursión':>11}{'% vuelo':>9}" \
          f"{'saltos':>8}{'dθ1/dt':>9}{'τ_max':>8}{'i_max':>8}"
    print("\n" + cab)
    print("-" * len(cab))
    for r, mt in zip(runs, mets):
        print(f"{r['nombre']:<14}{mt['clear_cm']:>10.1f}cm{mt['exc_cm']:>9.1f}cm"
              f"{mt['vuelo_pct']:>8.0f}%{mt['saltos']:>8d}{mt['dtheta1']:>9.2f}"
              f"{mt['tau_max']:>8.2f}{mt['i_max']:>8.1f}")

    # ---- figura ----
    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    for r in runs:
        c = COLORES[r["nombre"]]
        lw = 2.2 if r["nombre"] == "full" else 1.3
        ax[0, 0].plot(r["t"], 100 * r["body_z"], c, lw=lw, label=r["nombre"])
        ax[0, 1].plot(r["t"], 100 * r["foot_z"], c, lw=lw, label=r["nombre"])
    ax[0, 0].set_ylabel("altura del cuerpo (cm)")
    ax[0, 0].set_title("Cuerpo: la ablación cambia el ciclo de salto")
    ax[0, 1].set_ylabel("altura del pie (cm)")
    ax[0, 1].set_title("Pie: despegue real (vuelo)")
    for a in ax[0]:
        a.set_xlabel("t (s)")
        a.grid(True, alpha=0.4)
        a.legend(fontsize=8, ncol=2)
        a.set_xlim(1.0, T_TOTAL)

    nombres = [r["nombre"] for r in runs]
    x = np.arange(len(nombres))
    cs = [COLORES[n] for n in nombres]
    ax[1, 0].bar(x, [mt["clear_cm"] for mt in mets], color=cs)
    ax[1, 0].set_xticks(x, nombres, rotation=15)
    ax[1, 0].set_ylabel("despegue del pie (cm)")
    ax[1, 0].set_title("Altura de salto por variante")
    ax[1, 0].grid(True, axis="y", alpha=0.4)
    ax[1, 1].bar(x - 0.2, [mt["tau_max"] for mt in mets], 0.4, color=cs, label="|τ| máx (N·m)")
    ax[1, 1].bar(x + 0.2, [mt["i_max"] for mt in mets], 0.4, color=cs, alpha=0.45,
                 label="|i| máx (A)")
    ax[1, 1].axhline(H.kT * H.NK * H.IMAX,
                     color="r", ls="--", lw=1, label="τ físico máx (kT·N·i_max)")
    ax[1, 1].set_xticks(x, nombres, rotation=15)
    ax[1, 1].set_yscale("log")
    ax[1, 1].set_title("Esfuerzo de actuador (log): sin saturación se dispara")
    ax[1, 1].grid(True, axis="y", alpha=0.4)
    ax[1, 1].legend(fontsize=8)

    fig.suptitle("Ablación del modelo mecánico - influencia de armature, damping, "
                 "resorte paralelo y saturación (URDF real, controlador híbrido)", y=0.99)
    fig.tight_layout()
    out = "figuras/ablacion.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\nguardado {out}")


if __name__ == "__main__":
    main()
