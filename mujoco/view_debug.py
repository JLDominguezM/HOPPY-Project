"""Inspeccion visual de las CONEXIONES de cada link del gemelo HOPPY.

Genera figuras/debug_links.png con 3 vistas ortogonales (lateral XZ, frontal YZ,
superior XY) en frame MUNDO:
  - cada BODY como punto grande etiquetado con su nombre
  - cada GEOM como circulo proporcional a su tamano, del color de su body
  - leyenda de colores por body y ejes de referencia (origen) en cada vista

Con la opcion  --viewer  ademas abre el viewer interactivo de MuJoCo (modelo congelado)
con las etiquetas de body y los ejes de cada body activados (mjLABEL_BODY / mjFRAME_BODY),
para inspeccionar/corregir en vivo.

Modelo: twin.make_xml(vis=True, leg_proc=True) en qpos=0 (mismo que el dump de
posiciones). NO modifica twin.py.  Uso: python3 view_debug.py [--viewer]
"""
import os
import sys
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")                # siempre guarda PNG (no depende del display -> nunca cuelga)
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D

import twin

HERE = os.path.dirname(os.path.abspath(__file__))
FIGDIR = os.path.join(HERE, "figuras")

# color por body (segun la consigna)
BODY_COLOR = {
    "world": "black", "link1": "red", "link2": "blue",
    "cuerpo_cadera": "green", "link3": "orange", "link4": "purple",
}


def geom_radius(gtype, size):
    """Radio representativo del geom para dibujarlo como circulo."""
    if gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
        return float(size[0])
    if gtype in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        return float(size[0])                  # radio de la seccion
    return float(np.mean(size[:3]))            # box / mesh / ellipsoid: media de semi-dims


def build():
    m = mujoco.MjModel.from_xml_string(twin.make_xml(dict(twin.DEFAULTS, vis=True, leg_proc=True)))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)                     # qpos=0: pose de referencia (congelada)
    return m, d


def make_figure(m, d):
    bodies = [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or f"body_{i}", d.xpos[i].copy())
              for i in range(m.nbody)]
    geoms = []
    for i in range(m.ngeom):
        if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:   # piso: omitir (enorme)
            continue
        bid = m.geom_bodyid[i]
        bn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"
        r = min(geom_radius(m.geom_type[i], m.geom_size[i]), 0.18)
        geoms.append((bn, d.geom_xpos[i].copy(), r))

    views = [("Vista lateral (XZ)", 0, 2, "X (m)", "Z (m)"),
             ("Vista frontal (YZ)", 1, 2, "Y (m)", "Z (m)"),
             ("Vista superior (XY)", 0, 1, "X (m)", "Y (m)")]
    fig, ax = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("HOPPY Gemelo — Conexiones de links (bodies = puntos etiquetados, "
                 "geoms = circulos por body)  [qpos = 0]", fontsize=13, fontweight="bold")

    for k, (title, ia, ib, xl, yl) in enumerate(views):
        a = ax[k]
        a.axhline(0, color="0.8", lw=0.8); a.axvline(0, color="0.8", lw=0.8)   # ejes de referencia (origen)
        for bn, pos, r in geoms:                # geoms como circulos del color del body
            a.add_patch(Circle((pos[ia], pos[ib]), r, color=BODY_COLOR.get(bn, "gray"),
                               alpha=0.18, lw=0))
        for bn, pos in bodies:                  # bodies como puntos grandes etiquetados
            if bn == "world":
                continue
            a.plot(pos[ia], pos[ib], "o", color=BODY_COLOR.get(bn, "gray"),
                   ms=11, mec="k", mew=0.8, zorder=5)
            a.annotate(bn, (pos[ia], pos[ib]), textcoords="offset points",
                       xytext=(6, 5), fontsize=8, fontweight="bold", zorder=6)
        a.set_title(title); a.set_xlabel(xl); a.set_ylabel(yl)
        a.set_aspect("equal", adjustable="datalim"); a.grid(alpha=0.3)

    handles = [Line2D([0], [0], marker="o", color="w", markerfacecolor=c, markeredgecolor="k",
                      markersize=9, label=b) for b, c in BODY_COLOR.items()]
    fig.legend(handles=handles, loc="lower center", ncol=6, fontsize=9, frameon=True)

    os.makedirs(FIGDIR, exist_ok=True)
    out = os.path.join(FIGDIR, "debug_links.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("figura guardada en:", out)
    return fig


def launch_viewer(m, d):
    """Viewer interactivo con etiquetas y ejes de body (solo si hay display)."""
    import mujoco.viewer
    with mujoco.viewer.launch_passive(m, d) as v:
        v.opt.label = mujoco.mjtLabel.mjLABEL_BODY      # nombre de cada body
        v.opt.frame = mujoco.mjtFrame.mjFRAME_BODY      # ejes XYZ de cada body
        v.cam.lookat[:] = [0.36, 0.08, 0.13]; v.cam.distance = 1.6
        v.cam.azimuth = 52; v.cam.elevation = -12
        while v.is_running():
            mujoco.mj_forward(m, d)             # congelado: solo re-sincroniza
            v.sync()


def main():
    m, d = build()
    make_figure(m, d)
    # El viewer 3D interactivo (etiquetas/ejes de body) es OPT-IN para no colgarse en entornos
    # con DISPLAY seteado pero sin sesion interactiva real. Abrelo con:  --viewer
    if "--viewer" in sys.argv:
        print("abriendo viewer de MuJoCo con etiquetas y ejes de body (cierra la ventana para salir)...")
        launch_viewer(m, d)
    else:
        print("listo. Abre figuras/debug_links.png  |  para el 3D etiquetado: python3 view_debug.py --viewer")


if __name__ == "__main__":
    main()
