"""Visor interactivo del GEMELO (twin.py) saltando en vivo, CON CONTRAPESO.

Igual que view_twin.py pero con un contrapeso de 2 kg a x=-0.65 m en el lado de
balance del boom (cumple la Fase 1 de la rubrica: contrapeso en el gantry). El
contrapeso lo soporta twin.make_xml via los params cw_mass/cw_x; NO se modifica
twin.py.

Titulo: "HOPPY Gemelo - Con Contrapeso (2 kg @ 0.65 m)"
(mujoco.viewer.launch_passive no expone API para el titulo de la ventana GLFW, asi
que se imprime al arrancar; la masa roja del contrapeso es visible en el extremo de
balance del boom.)

Uso:  python3 view_twin_cw.py
"""
import time
import numpy as np
import mujoco
import mujoco.viewer

from controller import Hoppy
import twin

TITULO = "HOPPY Gemelo - Con Contrapeso (2 kg @ 0.65 m)"


def main():
    print(TITULO)
    # contrapeso para la rubrica Fase 1: 2 kg a -0.65 m (lado de balance del boom).
    params = dict(twin.DEFAULTS, vis=True, leg_proc=True)
    params["cw_mass"] = 2.0
    params["cw_x"] = -0.65
    h = Hoppy(params, mdl=twin)
    with mujoco.viewer.launch_passive(h.m, h.d) as viewer:
        viewer.cam.distance = 1.3
        viewer.cam.elevation = -14
        viewer.cam.azimuth = 70
        while viewer.is_running():
            t0 = time.time()
            h.step()
            # seguir al hoppy (centra la camara en la cadera; tu controlas angulo/zoom)
            viewer.cam.lookat[:] = h.d.xpos[h.bframe]
            viewer.sync()
            if np.any(np.isnan(h.d.qpos)):
                h.reset()
            dt = h.DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
