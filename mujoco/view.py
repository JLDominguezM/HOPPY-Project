"""Visor interactivo en tiempo real del salto de HOPPY.

Usa el controlador compartido (controller.Hoppy). Abre una ventana de MuJoCo y
corre el control en vivo (gira la camara con el mouse). Uso:  python3 view.py
"""
import time
import numpy as np
import mujoco
import mujoco.viewer

from controller import Hoppy
from control import PARAMS
from tune_eval import DT


def main():
    h = Hoppy(PARAMS)
    with mujoco.viewer.launch_passive(h.m, h.d) as viewer:
        viewer.cam.lookat[:] = [0.25, 0.0, 0.12]
        viewer.cam.distance = 1.8
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -10
        while viewer.is_running():
            t0 = time.time()
            h.step()
            viewer.sync()
            if np.any(np.isnan(h.d.qpos)):
                h.reset()
            dt = DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
