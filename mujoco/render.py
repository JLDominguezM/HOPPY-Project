"""Renderiza un video del salto de HOPPY. Backend EGL (headless).

Usa el controlador compartido (controller.Hoppy). Uso:  MUJOCO_GL=egl python3 render.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
import imageio

from controller import Hoppy
from control import PARAMS

FPS = 30
W, H = 640, 480
T_TOTAL = 6.0


def main():
    h = Hoppy(PARAMS)
    ren = mujoco.Renderer(h.m, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.25, 0.0, 0.12]
    cam.distance = 1.7
    cam.azimuth = 90
    cam.elevation = -10
    frames = []
    next_frame = 0.0
    for _ in range(int(T_TOTAL / 0.001)):
        h.step()
        if h.t >= next_frame:
            ren.update_scene(h.d, cam)
            frames.append(ren.render())
            next_frame += 1.0 / FPS
        if np.any(np.isnan(h.d.qpos)):
            break
    os.makedirs("figuras", exist_ok=True)
    imageio.mimsave("figuras/salto.mp4", frames, fps=FPS)
    print(f"figuras/salto.mp4 guardado ({len(frames)} frames)")


if __name__ == "__main__":
    main()
