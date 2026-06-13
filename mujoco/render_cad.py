"""Renderiza el salto de HOPPY con el overlay CAD realista (gantry fijo + boom de
PVC + housing del CAD + pierna procedural). Camara que sigue al hoppy (azimut ligado
al yaw theta1) para ver la pierna en el plano tangencial mientras avanza alrededor
del poste. Backend EGL.

Uso:  MUJOCO_GL=egl python3 render_cad.py
Sale: figures/salto_cad.mp4  y  figures/hoppy_real.png (still hero)
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
import imageio

from controller import Hoppy
from control import PARAMS

FPS = 30
W, H = 1280, 720
T_TOTAL = 6.0


def main():
    p = dict(PARAMS); p['vis'] = True; p['gantry'] = True
    h = Hoppy(p)
    ren = mujoco.Renderer(h.m, H, W)
    cam = mujoco.MjvCamera()
    frames = []
    next_frame = 0.0
    hero_saved = False
    for _ in range(int(T_TOTAL / 0.001)):
        h.step()
        if np.any(np.isnan(h.d.qpos)):
            break
        if h.t >= next_frame:
            hip = h.d.xpos[h.bframe]
            th1 = h.d.qpos[h.qadr['theta1']]
            cam.lookat[:] = [hip[0], hip[1], hip[2] - 0.08]
            cam.azimuth = np.degrees(th1) + 70     # 3/4 tangencial (lado de la pierna)
            cam.elevation = -11
            cam.distance = 0.78
            ren.update_scene(h.d, cam)
            img = ren.render()
            frames.append(img)
            if not hero_saved and h.t > 4.7:       # still hero en regimen
                imageio.imwrite("figures/hoppy_real.png", img); hero_saved = True
            next_frame += 1.0 / FPS
    os.makedirs("figures", exist_ok=True)
    imageio.mimsave("figures/salto_cad.mp4", frames, fps=FPS)
    print(f"figures/salto_cad.mp4 ({len(frames)} frames) + figures/hoppy_real.png")


if __name__ == "__main__":
    main()
