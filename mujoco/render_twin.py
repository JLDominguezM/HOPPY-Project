"""Renderiza el salto del GEMELO (twin.py): modelo estructural real con las mallas
CAD como eslabones, dimensiones y masas medidas. Camara que sigue al hoppy.

Uso:  MUJOCO_GL=egl python3 render_twin.py
Sale: figuras/twin_salto.mp4  y  figuras/twin_hero.png
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
import imageio

from controller import Hoppy
import twin

FPS, W, H, T_TOTAL = 30, 1280, 720, 6.0


def main():
    p = dict(twin.DEFAULTS); p['vis'] = True; p['leg_proc'] = True  # pierna limpia (4-barras se desconecta al saltar)
    h = Hoppy(p, mdl=twin)
    ren = mujoco.Renderer(h.m, H, W)
    cam = mujoco.MjvCamera()
    frames, nf, hero = [], 0.0, False
    for _ in range(int(T_TOTAL / 0.001)):
        h.step()
        if np.any(np.isnan(h.d.qpos)):
            break
        if h.t >= nf:
            hip = h.d.xpos[h.bframe]; th1 = h.d.qpos[h.qadr['theta1']]
            cam.lookat[:] = [hip[0], hip[1], hip[2] - 0.05]
            cam.azimuth = np.degrees(th1) + 70
            cam.elevation = -11; cam.distance = 0.85
            ren.update_scene(h.d, cam); img = ren.render(); frames.append(img)
            if not hero and h.t > 3.5:
                imageio.imwrite("figuras/twin_hero.png", img); hero = True
            nf += 1.0 / FPS
    os.makedirs("figuras", exist_ok=True)
    imageio.mimsave("figuras/twin_salto.mp4", frames, fps=FPS)
    print(f"figuras/twin_salto.mp4 ({len(frames)} frames) + figuras/twin_hero.png")


if __name__ == "__main__":
    main()
