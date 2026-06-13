"""Short looping GIF of the real URDF model hopping, for the README.

GIFs render inline on GitHub (mp4 does not), so this is the lead visual.

Run:  MUJOCO_GL=egl python3 render_gif.py
Out:  figures/hop.gif
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
import imageio.v2 as imageio

from controller import Hoppy
import hoppy_urdf as H

W, Ht = 440, 330
FPS = 20
T0, T1 = 1.2, 3.4          # skip the first settling second, then ~2.2 s of hopping


def main():
    h = Hoppy(dict(H.FORWARD), mdl=H)
    h.m.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
    h.m.vis.headlight.diffuse[:] = [0.6, 0.6, 0.6]
    ren = mujoco.Renderer(h.m, Ht, W)
    cam = mujoco.MjvCamera()
    frames, nxt = [], T0
    for _ in range(int(T1 / 0.001) + 1):
        h.step()
        if np.any(np.isnan(h.d.qpos)):
            break
        if h.t >= T0 and h.t >= nxt:
            hip = h.d.xpos[h.bframe]
            th1 = h.d.qpos[h.qadr["theta1"]]
            cam.lookat[:] = [hip[0], hip[1], hip[2] - 0.05]
            cam.azimuth = np.degrees(th1) + 70
            cam.elevation = -11
            cam.distance = 0.80
            ren.update_scene(h.d, cam)
            frames.append(ren.render())
            nxt += 1.0 / FPS
    os.makedirs("figures", exist_ok=True)
    out = "figures/hop.gif"
    imageio.mimsave(out, frames, fps=FPS, loop=0)
    print(f"saved {out} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
