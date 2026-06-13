"""Hero figure: a strip of frames of the real URDF model hopping and turning
around the post.

Runs the URDF model (hoppy_urdf.FORWARD) with the hybrid controller, captures
one frame per second from a fixed camera (so the boom sweeping around the post
is visible) and lays them out in a 2x4 grid.

Run:  MUJOCO_GL=egl python3 render_sequence.py
Out:  figuras/hop_sequence.png
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controller import Hoppy
import hoppy_urdf as H

W, Ht = 640, 480          # MuJoCo default offscreen framebuffer
T_TOTAL = 7.0
TIMES = list(range(8))            # frames at t = 0, 1, ..., 7 s


def main():
    h = Hoppy(dict(H.FORWARD), mdl=H)
    ren = mujoco.Renderer(h.m, Ht, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.0, 0.0, 0.12]
    cam.azimuth = 52
    cam.elevation = -13
    cam.distance = 1.9

    frames, yaws, want = [], [], list(TIMES)
    for _ in range(int(T_TOTAL / 0.001) + 1):
        if want and h.t >= want[0] - 1e-9:
            ren.update_scene(h.d, cam)
            frames.append(ren.render())
            yaws.append(float(np.degrees(h.d.qpos[h.qadr["theta1"]])))
            want.pop(0)
        h.step()
        if np.any(np.isnan(h.d.qpos)):
            break

    fig, axes = plt.subplots(2, 4, figsize=(15, 6))
    for k, ax in enumerate(axes.flat):
        ax.axis("off")
        if k < len(frames):
            ax.imshow(frames[k])
            ax.set_title(f"t = {TIMES[k]} s    yaw = {yaws[k]:+.0f} deg", fontsize=10)
    fig.suptitle("HOPPY real URDF model: hopping and turning around the post",
                 fontsize=14, y=0.98)
    fig.tight_layout()
    os.makedirs("figuras", exist_ok=True)
    out = "figuras/hop_sequence.png"
    fig.savefig(out, dpi=140, bbox_inches="tight")
    print(f"saved {out} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
