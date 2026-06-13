"""Hero figure: a strip of frames of the real URDF model hopping and turning
around the post.

Runs the URDF model (hoppy_urdf.FORWARD) with the hybrid controller and captures
one frame per second. The camera follows the hip and its azimuth tracks the boom
yaw (theta1), so the leg stays framed and large while the yaw label shows the
robot advancing around the post.

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
TIMES = list(range(8))    # frames at t = 0, 1, ..., 7 s


def main():
    h = Hoppy(dict(H.FORWARD), mdl=H)
    # brighten the scene a little so the robot reads clearly against the floor
    h.m.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
    h.m.vis.headlight.diffuse[:] = [0.6, 0.6, 0.6]
    ren = mujoco.Renderer(h.m, Ht, W)
    cam = mujoco.MjvCamera()

    frames, yaws, want = [], [], list(TIMES)
    for _ in range(int(T_TOTAL / 0.001) + 1):
        if want and h.t >= want[0] - 1e-9:
            hip = h.d.xpos[h.bframe]
            th1 = h.d.qpos[h.qadr["theta1"]]
            cam.lookat[:] = [hip[0], hip[1], hip[2] - 0.05]
            cam.azimuth = np.degrees(th1) + 70    # 3/4 tangential view (leg side)
            cam.elevation = -11
            cam.distance = 0.85
            ren.update_scene(h.d, cam)
            frames.append(ren.render())
            yaws.append(float(np.degrees(th1)))
            want.pop(0)
        h.step()
        if np.any(np.isnan(h.d.qpos)):
            break

    fig, axes = plt.subplots(2, 4, figsize=(15, 6))
    for k, ax in enumerate(axes.flat):
        ax.axis("off")
        if k < len(frames):
            ax.imshow(frames[k])
            # label inside the frame (white on the dark render) so titles never overlap
            ax.text(0.035, 0.96, f"t = {TIMES[k]} s\nyaw = {yaws[k]:+.0f} deg",
                    transform=ax.transAxes, va="top", ha="left", fontsize=9, color="white",
                    bbox=dict(boxstyle="round,pad=0.3", fc="black", alpha=0.45, ec="none"))
    fig.suptitle("HOPPY real URDF model: hopping and turning around the post", fontsize=14)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.93, bottom=0.01, wspace=0.04, hspace=0.04)
    os.makedirs("figuras", exist_ok=True)
    out = "figuras/hop_sequence.png"
    fig.savefig(out, dpi=140)
    print(f"saved {out} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
