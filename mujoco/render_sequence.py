"""Hero figure: a strip of frames of the real URDF model hopping and turning
around the post.

Runs the URDF model (hoppy_urdf.FORWARD) with the hybrid controller and captures
one frame per second. The camera follows the hip and its azimuth tracks the boom
yaw (theta1), so the leg stays framed and large while the yaw label shows the
robot advancing around the post.

The 2x4 grid is composited by hand (PIL) with explicit white gaps and the label
burned into each frame, so nothing can overlap.

Run:  MUJOCO_GL=egl python3 render_sequence.py
Out:  figuras/hop_sequence.png
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
import matplotlib.font_manager as fm

from controller import Hoppy
import hoppy_urdf as H

W, Ht = 640, 480          # MuJoCo default offscreen framebuffer
T_TOTAL = 7.0
TIMES = list(range(8))    # frames at t = 0, 1, ..., 7 s
COLS = 4
GAP = 18                  # white gap between frames (px)
BORDER = 18
TITLE_H = 70


def _font(sz):
    return ImageFont.truetype(fm.findfont("DejaVu Sans"), sz)


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

    f_lbl = _font(22)
    f_title = _font(30)

    # burn a label into the top-left of each frame (white text on a translucent box)
    labeled = []
    for k, fr in enumerate(frames):
        im = Image.fromarray(np.asarray(fr)).convert("RGBA")
        txt = f"t = {TIMES[k]} s    yaw = {yaws[k]:+.0f} deg"
        box = ImageDraw.Draw(im).textbbox((0, 0), txt, font=f_lbl)
        tw, th = box[2] - box[0], box[3] - box[1]
        overlay = Image.new("RGBA", im.size, (0, 0, 0, 0))
        od = ImageDraw.Draw(overlay)
        od.rounded_rectangle([10, 10, 10 + tw + 18, 14 + th + 12], radius=6, fill=(0, 0, 0, 140))
        od.text((19, 16), txt, font=f_lbl, fill=(255, 255, 255, 255))
        labeled.append(Image.alpha_composite(im, overlay).convert("RGB"))

    rows = (len(labeled) + COLS - 1) // COLS
    total_w = BORDER * 2 + COLS * W + (COLS - 1) * GAP
    total_h = BORDER * 2 + TITLE_H + rows * Ht + (rows - 1) * GAP
    canvas = Image.new("RGB", (total_w, total_h), (255, 255, 255))
    for k, im in enumerate(labeled):
        r, c = divmod(k, COLS)
        x = BORDER + c * (W + GAP)
        y = BORDER + TITLE_H + r * (Ht + GAP)
        canvas.paste(im, (x, y))

    dr = ImageDraw.Draw(canvas)
    title = "HOPPY real URDF model: hopping and turning around the post"
    tb = dr.textbbox((0, 0), title, font=f_title)
    dr.text(((total_w - (tb[2] - tb[0])) // 2, BORDER + (TITLE_H - (tb[3] - tb[1])) // 2 - 4),
            title, font=f_title, fill=(25, 25, 25))

    os.makedirs("figuras", exist_ok=True)
    out = "figuras/hop_sequence.png"
    canvas.save(out)
    print(f"saved {out} {canvas.size} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
