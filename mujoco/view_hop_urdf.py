"""Visor del HOPPY URDF real SALTANDO con hop_controller (FSM propio).

Uso:
  python3 view_hop_urdf.py            # headless: 8 s, reporta subida + vuelos
  python3 view_hop_urdf.py --viewer   # GUI en vivo (necesita display)

mj_step lo llama el loop (la arquitectura del proyecto): h.step() fija d.ctrl, el loop
avanza la fisica. Imprime estado FSM / altura / GRF cada segundo.
"""
import sys
import time
import numpy as np
import mujoco
import hoppy_urdf
from hop_controller import HopController


def main():
    m = mujoco.MjModel.from_xml_string(hoppy_urdf.make_xml(hoppy_urdf.DEFAULTS))
    d = mujoco.MjData(m)
    h = HopController(hoppy_urdf.DEFAULTS, m, d)
    h.reset()
    hip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "Link3")   # extremo hoppy (sube al saltar)

    if "--viewer" not in sys.argv:
        z, g = [], []
        for _ in range(8000):
            rec = h.step()
            mujoco.mj_step(m, d)
            if np.any(np.isnan(d.qpos)):
                print("NaN"); return
            z.append(d.xpos[hip][2]); g.append(rec["grf"])
        z, g = np.array(z), np.array(g)
        z_rest = float(z[300:1500].min())
        air = g < 2.0
        fl = sum(1 for k in range(1, len(air)) if air[k] and not air[k - 1])
        print("headless 8 s: reposo=%.3f m  pico=%.3f m  sube=%.1f cm sobre reposo  vuelos~%d"
              % (z_rest, float(z[3000:].max()), (float(z[3000:].max()) - z_rest) * 100, fl))
        print("para verlo en 3D:  python3 view_hop_urdf.py --viewer")
        return

    from mujoco import viewer as mjv
    last = 0.0
    with mjv.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.0, 0.0, 0.25]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 2.4, 50, -16
        while v.is_running():
            t0 = time.time()
            rec = h.step()
            mujoco.mj_step(m, d)
            v.sync()
            if np.any(np.isnan(d.qpos)):
                h.reset()
            if h.t - last >= 1.0:
                last = h.t
                print("t=%.1fs  estado=%-6s  hip_z=%.3f  grf=%.0f N" % (h.t, rec["estado"], d.xpos[hip][2], rec["grf"]))
            dt = m.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
