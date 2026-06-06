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
    DT = m.opt.timestep                 # 0.001 s
    last = 0.0
    rec = None
    t_sim = 0.0                         # tiempo de simulacion acumulado
    t_wall = time.perf_counter()        # tiempo real al arrancar el loop
    with mjv.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.0, 0.0, 0.25]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 2.4, 50, -16
        while v.is_running():
            # catchup: avanza la fisica hasta alcanzar el tiempo real transcurrido.
            # cap de 5 pasos/frame: si el render se atrasa, recupera en frames
            # siguientes en vez de congelarse intentando todo de golpe.
            t_sim_target = time.perf_counter() - t_wall
            pasos = 0
            while t_sim < t_sim_target and pasos < 5:
                rec = h.step()
                mujoco.mj_step(m, d)
                if np.any(np.isnan(d.qpos)):
                    h.reset()
                t_sim += DT
                pasos += 1
            v.sync()                    # UNA vez por frame (no dentro del catchup)
            if rec is not None and h.t - last >= 1.0:
                last = h.t
                print("t=%.1fs  estado=%-6s  hip_z=%.3f  grf=%.0f N"
                      % (h.t, rec["estado"], d.xpos[hip][2], rec["grf"]))
            # si la fisica va adelantada del reloj, espera hasta su instante real
            dormir = (t_wall + t_sim) - time.perf_counter()
            if dormir > 0:
                time.sleep(dormir)


if __name__ == "__main__":
    main()
