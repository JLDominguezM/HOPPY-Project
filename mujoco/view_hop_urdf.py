"""Visor del HOPPY URDF real SALTANDO HACIA ADELANTE con el controlador REAL.

Por defecto usa el controlador HIBRIDO REAL (controller.py = port fiel del simulador
MATLAB / firmware cpu01_main) con las CONSTANTES REALES y el boom balanceado
(hoppy_urdf.FORWARD). El robot salta hacia ADELANTE = -theta1 (el lado OPUESTO a la
pierna, "el lado sin pierna"), igual que el HOPPY del paper y que el MATLAB.

Este es el control que correra en la LaunchPad F28379D, asi que la simulacion sirve para
probar ganancias/constantes ANTES de pasar al fisico.

Uso:
  python3 view_hop_urdf.py            # headless: 10 s, reporta subida + vuelos + sentido
  python3 view_hop_urdf.py --viewer   # GUI en vivo (necesita display)
  python3 view_hop_urdf.py --fsm ...  # usar el FSM tonto viejo (hop_controller), para comparar
"""
import sys
import time
import numpy as np
import mujoco

import hoppy_urdf
import controller


def _build(use_fsm):
    """Devuelve (h, m, d, paso, hip_bid, label). paso() hace control+fisica un dt."""
    if use_fsm:
        from hop_controller import HopController
        m = mujoco.MjModel.from_xml_string(hoppy_urdf.make_xml(hoppy_urdf.DEFAULTS))
        d = mujoco.MjData(m)
        h = HopController(hoppy_urdf.DEFAULTS, m, d)
        h.reset()

        def paso():
            rec = h.step()
            mujoco.mj_step(m, d)
            return rec
        return h, m, d, paso, "Link3", "FSM tonto (hop_controller)"
    # controlador REAL forward (por defecto)
    h = controller.Hoppy(hoppy_urdf.FORWARD, mdl=hoppy_urdf)
    m, d = h.m, h.d

    def paso():
        return h.step()           # control_step + mj_step + t+=DT
    return h, m, d, paso, "cuerpo_cadera", "control REAL forward (MATLAB/firmware)"


def main():
    use_fsm = "--fsm" in sys.argv
    h, m, d, paso, hip_name, label = _build(use_fsm)
    hip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, hip_name)
    th1 = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "theta1")]
    print("controlador:", label)

    if "--viewer" not in sys.argv:
        z, g, t1 = [], [], []
        for _ in range(10000):
            rec = paso()
            if np.any(np.isnan(d.qpos)):
                print("NaN"); return
            z.append(d.xpos[hip][2]); g.append(rec["grf"]); t1.append(d.qpos[th1])
        z, g, t1 = np.array(z), np.array(g), np.array(t1)
        z_rest = float(z[300:1500].min())
        air = g < 2.0
        fl = sum(1 for k in range(1, len(air)) if air[k] and not air[k - 1])
        dth = float(t1[-1] - t1[0])
        sentido = ("-theta1  -> AVANZA al lado SIN pierna (FORWARD, como el paper)"
                   if dth < 0 else "+theta1  -> avanza al lado DE la pierna")
        print("headless 10 s: reposo=%.3f m  pico=%.3f m  sube=%.1f cm  vuelos~%d"
              % (z_rest, float(z[3000:].max()), (float(z[3000:].max()) - z_rest) * 100, fl))
        print("sentido de avance: dtheta1=%.2f rad  (%.3f rad/s)  | %s"
              % (dth, dth / 10.0, sentido))
        print("para verlo en 3D:  python3 view_hop_urdf.py --viewer")
        return

    from mujoco import viewer as mjv
    DT = m.opt.timestep
    last = 0.0
    rec = None
    t_sim = 0.0
    t_wall = time.perf_counter()
    with mjv.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.0, 0.0, 0.25]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 2.6, 50, -16
        while v.is_running():
            t_sim_target = time.perf_counter() - t_wall
            pasos = 0
            while t_sim < t_sim_target and pasos < 5:
                rec = paso()
                if np.any(np.isnan(d.qpos)):
                    h.reset(); t_sim = 0.0; t_wall = time.perf_counter()
                t_sim += DT
                pasos += 1
            v.sync()
            if rec is not None and h.t - last >= 1.0:
                last = h.t
                est = rec.get("estado") or ("apoyo" if rec.get("phase") else "vuelo")
                print("t=%.1fs  fase=%-6s  hip_z=%.3f  theta1=%+.2f  grf=%.0f N"
                      % (h.t, est, d.xpos[hip][2], d.qpos[th1], rec["grf"]))
            dormir = (t_wall + t_sim) - time.perf_counter()
            if dormir > 0:
                time.sleep(dormir)


if __name__ == "__main__":
    main()
