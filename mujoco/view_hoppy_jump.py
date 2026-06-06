"""Visor del HOPPY (URDF real) con el control hibrido (controller.py + hoppy_urdf).

Uso:  python3 view_hoppy_jump.py [--viewer]

ESTADO (honesto): la integracion CORRE — compila, sin NaN, contacto de pie y actuadores
hip/knee OK, y el frame de cadera ya esta corregido a la convencion del controlador
(Z=vertical). PERO el gait afinado del twin NO transfiere al URDF (masas/inercias reales
distintas): el cuerpo aun NO despega. Falta re-afinar la marcha (busqueda tipo twin_tune
adaptada a hoppy_urdf). Este visor sirve para observar/depurar el comportamiento.
"""
import sys
import time
import numpy as np
import mujoco
from controller import Hoppy
import hoppy_urdf


def main():
    h = Hoppy(dict(hoppy_urdf.DEFAULTS), mdl=hoppy_urdf)
    if "--viewer" not in sys.argv:
        for _ in range(2000):
            h.step()
            if np.any(np.isnan(h.d.qpos)):
                print("NaN"); return
        print("headless 2000 pasos OK (corre sin NaN; aun NO salta — ver reporte).")
        print("para verlo en 3D:  python3 view_hoppy_jump.py --viewer")
        return
    from mujoco import viewer as mjv
    with mjv.launch_passive(h.m, h.d) as v:
        v.cam.lookat[:] = [0.0, 0.0, 0.25]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 2.4, 50, -16
        while v.is_running():
            t0 = time.time()
            h.step()
            v.sync()
            if np.any(np.isnan(h.d.qpos)):
                h.reset()
            dt = h.DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
