"""Visor interactivo del GEMELO (twin.py) saltando en vivo, SIN contrapeso.

Modelo estructural real (gantry+boom+housing + pierna 4-barras del CAD, la MISMA malla
detallada del view_twin_static, dims/masas medidas), con la marcha afinada al angulo del
paper. La camara sigue al hoppy mientras AVANZA alrededor del poste; gira/zoom con el mouse
(tecla 'w' = wireframe). Uso:  python3 view_twin.py

La pierna es PROCEDURAL gris detallada (placas IMP-8/9 + tubo TUB-1 + regaton, color housing)
que SIEMPRE articula conectada (la malla CAD 4-barras se parte porque las placas cruzan la
rodilla; va en view_twin_static). Los 2 resortes son tendones dorados (visuales). SALTA de
verdad: cuerpo +8.8cm, pie despega 6.3cm, avanza 0.59 rad/s (casi una vuelta al poste).

NOTA: el resorte 100% real (Ks=1.67kN/m) NO permite vuelo (rodilla casi rigida); aqui el rebote
lo da un resorte de junta suave (config seed23). Ver HANDOFF para el trade-off fidelidad/salto.
"""
import time
import numpy as np
import mujoco
import mujoco.viewer

from controller import Hoppy
import twin


def main():
    # leg_proc=True -> pierna procedural DETALLADA gris (placas IMP-8/9 + tubo + regaton) que
    # SIEMPRE articula conectada + el resorte como tendon que se estira. (La malla CAD 4-barras
    # se parte en la rodilla porque las placas cruzan la junta; va en view_twin_static.)
    h = Hoppy(dict(twin.DEFAULTS, vis=True, leg_proc=True), mdl=twin)
    with mujoco.viewer.launch_passive(h.m, h.d) as viewer:
        viewer.cam.distance = 1.3
        viewer.cam.elevation = -14
        viewer.cam.azimuth = 70
        while viewer.is_running():
            t0 = time.time()
            h.step()
            # seguir al hoppy (centra la camara en la cadera; tu controlas angulo/zoom)
            viewer.cam.lookat[:] = h.d.xpos[h.bframe]
            viewer.sync()
            if np.any(np.isnan(h.d.qpos)):
                h.reset()
            dt = h.DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
