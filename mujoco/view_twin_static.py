"""Visor ESTATICO del gemelo HOPPY bien armado (SIN salto/control).

Muestra el modelo estructural completo (gantry + boom PVC + housing + pierna 4-barras
real) congelado en una pose de reposo, para verificar el ENSAMBLE visual. Gira/zoom
con el mouse. El salto (control) esta desactivado a proposito.

Uso:  python3 view_twin_static.py
"""
import numpy as np
import mujoco
import mujoco.viewer

from controller import Hoppy
import twin

# pose del CAD (theta3=theta4=0): la pierna queda CONECTADA tal como se extrajo del CAD
# (al doblarla se separa en la rodilla porque el 4-barras real != 2 juntas seriales).
# theta2=-0.06 cabecea el boom apenas para que el pie quede sobre el piso (no penetra).
Q2, Q3, Q4 = -0.06, 0.0, 0.0


def main():
    h = Hoppy(dict(twin.DEFAULTS, vis=True), mdl=twin)
    d = h.d
    d.qpos[h.qadr["theta2"]] = Q2
    d.qpos[h.qadr["theta3"]] = Q3
    d.qpos[h.qadr["theta4"]] = Q4
    mujoco.mj_forward(h.m, d)
    with mujoco.viewer.launch_passive(h.m, d) as viewer:
        viewer.cam.lookat[:] = [0.58, 0.12, 0.12]   # centra el housing+pierna (cadera real)
        viewer.cam.distance = 1.1
        viewer.cam.azimuth = 35
        viewer.cam.elevation = -12
        # congelado: NO se llama mj_step, solo se sincroniza (sin salto, sin gravedad)
        while viewer.is_running():
            mujoco.mj_forward(h.m, d)
            viewer.sync()


if __name__ == "__main__":
    main()
