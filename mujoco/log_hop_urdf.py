"""log_hop_urdf.py — registra 10 s del salto del HOPPY URDF a CSV (hop_urdf_log.csv).

Una fila por paso (1 kHz) tras asentar 1 s. Columnas: tiempo, angulos/velocidades de
junta, posicion del pie y del cuerpo, GRF (mj_contactForce normal y sensor touch), torques
y estado FSM. Alimenta las graficas de señales (señales_urdf.png) y el analisis del reporte.

GRF: se usa la componente NORMAL del contacto (f[0] del frame de contacto), igual que el
controlador (hop_controller.foot_force) y que el sensor touch -> grf_contact y grf_sensor
coinciden, lo que valida el sensor. (OJO: f[2] seria una componente TANGENCIAL de friccion,
no la fuerza de reaccion del piso.)

Uso:  python3 log_hop_urdf.py   ->  hop_urdf_log.csv
"""
import csv
import numpy as np
import mujoco
import hoppy_urdf
from hop_controller import HopController

T_SETTLE = 1000        # pasos para asentar antes de registrar (1 s)
N_STEPS = 10000        # 10 s @ 1 kHz


def foot_grf_normal(m, d, foot_geom):
    """Suma la componente NORMAL (f[0]) de los contactos del geom 'foot'."""
    fn = 0.0
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        if foot_geom in (c.geom1, c.geom2):
            mujoco.mj_contactForce(m, d, i, f6)
            fn += f6[0]
    return fn


def main(out="hop_urdf_log.csv"):
    m = mujoco.MjModel.from_xml_string(hoppy_urdf.make_xml(hoppy_urdf.DEFAULTS))
    d = mujoco.MjData(m)
    h = HopController(hoppy_urdf.DEFAULTS, m, d)
    h.reset()
    for _ in range(T_SETTLE):
        mujoco.mj_step(m, d)

    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    q = {n: m.jnt_qposadr[jid(n)] for n in ("theta1", "theta2", "theta3", "theta4")}
    v = {n: m.jnt_dofadr[jid(n)] for n in ("theta3", "theta4")}
    foot_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    link3 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "Link3")
    link4 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "Link4")
    sensor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "foot_touch")
    sadr = m.sensor_adr[sensor]
    foot_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot")
    a_hip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip")
    a_knee = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee")
    DT = m.opt.timestep

    rows = []
    for step in range(N_STEPS):
        rec = h.step()
        mujoco.mj_step(m, d)
        rows.append({
            "t":           step * DT,
            "theta1":      float(d.qpos[q["theta1"]]),
            "theta2":      float(d.qpos[q["theta2"]]),
            "theta3":      float(d.qpos[q["theta3"]]),
            "theta4":      float(d.qpos[q["theta4"]]),
            "dtheta3":     float(d.qvel[v["theta3"]]),
            "dtheta4":     float(d.qvel[v["theta4"]]),
            "foot_x":      float(d.site_xpos[foot_site][0]),
            "foot_y":      float(d.site_xpos[foot_site][1]),
            "foot_z":      float(d.site_xpos[foot_site][2]),
            "link3_z":     float(d.xpos[link3][2]),
            "link4_z":     float(d.xpos[link4][2]),
            "grf_contact": foot_grf_normal(m, d, foot_geom),
            "grf_sensor":  float(d.sensordata[sadr]),
            "tau_hip":     float(d.actuator_force[a_hip]),
            "tau_knee":    float(d.actuator_force[a_knee]),
            "estado_fsm":  rec["estado"],
            "en_vuelo":    int(rec["grf"] < 2.0),
        })

    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    gc = np.array([r["grf_contact"] for r in rows])
    gs = np.array([r["grf_sensor"] for r in rows])
    print("CSV guardado: %s (%d filas)" % (out, len(rows)))
    print("columnas:", list(rows[0].keys()))
    print("validacion sensor: grf_contact max=%.1f N | grf_sensor max=%.1f N | dif media=%.3f N"
          % (gc.max(), gs.max(), float(np.mean(np.abs(gc - gs)))))


if __name__ == "__main__":
    main()
