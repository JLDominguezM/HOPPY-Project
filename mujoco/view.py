"""Visor interactivo en tiempo real del salto de HOPPY.

Abre una ventana de MuJoCo y corre el controlador en vivo (puedes girar la
camara con el mouse). Ejecutar:  python3 view.py
"""
import time
import numpy as np
import mujoco
import mujoco.viewer

from tune_eval import (make_xml, bezier, FZ_BZ, FX_BZ, NH, NK, Rw, kT, kv, VMAX, IMAX, N, DT)
from control import PARAMS


def main():
    p = PARAMS
    m = mujoco.MjModel.from_xml_string(make_xml(p)); d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    qa = {n: m.jnt_qposadr[jid(n)] for n in ["theta3", "theta4"]}
    va = {n: m.jnt_dofadr[jid(n)] for n in ["theta3", "theta4"]}
    dof34 = [va["theta3"], va["theta4"]]
    hip_a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip")
    knee_a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee")
    fs = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    fg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot")
    hb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link3")

    def frh():
        R = d.xmat[hb].reshape(3, 3); return R.T @ (d.site_xpos[fs] - d.xpos[hb])

    def jac():
        J = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, J, None, fs); return J[:, dof34]

    def ff():
        f = 0.0
        for k in range(d.ncon):
            c = d.contact[k]
            if fg in (c.geom1, c.geom2):
                f6 = np.zeros(6); mujoco.mj_contactForce(m, d, k, f6); f += f6[0]
        return f

    KPf = np.array([p['kp_x'], 0.0, p['kp_z']]); KDf = np.array([p['kd'], 0.0, p['kd']])
    d.qpos[qa["theta3"]], d.qpos[qa["theta4"]] = p['q3_ref'], p['q4_ref']
    mujoco.mj_forward(m, d)
    p_ref = frh().copy() + np.array([p['pref_dx'], 0.0, p['pref_dz']])

    qd_filt = np.zeros(2); q_prev = None
    phase = "stance"; t_td = 0.0; t_lo = -1.0; t = 0.0

    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = [0.3, 0.0, 0.15]
        viewer.cam.distance = 1.8; viewer.cam.azimuth = 90; viewer.cam.elevation = -12
        while viewer.is_running():
            t0 = time.time()
            qd_real = np.array([d.qvel[dof34[0]], d.qvel[dof34[1]]])
            q = np.array([d.qpos[qa["theta3"]], d.qpos[qa["theta4"]]])
            if q_prev is None: q_prev = q.copy()
            af = 10.0 * DT / (1 + 10.0 * DT)
            qd_filt = qd_filt + af * ((q - q_prev) / DT - qd_filt); q_prev = q.copy()
            Jc = jac(); R = d.xmat[hb].reshape(3, 3)
            tau_air = Jc.T @ (KPf * (R @ (p_ref - frh())) - KDf * (Jc @ qd_filt))
            s = (t - t_td) / p['tst'] if phase == "stance" else 0.0
            Fz = bezier(FZ_BZ, s) * p['fz_scale']; Fx = bezier(FX_BZ, s) * p['fx_scale']
            tau_st = Jc.T @ np.array([Fx, 0.0, -Fz])
            Vair = (Rw / (kT * N)) * tau_air + kv * N * qd_filt
            Vst = (Rw / (kT * N)) * tau_st + kv * N * qd_filt
            V = (min(1.0, (t - t_td) / p['blend']) * Vst +
                 (1 - min(1.0, (t - t_td) / p['blend'])) * Vair) if phase == "stance" else Vair
            V = np.clip(V, -VMAX, VMAX)
            i = np.clip((V - kv * N * qd_real) / Rw, -IMAX, IMAX)
            d.ctrl[hip_a], d.ctrl[knee_a] = kT * N * i
            fz_site = d.site_xpos[fs][2]
            contact = (fz_site <= 0.014) or (ff() > 1.0)
            if phase == "stance":
                if (t - t_td) >= p['tst'] or fz_site > 0.02:
                    phase = "aerial"; t_lo = t
            else:
                if contact and (t - t_lo) > 0.03:
                    phase = "stance"; t_td = t
            mujoco.mj_step(m, d); t += DT
            viewer.sync()
            dt = DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
