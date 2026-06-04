"""Harness de evaluacion parametrizado para tunear el salto de HOPPY en MuJoCo.

evaluate(params) -> dict con score y metricas. Lo usan los agentes de tuning.
CLI:  python3 tune_eval.py '{"kp":800,"fz_scale":1.0,...}'
Imprime un JSON con el resultado.

Parametros (con defaults):
  modelo:  cw_mass, cw_pos, body_hip, post_h, knee_stiff, spring_ref, solref0
  control: kp_x, kp_z, kd, fz_scale, fx_scale, tst, blend, pref_dx, pref_dz,
           q3_ref, q4_ref
Score: numero de saltos sostenidos, premiando consistencia de altura y
       penalizando inestabilidad (NaN o cuerpo bajo el suelo).
"""
import sys, json
import numpy as np
import mujoco
from math import comb

# parametros fisicos fijos
NH, NK = 26.9, 28.8
Rw, kT, kv = 1.3, 0.0135, 0.0186
VMAX, IMAX = 12.0, 30.0
N = np.array([NH, NK])
ARM_H, ARM_K = NH**2*7e-6, NK**2*7e-6
RBOOM, L3, L4 = 0.556, 0.096, 0.1545
M1, M3, M4 = 0.268, 0.656, 0.149
DT = 0.001
FZ_BZ = np.array([0.0, 20.0, 100.0, 0.0, 0.0])
FX_BZ = np.array([0.0,  0.0, -25.0, 0.0, 0.0])

DEFAULTS = dict(
    cw_mass=2.1, cw_pos=-0.45, body_hip=1.2, post_h=0.20,
    knee_stiff=3.0, spring_ref=-1.5708, solref0=0.002,
    j_damp=0.05,                       # amortiguamiento de las juntas pasivas del gantry (fric. rodamientos)
    kp_x=800.0, kp_z=800.0, kd=15.0, fz_scale=1.0, fx_scale=1.0,
    tst=0.15, blend=0.010, pref_dx=0.0, pref_dz=0.0,
    q3_ref=np.pi/3, q4_ref=-np.pi/2,
)


def bezier(coef, s):
    n = len(coef) - 1
    s = min(max(s, 0.0), 1.0)
    return sum(comb(n, i) * coef[i] * s**i * (1 - s)**(n - i) for i in range(n + 1))


def make_xml(p):
    I1 = (0.00115952, 0.00104649, 0.00030518)
    I3 = (0.00082110, 0.00235762, 0.00168340)
    I4 = (0.00039424, 0.00032191, 0.00010442)
    return f"""<mujoco model="hoppy">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
  <default>
    <joint damping="0"/><geom contype="0" conaffinity="0"/>
    <default class="contact"><geom contype="1" conaffinity="1" solref="{p['solref0']} 1"
       solimp="0.95 0.99 0.001" friction="2.0 0.1 0.1"/></default>
  </default>
  <worldbody>
    <geom name="floor" class="contact" type="plane" size="3 3 0.1"/>
    <geom name="post" type="cylinder" fromto="0 0 0 0 0 {p['post_h']}" size="0.02"/>
    <body name="link1" pos="0 0 {p['post_h']}">
      <joint name="theta1" type="hinge" axis="0 0 1" damping="{p['j_damp']}"/>
      <inertial pos="0 0 0" mass="{M1}" diaginertia="{I1[0]} {I1[1]} {I1[2]}"/>
      <geom type="box" size="0.02 0.02 0.02"/>
      <body name="link2" pos="0 0 0">
        <joint name="theta2" type="hinge" axis="0 1 0" damping="{p['j_damp']}"/>
        <inertial pos="-0.25 0 0" mass="0.15" diaginertia="0.002 0.05 0.05"/>
        <geom type="cylinder" fromto="{p['cw_pos']} 0 0 {RBOOM} 0 0" size="0.008"/>
        <body name="cuerpo_cadera" pos="{RBOOM} 0 0">
          <inertial pos="0 0 0" mass="{p['body_hip']}" diaginertia="0.002 0.002 0.002"/>
          <geom type="box" size="0.03 0.03 0.03"/>
        </body>
        <body name="contrapeso" pos="{p['cw_pos']} 0 0">
          <inertial pos="0 0 0" mass="{p['cw_mass']}" diaginertia="0.003 0.003 0.003"/>
          <geom type="cylinder" fromto="0 -0.02 0 0 0.02 0" size="0.05"/>
        </body>
        <body name="link3" pos="{RBOOM} 0 0">
          <joint name="theta3" type="hinge" axis="0 1 0" armature="{ARM_H}"/>
          <inertial pos="0.048 0 0.077" mass="{M3}" diaginertia="{I3[0]} {I3[1]} {I3[2]}"/>
          <geom type="capsule" fromto="0 0 0 0 0 -{L3}" size="0.012"/>
          <body name="link4" pos="0 0 -{L3}">
            <joint name="theta4" type="hinge" axis="0 1 0" armature="{ARM_K}"
                   stiffness="{p['knee_stiff']}" springref="{p['spring_ref']}"/>
            <inertial pos="0.002 0.021 0.145" mass="{M4}" diaginertia="{I4[0]} {I4[1]} {I4[2]}"/>
            <geom type="capsule" fromto="0 0 0 0 0 -{L4}" size="0.010"/>
            <geom name="foot" class="contact" type="sphere" pos="0 0 -{L4}" size="0.012"/>
            <site name="foot_site" pos="0 0 -{L4}" size="0.005"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="hip" joint="theta3" gear="1" ctrlrange="-50 50"/>
    <motor name="knee" joint="theta4" gear="1" ctrlrange="-50 50"/>
  </actuator>
</mujoco>"""


def evaluate(user_params, t_total=6.0, return_log=False):
    p = dict(DEFAULTS); p.update(user_params or {})
    m = mujoco.MjModel.from_xml_string(make_xml(p))
    d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    qa = {n: m.jnt_qposadr[jid(n)] for n in ["theta1", "theta2", "theta3", "theta4"]}
    va = {n: m.jnt_dofadr[jid(n)] for n in ["theta1", "theta2", "theta3", "theta4"]}
    dof34 = [va["theta3"], va["theta4"]]
    hip_a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip")
    knee_a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee")
    fs = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    fg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot")
    hb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link3")
    bb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cuerpo_cadera")

    KPf = np.array([p['kp_x'], 0.0, p['kp_z']])
    KDf = np.array([p['kd'], 0.0, p['kd']])

    def foot_rel_hip():
        R = d.xmat[hb].reshape(3, 3)
        return R.T @ (d.site_xpos[fs] - d.xpos[hb])

    def foot_jac():
        J = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, J, None, fs)
        return J[:, dof34]

    def foot_force():
        f = 0.0
        for k in range(d.ncon):
            c = d.contact[k]
            if fg in (c.geom1, c.geom2):
                f6 = np.zeros(6); mujoco.mj_contactForce(m, d, k, f6); f += f6[0]
        return f

    d.qpos[qa["theta3"]], d.qpos[qa["theta4"]] = p['q3_ref'], p['q4_ref']
    mujoco.mj_forward(m, d)
    p_ref = foot_rel_hip().copy() + np.array([p['pref_dx'], 0.0, p['pref_dz']])

    qd_filt = np.zeros(2); q_prev = None
    phase = "stance"; t_td = 0.0; t_lo = -1.0
    apexes = []; cur_apex = -9.9
    body_z_log = []; t = 0.0
    nstep = int(t_total / DT)
    crashed = False
    for step in range(nstep):
        qd_real = np.array([d.qvel[dof34[0]], d.qvel[dof34[1]]])
        q = np.array([d.qpos[qa["theta3"]], d.qpos[qa["theta4"]]])
        if q_prev is None: q_prev = q.copy()
        a = p_ref  # placeholder
        raw = (q - q_prev) / DT
        af = 10.0 * DT / (1 + 10.0 * DT)
        qd_filt = qd_filt + af * (raw - qd_filt); q_prev = q.copy()
        Jc = foot_jac()
        R = d.xmat[hb].reshape(3, 3)
        err = R @ (p_ref - foot_rel_hip())
        Fair = KPf * err - KDf * (Jc @ qd_filt)
        tau_air = Jc.T @ Fair
        s = (t - t_td) / p['tst'] if phase == "stance" else 0.0
        Fz = bezier(FZ_BZ, s) * p['fz_scale']
        Fx = bezier(FX_BZ, s) * p['fx_scale']
        tau_st = Jc.T @ np.array([Fx, 0.0, -Fz])
        Vair = (Rw / (kT * N)) * tau_air + kv * N * qd_filt
        Vst = (Rw / (kT * N)) * tau_st + kv * N * qd_filt
        if phase == "stance":
            al = min(1.0, (t - t_td) / p['blend']); V = al * Vst + (1 - al) * Vair
        else:
            V = Vair
        V = np.clip(V, -VMAX, VMAX)
        i = np.clip((V - kv * N * qd_real) / Rw, -IMAX, IMAX)
        tau = kT * N * i
        d.ctrl[hip_a], d.ctrl[knee_a] = tau

        bz = d.xpos[bb][2]; body_z_log.append(bz)
        fz_site = d.site_xpos[fs][2]
        contact = (fz_site <= 0.014) or (foot_force() > 1.0)
        # FSM robusta: apoyo empuja hasta despegar o agotar tst; aereo sostiene
        # postura y reentra a apoyo al tocar (con debounce para no castanetear).
        if phase == "stance":
            if (t - t_td) >= p['tst'] or fz_site > 0.02:
                phase = "aerial"; t_lo = t; cur_apex = bz
        else:  # aerial
            cur_apex = max(cur_apex, bz)
            if contact and (t - t_lo) > 0.03:
                phase = "stance"; t_td = t
                apexes.append(cur_apex)
        mujoco.mj_step(m, d); t += DT
        if np.any(np.isnan(d.qpos)) or bz < -0.10:
            crashed = True; break

    body_z = np.array(body_z_log)
    apexes = np.array(apexes)
    n_hops = len(apexes)
    # score: saltos sostenidos, premiando consistencia, penalizando crash
    if n_hops >= 2 and not crashed:
        apex_std = float(np.std(apexes[1:])) if n_hops > 2 else 1.0
        consistency = 1.0 / (1.0 + 20 * apex_std)
        score = n_hops + 5 * consistency
    else:
        score = n_hops - (10 if crashed else 0)
    res = dict(score=float(score), n_hops=int(n_hops), crashed=bool(crashed),
               apex_mean=float(np.mean(apexes)) if n_hops else -9.9,
               apex_std=float(np.std(apexes[1:])) if n_hops > 2 else 9.9,
               body_z_min=float(body_z.min()), body_z_max=float(body_z.max()),
               t_end=float(t))
    if return_log:
        res["body_z"] = body_z.tolist()
    return res


if __name__ == "__main__":
    params = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    print(json.dumps(evaluate(params)))
