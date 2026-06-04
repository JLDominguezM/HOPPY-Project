"""Generador parametrico del MJCF de HOPPY + tuner de balance del gantry (M0).
Construye el modelo con variables (poste, masa cuerpo en cadera, contrapeso) y
busca el balance para que el robot descanse con el pie en el suelo y fuerza ligera.
"""
import mujoco, numpy as np

# Parametros fisicos (get_params.m + guia tecnica)
NH, NK, Ir = 26.9, 28.8, 7e-6
ARM_H, ARM_K = NH**2*Ir, NK**2*Ir   # inercia reflejada (armature)
RBOOM = 0.556
L3, L4 = 0.096, 0.1545              # muslo, pantorrilla
M1, M3, M4 = 0.268, 0.656, 0.149
I1 = (0.00115952, 0.00104649, 0.00030518)
I3 = (0.00082110, 0.00235762, 0.00168340)
I4 = (0.00039424, 0.00032191, 0.00010442)


def make_xml(post_h, body_hip, cw_mass, cw_pos, boom_mass=0.15):
    return f"""<mujoco model="hoppy">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
  <default>
    <joint damping="0"/>
    <geom contype="0" conaffinity="0"/>
    <default class="contact">
      <geom contype="1" conaffinity="1" solref="0.002 1" solimp="0.95 0.99 0.001"
            friction="2.0 0.1 0.1"/>
    </default>
  </default>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    <geom name="floor" class="contact" type="plane" size="3 3 0.1" rgba="0.8 0.8 0.8 1"/>
    <geom name="post" type="cylinder" fromto="0 0 0 0 0 {post_h}" size="0.02" rgba="0.2 0.2 0.2 1"/>
    <body name="link1" pos="0 0 {post_h}">
      <joint name="theta1" type="hinge" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="{M1}" diaginertia="{I1[0]} {I1[1]} {I1[2]}"/>
      <geom type="box" size="0.02 0.02 0.02" rgba="0.4 0.4 0.8 1"/>
      <body name="link2" pos="0 0 0">
        <joint name="theta2" type="hinge" axis="0 1 0"/>
        <inertial pos="-0.25 0 0" mass="{boom_mass}" diaginertia="0.002 0.05 0.05"/>
        <geom type="cylinder" fromto="{cw_pos} 0 0 {RBOOM} 0 0" size="0.008" rgba="0.7 0.7 0.2 1"/>
        <body name="cuerpo_cadera" pos="{RBOOM} 0 0">
          <inertial pos="0 0 0" mass="{body_hip}" diaginertia="0.002 0.002 0.002"/>
          <geom type="box" size="0.03 0.03 0.03" rgba="0.9 0.7 0.1 1"/>
        </body>
        <body name="contrapeso" pos="{cw_pos} 0 0">
          <inertial pos="0 0 0" mass="{cw_mass}" diaginertia="0.003 0.003 0.003"/>
          <geom type="cylinder" fromto="0 -0.02 0 0 0.02 0" size="0.05" rgba="0.5 0.5 0.5 1"/>
        </body>
        <body name="link3" pos="{RBOOM} 0 0">
          <joint name="theta3" type="hinge" axis="0 1 0" armature="{ARM_H}"/>
          <inertial pos="0.048 0 0.077" mass="{M3}" diaginertia="{I3[0]} {I3[1]} {I3[2]}"/>
          <geom type="capsule" fromto="0 0 0 0 0 -{L3}" size="0.012" rgba="0.8 0.3 0.3 1"/>
          <body name="link4" pos="0 0 -{L3}">
            <joint name="theta4" type="hinge" axis="0 1 0" armature="{ARM_K}"
                   stiffness="3.0" springref="-1.5708"/>
            <inertial pos="0.002 0.021 0.145" mass="{M4}" diaginertia="{I4[0]} {I4[1]} {I4[2]}"/>
            <geom type="capsule" fromto="0 0 0 0 0 -{L4}" size="0.010" rgba="0.3 0.3 0.8 1"/>
            <geom name="foot" class="contact" type="sphere" pos="0 0 -{L4}" size="0.012" rgba="0.1 0.1 0.1 1"/>
            <site name="foot_site" pos="0 0 -{L4}" size="0.005"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="hip"  joint="theta3" gear="1" ctrlrange="-100 100"/>
    <motor name="knee" joint="theta4" gear="1" ctrlrange="-100 100"/>
  </actuator>
  <sensor>
    <framepos name="foot_pos" objtype="site" objname="foot_site"/>
    <touch name="foot_touch" site="foot_site"/>
  </sensor>
</mujoco>"""


def settle(xml, q3d=np.pi/3, q4d=-np.pi/2, steps=4000):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    ja = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    da = lambda n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    hid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip")
    kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee")
    sf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    d.qpos[ja("theta3")], d.qpos[ja("theta4")] = q3d, q4d
    for _ in range(steps):
        d.ctrl[hid] = 80*(q3d - d.qpos[ja("theta3")]) - 3*d.qvel[da("theta3")]
        d.ctrl[kid] = 80*(q4d - d.qpos[ja("theta4")]) - 3*d.qvel[da("theta4")]
        mujoco.mj_step(m, d)
    fz = 0.0
    for i in range(d.ncon):
        f6 = np.zeros(6); mujoco.mj_contactForce(m, d, i, f6); fz += f6[0]
    return dict(theta2=d.qpos[ja("theta2")], foot_z=d.site_xpos[sf][2],
                ncon=d.ncon, fz=fz, mass=sum(m.body_mass),
                nan=bool(np.any(np.isnan(d.qpos))))


if __name__ == "__main__":
    print("=== Tuning del balance (M0): pie en suelo con fuerza ligera positiva ===")
    print(f"{'post':>5} {'cuerpo':>6} {'cw':>5} {'cw_pos':>6} | {'theta2':>7} {'foot_z':>7} {'ncon':>4} {'Fz(N)':>7} {'masa':>5}")
    for post_h in [0.20, 0.22, 0.24]:
        for cw_mass in [0.8, 1.2, 1.6, 2.0]:
            r = settle(make_xml(post_h, 1.2, cw_mass, -0.45))
            print(f"{post_h:>5} {1.2:>6} {cw_mass:>5} {-0.45:>6} | {r['theta2']:>7.3f} {r['foot_z']:>7.4f} {r['ncon']:>4} {r['fz']:>7.2f} {r['mass']:>5.2f}")
