"""hoppy_urdf.py - the real HOPPY URDF as a module compatible with controller.py.

Takes the imported URDF (load_hoppy_urdf), renames joint1..4 -> theta1..4 (so
controller.py finds them unchanged) and INJECTS what the hybrid controller needs
and the URDF does not provide:
  - floor (class "contact")
  - geom 'foot' + site 'foot_site' at the real shank tip of Link4 (measured)
  - body 'cuerpo_cadera' = boom frame (Link2) at the hip, for the Cartesian PD
  - hip(theta3)/knee(theta4) torque actuators
  - knee spring, armatures N^2*Ir and damping (back-EMF) as in twin
  - implicitfast option (stable with contact; twin uses this, NOT RK4)

The frame is compatible: the leg swing axis lies on X of the boom frame
(checked: R2.T @ axis = [-1,0,0]), which is what controller.py assumes. Real URDF
masses = 3.43 kg (vs 2.58 for the twin), so the force gains are scaled ~1.33 as a
first guess; the fine gait is re-tuned separately.

Reuses goBILDA motors + Bezier + twin constants. Does not modify controller.py,
load_hoppy_urdf.py, twin.py or verify.py.
"""
import os
import re
import numpy as np
import mujoco

import load_hoppy_urdf as L
import twin
# names controller.py reads as attributes of the model module:
from twin import (bezier, FZ_BZ, FX_BZ, NH, NK, Rw, kT, kv, VMAX, IMAX, N, DT,
                  ARM_H, ARM_K, DAMP_H, DAMP_K)

RBOOM = 0.661                              # hip -> URDF yaw axis (measured)
FOOT_TIP = (0.0227, -0.1824, -0.0274)     # shank tip in Link4-local coords (measured)
MASS_RATIO = 3.43 / 2.58                   # ~1.33  (URDF vs twin)
HIP_POS_L2 = "-0.6585 0 0.0425"            # hip in the Link2 frame (Link3 pos without its quat)

# DEFAULTS = twin with the force gains scaled by the mass ratio (first guess)
DEFAULTS = dict(twin.DEFAULTS)
for _k in ("kp_sw", "kd_sw", "kp_st", "kd_st", "fz_scale", "fx_scale", "knee_stiff"):
    DEFAULTS[_k] = DEFAULTS[_k] * MASS_RATIO

# ---------------------------------------------------------------------------------------
# FORWARD: the REAL hybrid controller (controller.py = faithful port of the MATLAB/paper)
# with the REAL constants of the MATLAB simulator + a boom BALANCED with a counterweight
# (like the paper's HOPPY; the MATLAB model puts the boom CoM at rx2=-0.50). This is the
# control that runs on the LaunchPad F28379D (cpu01_main), NOT the simple FSM of
# hop_controller.
#
# Result: the URDF hops FORWARD = -theta1 (the side OPPOSITE the leg offset, the "no-leg"
# side), ~0.55 rad/s, the foot clears ~4.5 cm, ~63% flight, stable limit cycle,
# V<=12 / i<=9.2 A (like the MATLAB). The direction is set by the tangential stance push
# (GRF Bezier Fx peak=-25, the real MATLAB value); it matches the MATLAB direction.
#
# Real constants (= get_params.m / firmware cpu01_main):
#   kp_sw=150, kd_sw=5  (Cartesian flight PD; firmware Kp_a/Kd_a)
#   krh=0.10            (Raibert foot placement)            p_toe_z=-0.15 (foot 15 cm below hip)
#   Tst=0.35           (nominal stance time)                grf_liftoff=1.5 (liftoff threshold)
#   Fz_bz peak=100, Fx_bz peak=-25 (Bezier stance GRF profile)   kp_st=0.03 kd_st=0.08 (soft PD)
#   goBILDA motor model: Rw=1.3 kT=0.0135 kv=0.0186 Nh=26.9 Nk=28.8, sat 12 V / 9.2 A
FORWARD = dict(
    DEFAULTS,
    kp_sw=150.0, kd_sw=5.0, krh=0.10, p_toe_z=-0.18, Tst=0.35,  # p_toe_z=-0.18: lands more extended (leg more vertical)
    kp_st=0.03, kd_st=0.08, fz_scale=1.0, fx_scale=1.0, grf_liftoff=1.5,
    q3_ref=0.55, q4_ref=-0.95,        # valid crouch in the URDF joint space
    cw_mass=2.86, cw_x=0.35,          # counterweight: REAL physical boom arm (35 cm from the pivot),
                                      # mass by MOMENT BALANCE at ~76% (leaves effective weight on
                                      # the leg; a 100% balance = 3.77 kg overshoots). See counterweight.py
    vx_d=0.0,                         # desired Raibert velocity = 0 (pure MATLAB; Fx sets the travel)
)

_BASE = None


def _base_mjcf():
    """Base MJCF of the URDF (kinematics + inertia + meshes), cached."""
    global _BASE
    if _BASE is None:
        m0 = L.build()
        path = os.path.join(L.PKG, "urdf", "HOPPY-E0-final.mjcf.xml")
        mujoco.mj_saveLastXML(path, m0)
        _BASE = open(path).read()
    return _BASE


def make_xml(p):
    xml = _base_mjcf()
    jd = p.get("j_damp", 0.0)
    ks, kr = p.get("knee_stiff", 0.0), p.get("knee_ref", 0.0)
    kdmp = DAMP_K + p.get("knee_damp", 0.0)
    sol = p.get("solref0", 0.0191)
    fx, fy, fz = FOOT_TIP

    # 1) joint1..4 -> theta1..4
    for i in (1, 2, 3, 4):
        xml = xml.replace('name="joint%d"' % i, 'name="theta%d"' % i)

    # 2) augment the joints (armature, damping, knee spring)
    xml = xml.replace(
        '<joint name="theta1" pos="0 0 0" axis="0 0 1" actuatorfrcrange="-10 10"/>',
        '<joint name="theta1" pos="0 0 0" axis="0 0 1" damping="%g"/>' % jd)
    xml = xml.replace(
        '<joint name="theta2" pos="0 0 0" axis="0 0 1" range="-0.5 0.5" actuatorfrcrange="-10 10"/>',
        '<joint name="theta2" pos="0 0 0" axis="0 0 1" range="-0.8 0.8" damping="%g"/>' % jd)
    xml = xml.replace(
        '<joint name="theta3" pos="0 0 0" axis="0 0 1" range="-0.5 0.9" actuatorfrcrange="-10 10"/>',
        '<joint name="theta3" pos="0 0 0" axis="0 0 1" range="-0.5 0.9" armature="%g" damping="%g"/>'
        % (ARM_H, DAMP_H))
    # foot: the leg stays IDENTICAL to the URDF (CAD mesh). Contact is the shank tip
    # (FOOT_TIP). Physically it is a point at that tip.
    foot = ('<site name="foot_site" pos="%g %g %g" size="0.03" rgba="1 0 0 0"/>'
            '<geom name="foot" class="contact" type="sphere" size="0.016" pos="%g %g %g" '
            'group="3" rgba="0.9 0.6 0.1 1"/>' % (fx, fy, fz, fx, fy, fz))
    xml = xml.replace(
        '<joint name="theta4" pos="0 0 0" axis="0 0 -1" range="-1.3 0.4" actuatorfrcrange="-10 10"/>',
        '<joint name="theta4" pos="0 0 0" axis="0 0 -1" range="-1.3 0.4" armature="%g" '
        'stiffness="%g" springref="%g" damping="%g"/>' % (ARM_K, ks, kr, kdmp) + foot)

    # 3) option + contact defaults (after <compiler/>); implicitfast for stability
    head = ('<option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>'
            '<default><geom contype="0" conaffinity="0"/>'
            '<default class="contact"><geom contype="1" conaffinity="1" solref="%g 1" '
            'solimp="0.95 0.99 0.001" friction="2.0 0.1 0.1"/></default></default>' % sol)
    xml = re.sub(r'(<compiler[^>]*/>)', lambda mm: mm.group(1) + "\n  " + head, xml, count=1)

    # 4) floor (grid: uses the 'grid' material injected at the end; visual ONLY.
    #    The physics/friction stays in class="contact" -> untouched).
    xml = xml.replace(
        "<worldbody>",
        '<worldbody>\n    <geom name="floor" class="contact" type="plane" size="3 3 0.1" '
        'material="grid"/>', 1)

    # 5) cuerpo_cadera (boom frame at the hip) as a child of Link2, before Link3.
    #    + optional boom COUNTERWEIGHT: the real HOPPY (and the MATLAB model, rx2=-0.50)
    #    carry a mass on the side OPPOSITE the hopper (+x local of Link2) to balance pitch.
    #    Without it the boom CoM sits 0.55 m toward the hopper (~13 N*m of gravity) and the
    #    robot only bobs without taking off. cw_mass/cw_x reproduce it (faithful to the paper).
    cw_m, cw_x = p.get("cw_mass", 0.0), p.get("cw_x", 0.65)
    cw = ""
    if cw_m > 0:
        cw = ('<body name="cw" pos="%g 0 0"><inertial pos="0 0 0" mass="%g" '
              'diaginertia="%g %g %g"/><geom type="cylinder" fromto="-0.03 0 0 0.03 0 0" '
              'size="%g" rgba="0.7 0.2 0.2 1"/></body>'
              % (cw_x, cw_m, cw_m*0.002, cw_m*0.002, cw_m*0.002, 0.03+0.012*cw_m))
    xml = xml.replace(
        '<body name="Link3"',
        cw + '<body name="cuerpo_cadera" pos="%s" quat="0 0 0.70711 0.70711"><inertial pos="0 0 0" '
        'mass="1e-6" diaginertia="1e-9 1e-9 1e-9"/></body>\n          <body name="Link3"' % HIP_POS_L2, 1)

    # 6) contact sensor + torque actuators
    hg, kg = p.get("hip_gear", 1.0), p.get("knee_gear", 1.0)   # actuator sign (URDF knee axis is flipped)
    xml = xml.replace(
        "</mujoco>",
        '<sensor><touch name="foot_touch" site="foot_site"/></sensor>'
        '<actuator><motor name="hip" joint="theta3" gear="%g" ctrlrange="-5 5"/>'
        '<motor name="knee" joint="theta4" gear="%g" ctrlrange="-5 5"/></actuator></mujoco>' % (hg, kg))

    # fast mode (tuner): drops the VISUAL meshes so it loads instantly. The dynamics are
    # IDENTICAL: inertias are explicit on each body and contact is the 'foot' sphere.
    if p.get("fast"):
        xml = re.sub(r'<geom type="mesh"[^>]*/>', '', xml)
        xml = re.sub(r'<asset>.*?</asset>', '<asset/>', xml, flags=re.DOTALL)

    # floor checker texture (MuJoCo builtin checker). Injected ALWAYS, also in fast mode
    # (which empties the <asset>), so the floor's 'grid' material exists. It is PURELY
    # visual: it does not touch contype/conaffinity/solref/friction (those live in class="contact").
    grid = ('<texture name="grid" type="2d" builtin="checker" rgb1="0.18 0.22 0.28" '
            'rgb2="0.28 0.33 0.40" width="512" height="512"/>'
            '<material name="grid" texture="grid" texrepeat="4 4" texuniform="true" reflectance="0.2"/>')
    if "<asset/>" in xml:
        xml = xml.replace("<asset/>", "<asset>" + grid + "</asset>", 1)
    elif "<asset>" in xml:
        xml = xml.replace("<asset>", "<asset>" + grid, 1)
    return xml


if __name__ == "__main__":
    m = mujoco.MjModel.from_xml_string(make_xml(DEFAULTS))
    print("compiles OK | nq=%d nu=%d nsite=%d nsensor=%d ngeom=%d" %
          (m.nq, m.nu, m.nsite, m.nsensor, m.ngeom))
    for nm in ("theta1", "theta2", "theta3", "theta4", "foot_site", "foot", "cuerpo_cadera", "hip", "knee"):
        for obj in (mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_SITE,
                    mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_ACTUATOR):
            if mujoco.mj_name2id(m, obj, nm) >= 0:
                print("  OK exists:", nm); break
        else:
            print("  MISSING:", nm)
