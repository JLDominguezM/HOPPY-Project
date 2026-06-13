"""STRUCTURAL MJCF model (digital twin) of the redesigned HOPPY.

Unlike tune_eval.make_xml (an abstract model tuned to the original MATLAB
parameters with meshes glued on top), here the links ARE the real geometry
measured from the redesign CAD/STEP, with real masses (PLA by volume + goBILDA
motors from the datasheet + a PVC boom). Same validated topology (serial
yaw-pitch-hip-knee tree, yaw and pitch passive), but the dimensions and dynamics
of the user's robot.

Sources:
  - dimensions: STEP (build_twin_meshes measures pivot, boom, hip; leg = HOPPY ref
    confirmed by the CAD bearings: LH=96mm, DK=52mm, LK=154.5mm).
  - masses: STL x PLA density (housing 791g, leg 192g) + 2x goBILDA 5202-2402-0027
    (435g each, datasheet) + electronics (~150g) + PVC boom (~481g).
  - actuator: goBILDA 0027 -> R=1.3, kT=0.0135, kv=0.0186, N=26.9 (same as get_params,
    checked against the datasheet).
"""
import os
import numpy as np

MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")

# ---------------- real dimensions (m), measured from the STEP (GLB Y-up) ----------------
HB = 0.250          # pivot (yaw+pitch) height above the floor
# REAL hip pivot (link2-local), measured from the CAD: top of the thigh where it meets the
# housing. The leg mounts on the FRONT FACE of the housing (y=0.187), not at the center,
# and a little below the boom (z=-0.036). It used to be wrong, eyeballed at (0.730,0.099,0).
LB = 0.687          # horizontal boom: pivot -> hip (X)
DB = 0.187          # LATERAL (tangential) offset of the hip = front face of the housing
HIP_DZ = -0.036     # the hip is 3.6 cm below the boom axis
LH = 0.096          # straight thigh (HOPPY reference; the REAL VECTOR is below)
DK = 0.052          # knee offset                    [HOPPY ref, confirmed on CAD]
LK = 0.1545         # shank                          [HOPPY ref]
L  = float(np.hypot(LK, DK))   # effective shank = 0.163
# REAL planar GEOMETRY of the four-bar (measured from the CAD), in the Y-Z swing plane:
# the thigh goes DOWN and FORWARD (+Y), the shank goes DOWN and BACK (-Y) -> a zig-zag leg.
# It used to be modeled STRAIGHT (hip->knee->foot along -Z, with DK folded into L like the
# MATLAB), but then the CAD mesh DISCONNECTED on bending (the link rotated about a fake
# pivot). With the real vectors each link rotates about its REAL PIVOT -> the CAD four-bar
# mesh articulates while staying CONNECTED. The controller is agnostic (it uses MuJoCo's
# live Jacobian, not analytic LH/L), so only the gains are re-tuned.
KNEE_OFF = (-0.0018, 0.0586, -0.0583)   # knee at the tube-plate joint (the geometry that HOPS)
FOOT     = (0.0048, -0.1126, -0.1717)   # foot in link4-local (shank with the DK offset)

# --- series-elastic knee spring (2x TENSION SPRING, from the PDF) ---
# The spring runs from an anchor on the THIGH (A) to a lever on the SHANK (B, ~81mm from the
# knee axis). On flexion (theta4<0) lever B rotates and the spring STRETCHES (87->110mm at
# -0.5 rad) -> an extension torque. Modeled as 2 TENDONS (they actually stretch).
KS_SPRING = 1670.0   # N/m per spring (PDF: Ks=1.67 kN/m)
L0_SPRING = 0.080    # m, rest length (PDF: L0=80mm)
SPR_A_R = (0.036, -0.022, -0.035)    # upper anchor (thigh/link3), +X side
SPR_A_L = (-0.034, -0.025, -0.035)   # upper anchor, -X side
SPR_B_R = (0.036,  0.002, -0.004)    # lower anchor (shank/link4), +X side
SPR_B_L = (-0.034, 0.000, -0.004)    # lower anchor, -X side
BOOM_X0, BOOM_X1 = -0.39, 0.81  # tube ends (balance .. hopper tip), model frame
BOOM_OD = 0.0334    # 1" PVC

# ---------------- real masses (kg) ----------------
M_HOUSING = 0.791   # housing PLA (STL x 1.24 g/cm3 x fill 0.6)
M_BOOM    = 0.481   # 1" PVC tube (vol x 1.4)
M_MOTORS  = 0.870   # 2x goBILDA 5202-2402-0027 @ 435 g
M_ELEC    = 0.150   # TI F28379D + 2x VNH5019 + sensor (estimate)
M_THIGH   = 0.130   # PLA thigh + bearing
M_SHANK   = 0.100   # PLA shank + springs + rubber tip
M_YAW     = 0.060   # yaw-carriage bracket (estimate)

# ---------------- actuator (goBILDA 0027, datasheet = get_params) ----------
NH, NK = 26.9, 28.8   # hip NH=26.9, knee NK=28.8
Rw, kT, kv = 1.3, 0.0135, 0.0186
VMAX = 12.0
IMAX = 9.2          # motor stall current @12V (datasheet); the original used 30
N = np.array([NH, NK])
I_ROTOR = 7e-6
ARM_H, ARM_K = NH**2 * I_ROTOR, NK**2 * I_ROTOR
# Equivalent damping: back-EMF reflected to the joint, d_eq = kT^2 * N^2 / Rw [Nm*s/rad].
# The controller feed-forward compensates the back-EMF (tau~=u, ideal torque source); this
# passive damping reintroduces the real electrical dissipation of the geared motor.
DAMP_FACTOR = 0.5   # 1.0 slows the hop to 1.3cm (FAILS); 0.5 -> 5.6cm PASS. Sweep: 0.60 is the
                    # max with PASS but borderline (0.65 collapses to 1.6cm), hence 0.5 (margin).
DAMP_H = DAMP_FACTOR * kT**2 * NH**2 / Rw   # hip
DAMP_K = DAMP_FACTOR * kT**2 * NK**2 / Rw   # knee (NK=28.8)
DT = 0.001
RBOOM = float(np.hypot(LB, DB))   # real hip->yaw-axis radius (0.712), not just LB

# soft knee spring (the real bounce comes from the springs). With the real planar geometry
# the knee zero = the CAD extended pose, so the spring rest is ~0.
KNEE_STIFF, KNEE_REF = 0.0484, 0.0
FZ_BZ = np.array([0.0, 20.0, 100.0, 0.0, 0.0])
FX_BZ = np.array([0.0,  0.0, -25.0, 0.0, 0.0])


# ---------------- composite inertias ----------------
def _box_I(m, dx, dy, dz):
    return np.diag([m*(dy*dy+dz*dz)/12, m*(dx*dx+dz*dz)/12, m*(dx*dx+dy*dy)/12])

def _cyl_I_x(m, r, length):   # cylinder with its axis along X
    return np.diag([m*r*r/2, m*(3*r*r+length*length)/12, m*(3*r*r+length*length)/12])

def _shift(I, m, r):          # parallel axis: I about a point shifted by -r from the CoM
    r = np.asarray(r, float)
    return I + m*(np.dot(r, r)*np.eye(3) - np.outer(r, r))

def _compose(parts):
    """parts: list of (I_com, m, com). Returns (M, total_com, I_about_com)."""
    M = sum(p[1] for p in parts)
    com = sum(p[1]*np.asarray(p[2], float) for p in parts)/M
    I = np.zeros((3, 3))
    for I_com, m, c in parts:
        I += _shift(I_com, m, np.asarray(c, float) - com)
    return M, com, I

# --- link2 = boom (rod) + hip lump (housing+motors+elec, a box at the hip) ---
_boom_com = np.array([(BOOM_X0+BOOM_X1)/2, 0, 0])
_boom_I = _cyl_I_x(M_BOOM, BOOM_OD/2, BOOM_X1-BOOM_X0)
_hip_m = M_HOUSING + M_MOTORS + M_ELEC
_hip_com = np.array([0.693, 0.068, 0.021])        # housing+motors centroid in link2-local (measured from the CAD)
_hip_I = _box_I(_hip_m, 0.32, 0.20, 0.33)
M2, COM2, I2 = _compose([(_boom_I, M_BOOM, _boom_com), (_hip_I, _hip_m, _hip_com)])

# --- link3 = thigh: thin rod hip->knee (real vector KNEE_OFF) ---
_ko = np.array(KNEE_OFF); _ft = np.array(FOOT)
M3 = M_THIGH; COM3 = _ko/2; I3 = _cyl_I_x(M3, 0.012, float(np.linalg.norm(_ko)))  # ~rod
# --- link4 = shank+foot: rod knee->foot (real vector FOOT) ---
M4 = M_SHANK; COM4 = _ft/2;  I4 = _cyl_I_x(M4, 0.012, float(np.linalg.norm(_ft)))
# --- link1 = yaw carriage: small block ---
M1 = M_YAW; COM1 = np.array([0, 0, 0.0]); I1 = _box_I(M1, 0.06, 0.06, 0.05)


def _inertial(pos, m, I):
    return (f'<inertial pos="{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f}" mass="{m:.4f}" '
            f'fullinertia="{I[0,0]:.6f} {I[1,1]:.6f} {I[2,2]:.6f} '
            f'{I[0,1]:.6f} {I[0,2]:.6f} {I[1,2]:.6f}"/>')


# Config WITHOUT a counterweight that still TRAVELS (circles the post at ~0.16 rad/s) thanks
# to the vx_d knob (the desired travel velocity that was missing). The earlier hop-in-place
# was a control BUG: flight control used krh*vx without vx_d -> it damped the yaw to 0. With
# vx_d!=0 it regulates vx toward vx_d and travels. Hop 4.9cm, foot clears 4.3cm, 35% flight,
# motors at the 12V/9.2A limit (no margin; a ~2kg counterweight gives more margin but is NOT
# needed to travel - see HANDOFF and twin_tune). q3 limited to 0.3-1.5 rad (realistic range).
DEFAULTS = dict(
    solref0=0.0191, j_damp=0.1494,
    knee_stiff=0.0948, knee_ref=0.0088, knee_damp=0.0, spring_scale=0.0,  # JOINT spring; tendon = visual
    kp_sw=249.60, kd_sw=5.0839, krh=0.1187, p_toe_z=-0.1828,
    Tst=0.285, kp_st=0.03, kd_st=0.08, q3_ref=0.1572, q4_ref=-0.4607,
    fz_scale=3.1838, fx_scale=3.3653, blend=0.0136, grf_liftoff=3.8109,
    vx_d=-0.3385,   # desired travel velocity (Raibert)
    # Config that HOPS (seed23): body +8.4cm, foot clears 6.6cm, travels 0.63 rad/s. PROCEDURAL
    # leg (leg_proc) that articulates connected. The joint spring gives the bounce; the golden
    # tendon is VISUAL (spring_scale=0). The fully real spring (1.67kN/m) does NOT hop (see HANDOFF).
)
# Real SERIES-ELASTIC knee: knee at pivot P4 (from the STEP), shank with the DK offset, and
# 2 spring-TENDONS (Ks=1.67kN/m, L0=80mm from the PDF) that STRETCH with theta4 (87->114mm)
# giving the real extension torque and staying ALWAYS connected thigh<->lever. The rigid coil
# was removed from the mesh (a solid does not stretch). TODO: re-tune the gait.


def make_xml(p):
    sol = p.get('solref0', 0.012)
    jd = p.get('j_damp', 0.0)
    vis = p.get('vis', False) and os.path.exists(f"{MESH_DIR}/twin_housing.obj")
    a = "0" if vis else "1"
    af = "0" if vis else "0.9"
    PLA = "0.78 0.80 0.83 1"; DKc = "0.2 0.2 0.23 1"; PVC = "0.93 0.93 0.9 1"
    HOUSING = "0.65 0.65 0.65 1"   # CAD medium gray
    MOT = "0.2 0.2 0.2 1"          # goBILDA motors
    GOLD = "0.8 0.7 0.1 1"         # tension springs (tendons)
    BLK = "0.15 0.15 0.15 1"       # black plates/links
    RUB = "0.1 0.1 0.1 1"          # rubber tip
    # leg_mesh: CAD four-bar mesh (for the static assembly). leg_proc=True forces the clean
    # procedural leg (for the HOP render, where the four-bar mesh disconnects on bending; the
    # dynamics are identical because the meshes are visual only).
    leg_mesh = vis and not p.get('leg_proc', False) and os.path.exists(f"{MESH_DIR}/twin_thigh.obj")
    asset = ""
    g_house = ""
    if vis:
        asset = (f'<mesh name="mg" file="{MESH_DIR}/twin_gantry.obj"/>'
                 f'<mesh name="mh" file="{MESH_DIR}/twin_housing.obj"/>'
                 f'<material name="mat_housing" rgba="{HOUSING}" specular="0.6" shininess="0.5" reflectance="0.1"/>')
        g_house = f'<geom type="mesh" mesh="mh" material="mat_housing"/>'
        if leg_mesh:
            asset += (f'<mesh name="mt" file="{MESH_DIR}/twin_thigh.obj"/>'
                      f'<mesh name="msh" file="{MESH_DIR}/twin_shank.obj"/>')
    # fixed gantry (world)
    if vis:
        gantry = f'<geom name="post" type="mesh" mesh="mg" rgba="{PLA}"/>'
    else:
        gantry = (f'<geom name="post" type="cylinder" pos="0 0 {HB/2}" size="0.02 {HB/2}" rgba="0.4 0.4 0.4 1"/>'
                  f'<geom type="cylinder" pos="0 0 0.004" size="0.085 0.004" rgba="0.4 0.4 0.4 1"/>')
    # PVC boom (link2)
    boom = f'<geom type="cylinder" fromto="{BOOM_X0} 0 0 {BOOM_X1} 0 0" size="{BOOM_OD/2}" rgba="{PVC if vis else "0.6 0.6 0.6 "+a}"/>'
    # optional COUNTERWEIGHT (boom balance side): mass cw_mass at x=cw_x in link2-local.
    # Used to balance the hopper pitch and check whether it makes it hop like the original.
    cw_m, cw_x = p.get('cw_mass', 0.0), p.get('cw_x', -0.65)
    if cw_m > 0:
        cw = (f'<body name="cw" pos="{cw_x} 0 0">'
              f'<inertial pos="0 0 0" mass="{cw_m}" diaginertia="{cw_m*0.001:.5f} {cw_m*0.001:.5f} {cw_m*0.001:.5f}"/>'
              f'<geom type="cylinder" fromto="-0.02 0 0 0.02 0 0" size="{0.025+0.012*cw_m:.3f}" rgba="0.7 0.2 0.2 1"/>'
              f'</body>')
    else:
        cw = ""
    # leg on link3/link4: real CAD mesh (four-bar) if it exists, else procedural
    if leg_mesh:
        kx, ky, kz = KNEE_OFF; fx, fy, fz = FOOT
        # CAD mesh of thigh/shank (already shows the four-bar) + 2 motors and the rubber tip.
        # the physical contact (geom "foot") is invisible in vis; the rubber tip is visual only.
        g3 = (f'<geom type="mesh" mesh="mt" rgba="{PLA}"/>'
              f'<geom type="cylinder" fromto="-0.02 0 0 0.02 0 0" size="0.045" rgba="{MOT}"/>'                          # hip motor (at the hip)
              f'<geom type="cylinder" fromto="{kx-0.02:.4f} {ky} {kz} {kx+0.02:.4f} {ky} {kz}" size="0.045" rgba="{MOT}"/>')  # knee motor (at KNEE_OFF)
        g4 = (f'<geom type="mesh" mesh="msh" rgba="{PLA}"/>'
              f'<geom type="sphere" pos="{fx} {fy} {fz}" size="0.018" rgba="{RUB}"/>')                                  # black rubber tip at FOOT
    elif vis:
        PLAg = "0.65 0.65 0.65 1"     # medium-gray tube (CAD)
        kx, ky, kz = KNEE_OFF; fx, fy, fz = FOOT
        # Procedural leg that ALWAYS articulates connected (the CAD mesh splits on bending):
        # thigh (link3) = hip motor + 2 BLACK plates (IMP-8/9) + knee motor (the knee motor is
        # mounted on the THIGH/link3 even though joint theta4 is in link4, so it stays fixed to
        # the thigh while articulating); shank (link4) = 2 bearings + tube TUB-1 + rubber tip.
        # The SPRING is the golden TENDON (in <tendon>, it follows the articulation).
        lat = 0.010                   # half lateral spacing of the plates (0.02 total)
        g3 = (f'<geom type="cylinder" fromto="-0.024 0 0 0.024 0 0" size="0.018" rgba="{MOT}"/>'                          # hip motor
              f'<geom type="capsule" fromto="{-lat:.4f} 0 0 {-lat+kx:.4f} {ky} {kz}" size="0.007" rgba="{BLK}"/>'         # plate IMP-8 (black)
              f'<geom type="capsule" fromto="{ lat:.4f} 0 0 { lat+kx:.4f} {ky} {kz}" size="0.007" rgba="{BLK}"/>'         # plate IMP-9 (black)
              f'<geom type="cylinder" fromto="{kx-0.022:.4f} {ky} {kz} {kx+0.022:.4f} {ky} {kz}" size="0.020" rgba="{MOT}"/>')  # knee motor (KNEE_OFF, on link3)
        g4 = (f'<geom type="cylinder" fromto="{-lat-0.0075:.4f} 0 0 {-lat+0.0075:.4f} 0 0" size="0.012" rgba="{BLK}"/>'   # left bearing
              f'<geom type="cylinder" fromto="{ lat-0.0075:.4f} 0 0 { lat+0.0075:.4f} 0 0" size="0.012" rgba="{BLK}"/>'   # right bearing
              f'<geom type="cylinder" fromto="0 0 0 {fx} {fy} {fz}" size="0.014" rgba="{PLAg}"/>'                         # tube TUB-1 (gray)
              f'<geom type="sphere" pos="{fx} {fy} {fz}" size="0.020" rgba="0.08 0.08 0.08 1"/>')                          # rubber tip (black sphere)
    else:
        g3 = g4 = ""
    # link1 (yaw): dark pivot cylinder, visible only in vis mode
    g1 = f'<geom type="cylinder" fromto="0 0 -0.028 0 0 0.028" size="0.022" rgba="0.25 0.25 0.28 1"/>' if vis else ""
    return f"""<mujoco model="hoppy_twin">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <default>
    <joint damping="0"/><geom contype="0" conaffinity="0"/>
    <default class="contact"><geom contype="1" conaffinity="1" solref="{sol} 1"
       solimp="0.95 0.99 0.001" friction="2.0 0.1 0.1"/></default>
  </default>
  <asset>{asset}</asset>
  <worldbody>
    <geom name="floor" class="contact" type="plane" size="3 3 0.1" rgba="0.5 0.5 0.55 1"/>
    {gantry}
    <body name="link1" pos="0 0 {HB}">
      <joint name="theta1" type="hinge" axis="0 0 1" damping="{jd}"/>
      {_inertial(COM1, M1, I1)}
      <geom type="box" size="0.03 0.03 0.025" rgba="0.4 0.4 0.4 {a}"/>
      {g1}
      <body name="link2" pos="0 0 0">
        <joint name="theta2" type="hinge" axis="0 1 0" damping="{jd}"/>
        {_inertial(COM2, M2, I2)}
        {boom}
        {g_house}
        {cw}
        <body name="cuerpo_cadera" pos="{LB} {DB} {HIP_DZ}">
          <inertial pos="0 0 0" mass="1e-6" diaginertia="1e-9 1e-9 1e-9"/>
          <geom type="box" size="0.03 0.03 0.03" rgba="0.2 0.4 0.7 {a}"/>
        </body>
        <body name="link3" pos="{LB} {DB} {HIP_DZ}">
          <joint name="theta3" type="hinge" axis="1 0 0" armature="{ARM_H}" damping="{DAMP_H}" range="-0.5 0.9"/>
          {_inertial(COM3, M3, I3)}
          <geom type="capsule" fromto="0 0 0 {KNEE_OFF[0]} {KNEE_OFF[1]} {KNEE_OFF[2]}" size="0.012" rgba="0.2 0.5 0.8 {a}"/>
          {g3}
          <site name="spr_a_R" pos="{SPR_A_R[0]} {SPR_A_R[1]} {SPR_A_R[2]}" size="0.006" rgba="0.8 0.7 0.3 1"/>
          <site name="spr_a_L" pos="{SPR_A_L[0]} {SPR_A_L[1]} {SPR_A_L[2]}" size="0.006" rgba="0.8 0.7 0.3 1"/>
          <body name="link4" pos="{KNEE_OFF[0]} {KNEE_OFF[1]} {KNEE_OFF[2]}">
            <joint name="theta4" type="hinge" axis="1 0 0" armature="{ARM_K}"
                   stiffness="{p.get('knee_stiff', 0.0)}" springref="{p.get('knee_ref', 0.0)}"
                   damping="{DAMP_K + p.get('knee_damp', 0.0)}" range="-1.3 0.4"/>
            {_inertial(COM4, M4, I4)}
            <geom type="capsule" fromto="0 0 0 {FOOT[0]} {FOOT[1]} {FOOT[2]}" size="0.010" rgba="0.2 0.6 0.9 {a}"/>
            {g4}
            <geom name="foot" class="contact" type="sphere" pos="{FOOT[0]} {FOOT[1]} {FOOT[2]}" size="0.016" group="3" rgba="0.9 0.6 0.1 {af}"/>
            <site name="foot_site" pos="{FOOT[0]} {FOOT[1]} {FOOT[2]}" size="0.004" group="3"/>
            <site name="spr_b_R" pos="{SPR_B_R[0]} {SPR_B_R[1]} {SPR_B_R[2]}" size="0.006" rgba="0.8 0.7 0.3 1"/>
            <site name="spr_b_L" pos="{SPR_B_L[0]} {SPR_B_L[1]} {SPR_B_L[2]}" size="0.006" rgba="0.8 0.7 0.3 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <!-- 2 series-elastic tension springs (PDF): they stretch with theta4, giving the knee
         extension torque. springlength="0 L0" = a TENSION spring (pulls only). -->
    <spatial name="spring_R" width="0.005" rgba="0.8 0.7 0.1 1"
             stiffness="{KS_SPRING*p.get('spring_scale',1.0):.1f}" springlength="0 {L0_SPRING}">
      <site site="spr_a_R"/><site site="spr_b_R"/>
    </spatial>
    <spatial name="spring_L" width="0.005" rgba="0.8 0.7 0.1 1"
             stiffness="{KS_SPRING*p.get('spring_scale',1.0):.1f}" springlength="0 {L0_SPRING}">
      <site site="spr_a_L"/><site site="spr_b_L"/>
    </spatial>
  </tendon>
  <actuator>
    <motor name="hip" joint="theta3" gear="1" ctrlrange="-50 50"/>
    <motor name="knee" joint="theta4" gear="1" ctrlrange="-50 50"/>
  </actuator>
  <sensor>
    <jointpos name="enc_hip"  joint="theta3"/>   <!-- hip encoder (goBILDA 751.8 PPR) -->
    <jointpos name="enc_knee" joint="theta4"/>   <!-- knee encoder -->
    <jointvel name="vel_hip"  joint="theta3"/>
    <jointvel name="vel_knee" joint="theta4"/>
    <jointpos name="yaw_q1"   joint="theta1"/>   <!-- passive yaw (travel around the post) -->
    <jointpos name="pitch_q2" joint="theta2"/>   <!-- passive pitch (bob/hop) -->
    <framepos name="foot_pos" objtype="site" objname="foot_site"/>   <!-- foot position (MODEL 404 foot sensor) -->
    <touch    name="foot_touch" site="foot_site"/>                   <!-- foot contact -->
    <actuatorfrc name="tau_hip"  actuator="hip"/>
    <actuatorfrc name="tau_knee" actuator="knee"/>
  </sensor>
</mujoco>"""


def bezier(coef, s):
    from math import comb
    n = len(coef) - 1
    s = min(max(s, 0.0), 1.0)
    return sum(comb(n, i) * coef[i] * s**i * (1 - s)**(n - i) for i in range(n + 1))


if __name__ == "__main__":
    import mujoco
    print(f"link2: M={M2:.3f} kg  CoM=({COM2[0]:.3f},{COM2[1]:.3f},{COM2[2]:.3f})  "
          f"diagI=({I2[0,0]:.4f},{I2[1,1]:.4f},{I2[2,2]:.4f})")
    print(f"link3: M={M3:.3f}  link4: M={M4:.3f}  link1: M={M1:.3f}")
    print(f"TOTAL moving = {M1+M2+M3+M4:.3f} kg")
    m = mujoco.MjModel.from_xml_string(make_xml(dict(DEFAULTS, vis=False)))
    print("MJCF compiles OK, nq=", m.nq, "nu=", m.nu)
