"""Modelo MJCF ESTRUCTURAL (gemelo digital) del HOPPY rediseñado.

A diferencia de tune_eval.make_xml (modelo abstracto afinado a los parametros del
MATLAB original con mallas pegadas encima), aqui los eslabones SON la geometria real
medida del CAD/STEP del rediseño, con masas reales (PLA por volumen + motores goBILDA
del datasheet + boom PVC). Misma topologia validada (arbol serial yaw-pitch-hip-knee,
yaw+pitch pasivos), pero dimensiones y dinamica del robot del usuario.

Fuentes:
  - dimensiones: STEP (build_twin_meshes mide pivote, boom, cadera; pierna = HOPPY ref
    confirmada por los rodamientos del CAD: LH=96mm, DK=52mm, LK=154.5mm).
  - masas: STL x densidad PLA (housing 791g, pierna 192g) + 2x goBILDA 5202-2402-0027
    (435g c/u, datasheet) + electronica (~150g) + boom PVC (~481g).
  - actuador: goBILDA 0027 -> R=1.3, kT=0.0135, kv=0.0186, N=26.9 (idem get_params,
    validado contra el datasheet).
"""
import os
import numpy as np

MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")

# ---------------- dimensiones reales (m), medidas del STEP (GLB Y-up) ----------------
HB = 0.250          # altura del pivote (yaw+pitch) sobre el piso
# Pivote REAL de la cadera (link2-local), medido del CAD: top del muslo donde toca el
# housing. La pierna se monta en la CARA FRONTAL del housing (y=0.187), no en el centro,
# y un poco por debajo del boom (z=-0.036). Antes estaba mal a ojo en (0.730,0.099,0).
LB = 0.687          # boom horizontal: pivote -> cadera (X)
DB = 0.187          # offset LATERAL (tangencial) de la cadera = cara frontal del housing
HIP_DZ = -0.036     # la cadera esta 3.6 cm por debajo del eje del boom
LH = 0.096          # muslo recto (referencia HOPPY; el VECTOR real esta abajo)
DK = 0.052          # offset de rodilla              [HOPPY ref, confirmado CAD]
LK = 0.1545         # pantorrilla                    [HOPPY ref]
L  = float(np.hypot(LK, DK))   # pantorrilla efectiva = 0.163
# GEOMETRIA PLANAR REAL del 4-barras (medida del CAD), en el plano de oscilacion Y-Z:
# el muslo BAJA y va al FRENTE (+Y), la pantorrilla BAJA y VUELVE (-Y) -> pierna en zigzag.
# Antes la modelaba RECTA (cadera->rodilla->pie en -Z, con DK embebido en L como el MATLAB),
# pero entonces la malla CAD se DESCONECTABA al doblar (el eslabon giraba sobre un pivote
# falso). Con los vectores reales, cada eslabon gira sobre su PIVOTE REAL -> la malla CAD
# 4-barras articula CONECTADA. El controlador es agnostico (usa el Jacobiano de MuJoCo en
# vivo, no LH/L analiticos), asi que solo se re-afinan las ganancias.
KNEE_OFF = (-0.0018, 0.0586, -0.0583)   # rodilla en la union tubo-placas (geometria que SALTA)
FOOT     = (0.0048, -0.1126, -0.1717)   # pie en link4-local (pantorrilla con offset DK)

# --- resorte serie-elastico del knee (2x RESORTE DE TENSION, del PDF) ---
# El resorte va de un ancla en el MUSLO (A) a una palanca en la PANTORRILLA (B, ~81mm del
# eje de rodilla). Al flexionar (theta4<0) la palanca B rota y el resorte se ESTIRA (87->110mm
# a -0.5 rad) -> torque de extension. Se modela como 2 TENDONES (se estiran de verdad).
KS_SPRING = 1670.0   # N/m por resorte (PDF: Ks=1.67 kN/m)
L0_SPRING = 0.080    # m, longitud de reposo (PDF: L0=80mm)
SPR_A_R = (0.036, -0.022, -0.035)    # ancla superior (muslo/link3), lado +X
SPR_A_L = (-0.034, -0.025, -0.035)   # ancla superior, lado -X
SPR_B_R = (0.036,  0.002, -0.004)    # ancla inferior (pantorrilla/link4), lado +X
SPR_B_L = (-0.034, 0.000, -0.004)    # ancla inferior, lado -X
BOOM_X0, BOOM_X1 = -0.39, 0.81  # extremos del tubo (balance .. punta hoppy), model frame
BOOM_OD = 0.0334    # PVC 1"

# ---------------- masas reales (kg) ----------------
M_HOUSING = 0.791   # PLA del housing (STL x 1.24 g/cm3 x fill 0.6)
M_BOOM    = 0.481   # tubo PVC 1" (vol x 1.4)
M_MOTORS  = 0.870   # 2x goBILDA 5202-2402-0027 @ 435 g
M_ELEC    = 0.150   # TI F28379D + 2x VNH5019 + sensor (estimado)
M_THIGH   = 0.130   # PLA muslo + rodamiento
M_SHANK   = 0.100   # PLA pantorrilla + resortes + regaton
M_YAW     = 0.060   # bracket del carro de yaw (estimado)

# ---------------- actuador (goBILDA 0027, datasheet = get_params) ----------
NH, NK = 26.9, 26.9
Rw, kT, kv = 1.3, 0.0135, 0.0186
VMAX = 12.0
IMAX = 9.2          # corriente de stall del motor @12V (datasheet); el original usaba 30
N = np.array([NH, NK])
I_ROTOR = 7e-6
ARM_H, ARM_K = NH**2 * I_ROTOR, NK**2 * I_ROTOR
DT = 0.001
RBOOM = float(np.hypot(LB, DB))   # radio real cadera->eje yaw (0.712), no solo LB

# resorte suave de rodilla (el rebote real lo dan los resortes). Con la geometria planar
# real el cero de la rodilla = pose extendida del CAD, asi que el reposo del resorte ~0.
KNEE_STIFF, KNEE_REF = 0.0484, 0.0
FZ_BZ = np.array([0.0, 20.0, 100.0, 0.0, 0.0])
FX_BZ = np.array([0.0,  0.0, -25.0, 0.0, 0.0])


# ---------------- inercias compuestas ----------------
def _box_I(m, dx, dy, dz):
    return np.diag([m*(dy*dy+dz*dz)/12, m*(dx*dx+dz*dz)/12, m*(dx*dx+dy*dy)/12])

def _cyl_I_x(m, r, length):   # cilindro con eje en X
    return np.diag([m*r*r/2, m*(3*r*r+length*length)/12, m*(3*r*r+length*length)/12])

def _shift(I, m, r):          # parallel axis: I respecto a un punto desplazado -r del CoM
    r = np.asarray(r, float)
    return I + m*(np.dot(r, r)*np.eye(3) - np.outer(r, r))

def _compose(parts):
    """parts: lista de (I_com, m, com). Devuelve (M, com_total, I_about_com)."""
    M = sum(p[1] for p in parts)
    com = sum(p[1]*np.asarray(p[2], float) for p in parts)/M
    I = np.zeros((3, 3))
    for I_com, m, c in parts:
        I += _shift(I_com, m, np.asarray(c, float) - com)
    return M, com, I

# --- link2 = boom (rod) + hip-lump (housing+motors+elec, caja en la cadera) ---
_boom_com = np.array([(BOOM_X0+BOOM_X1)/2, 0, 0])
_boom_I = _cyl_I_x(M_BOOM, BOOM_OD/2, BOOM_X1-BOOM_X0)
_hip_m = M_HOUSING + M_MOTORS + M_ELEC
_hip_com = np.array([0.693, 0.068, 0.021])        # centroide housing+motores en link2-local (medido del CAD)
_hip_I = _box_I(_hip_m, 0.32, 0.20, 0.33)
M2, COM2, I2 = _compose([(_boom_I, M_BOOM, _boom_com), (_hip_I, _hip_m, _hip_com)])

# --- link3 = muslo: barra delgada hip->knee (vector real KNEE_OFF) ---
_ko = np.array(KNEE_OFF); _ft = np.array(FOOT)
M3 = M_THIGH; COM3 = _ko/2; I3 = _cyl_I_x(M3, 0.012, float(np.linalg.norm(_ko)))  # ~rod
# --- link4 = pantorrilla+pie: barra knee->foot (vector real FOOT) ---
M4 = M_SHANK; COM4 = _ft/2;  I4 = _cyl_I_x(M4, 0.012, float(np.linalg.norm(_ft)))
# --- link1 = carro de yaw: bloque chico ---
M1 = M_YAW; COM1 = np.array([0, 0, 0.0]); I1 = _box_I(M1, 0.06, 0.06, 0.05)


def _inertial(pos, m, I):
    return (f'<inertial pos="{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f}" mass="{m:.4f}" '
            f'fullinertia="{I[0,0]:.6f} {I[1,1]:.6f} {I[2,2]:.6f} '
            f'{I[0,1]:.6f} {I[0,2]:.6f} {I[1,2]:.6f}"/>')


# Config SIN contrapeso que AVANZA (circula el poste a ~0.16 rad/s) gracias al knob vx_d
# (la velocidad de avance deseada que faltaba). El salto en sitio anterior era un BUG de
# control: el control de vuelo usaba krh*vx sin vx_d -> amortiguaba el yaw a 0. Con vx_d!=0
# regula vx hacia vx_d y avanza. Salto 4.9cm, pie despega 4.3cm, 35% vuelo, motores al
# limite 12V/9.2A (sin margen; un contrapeso ~2kg da mas margen pero NO es necesario para
# avanzar — ver HANDOFF y twin_tune). q3 limitado a 0.3-1.5 rad (rango realista).
DEFAULTS = dict(
    solref0=0.0191, j_damp=0.1494,
    knee_stiff=0.0948, knee_ref=0.0088, knee_damp=0.0, spring_scale=0.0,  # resorte de JUNTA; tendon = visual
    kp_sw=249.60, kd_sw=5.0839, krh=0.1187, p_toe_z=-0.1828,
    Tst=0.285, kp_st=0.03, kd_st=0.08, q3_ref=0.1572, q4_ref=-0.4607,
    fz_scale=3.1838, fx_scale=3.3653, blend=0.0136, grf_liftoff=3.8109,
    vx_d=-0.3385,   # velocidad de avance deseada (Raibert)
    # Config que SALTA (seed23): cuerpo +8.4cm, pie despega 6.6cm, avanza 0.63 rad/s. Pierna
    # PROCEDURAL (leg_proc) que articula conectada. El resorte de junta da el rebote; el tendon
    # dorado es VISUAL (spring_scale=0). El resorte 100% real (1.67kN/m) NO salta (ver HANDOFF).
)
# Knee SERIE-ELASTICO real: rodilla en el pivote P4 (del STEP), pantorrilla con offset DK,
# y 2 TENDONES-resorte (Ks=1.67kN/m, L0=80mm del PDF) que se ESTIRAN con theta4 (87->114mm)
# dando el torque de extension real y quedando SIEMPRE conectados muslo<->palanca. La bobina
# rigida se quito de la malla (un solido no se estira). PENDIENTE: re-afinar la marcha.


def make_xml(p):
    sol = p.get('solref0', 0.012)
    jd = p.get('j_damp', 0.0)
    vis = p.get('vis', False) and os.path.exists(f"{MESH_DIR}/twin_housing.obj")
    a = "0" if vis else "1"
    af = "0" if vis else "0.9"
    PLA = "0.78 0.80 0.83 1"; DKc = "0.2 0.2 0.23 1"; PVC = "0.93 0.93 0.9 1"
    HOUSING = "0.65 0.65 0.65 1"   # gris medio del CAD (Tarea A)
    MOT = "0.2 0.2 0.2 1"          # motores goBILDA
    GOLD = "0.8 0.7 0.1 1"         # resortes de tension (tendones)
    BLK = "0.15 0.15 0.15 1"       # placas/eslabones negros
    RUB = "0.1 0.1 0.1 1"          # regaton
    # leg_mesh: malla CAD 4-barras (para el ensamble estatico). leg_proc=True fuerza la
    # pierna procedural limpia (para el render del SALTO, donde el 4-barras se desconecta
    # al doblar; la dinamica es identica porque las mallas son solo visuales).
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
    # gantry fijo (mundo)
    if vis:
        gantry = f'<geom name="post" type="mesh" mesh="mg" rgba="{PLA}"/>'
    else:
        gantry = (f'<geom name="post" type="cylinder" pos="0 0 {HB/2}" size="0.02 {HB/2}" rgba="0.4 0.4 0.4 1"/>'
                  f'<geom type="cylinder" pos="0 0 0.004" size="0.085 0.004" rgba="0.4 0.4 0.4 1"/>')
    # boom PVC (link2)
    boom = f'<geom type="cylinder" fromto="{BOOM_X0} 0 0 {BOOM_X1} 0 0" size="{BOOM_OD/2}" rgba="{PVC if vis else "0.6 0.6 0.6 "+a}"/>'
    # CONTRAPESO opcional (lado de balance del boom): masa cw_mass a x=cw_x en link2-local.
    # Sirve para balancear el cabeceo del hoppy y ver si lo hace saltar como el original.
    cw_m, cw_x = p.get('cw_mass', 0.0), p.get('cw_x', -0.65)
    if cw_m > 0:
        cw = (f'<body name="cw" pos="{cw_x} 0 0">'
              f'<inertial pos="0 0 0" mass="{cw_m}" diaginertia="{cw_m*0.001:.5f} {cw_m*0.001:.5f} {cw_m*0.001:.5f}"/>'
              f'<geom type="cylinder" fromto="-0.02 0 0 0.02 0 0" size="{0.025+0.012*cw_m:.3f}" rgba="0.7 0.2 0.2 1"/>'
              f'</body>')
    else:
        cw = ""
    # pierna en link3/link4: malla CAD real (4-barras) si existe, si no procedural
    if leg_mesh:
        kx, ky, kz = KNEE_OFF; fx, fy, fz = FOOT
        # malla CAD del muslo/pantorrilla (ya muestra el 4-barras) + 2 motores y el regaton.
        # el contacto fisico (geom "foot") va invisible en vis; el regaton es solo visual.
        g3 = (f'<geom type="mesh" mesh="mt" rgba="{PLA}"/>'
              f'<geom type="cylinder" fromto="-0.02 0 0 0.02 0 0" size="0.045" rgba="{MOT}"/>'                          # motor cadera (en el hip)
              f'<geom type="cylinder" fromto="{kx-0.02:.4f} {ky} {kz} {kx+0.02:.4f} {ky} {kz}" size="0.045" rgba="{MOT}"/>')  # motor rodilla (en KNEE_OFF)
        g4 = (f'<geom type="mesh" mesh="msh" rgba="{PLA}"/>'
              f'<geom type="sphere" pos="{fx} {fy} {fz}" size="0.018" rgba="{RUB}"/>')                                  # regaton negro en FOOT
    elif vis:
        PLAg = "0.65 0.65 0.65 1"     # tubo gris medio (CAD)
        kx, ky, kz = KNEE_OFF; fx, fy, fz = FOOT
        # Pierna procedural que SIEMPRE articula conectada (la malla CAD se parte al doblar):
        # muslo (link3) = motor de cadera + 2 placas NEGRAS (IMP-8/9) + motor de rodilla (el
        # motor de la rodilla va montado en el MUSLO/link3 aunque el joint theta4 este en link4,
        # asi queda fijo al muslo al articular); pantorrilla (link4) = 2 rodamientos + tubo
        # TUB-1 + regaton. El RESORTE es el TENDON dorado (en <tendon>, sigue la articulacion).
        lat = 0.010                   # media-separacion lateral de las placas (0.02 total)
        g3 = (f'<geom type="cylinder" fromto="-0.024 0 0 0.024 0 0" size="0.018" rgba="{MOT}"/>'                          # motor de cadera (hip)
              f'<geom type="capsule" fromto="{-lat:.4f} 0 0 {-lat+kx:.4f} {ky} {kz}" size="0.007" rgba="{BLK}"/>'         # placa IMP-8 (negra)
              f'<geom type="capsule" fromto="{ lat:.4f} 0 0 { lat+kx:.4f} {ky} {kz}" size="0.007" rgba="{BLK}"/>'         # placa IMP-9 (negra)
              f'<geom type="cylinder" fromto="{kx-0.022:.4f} {ky} {kz} {kx+0.022:.4f} {ky} {kz}" size="0.020" rgba="{MOT}"/>')  # motor de rodilla (KNEE_OFF, en link3)
        g4 = (f'<geom type="cylinder" fromto="{-lat-0.0075:.4f} 0 0 {-lat+0.0075:.4f} 0 0" size="0.012" rgba="{BLK}"/>'   # rodamiento izq
              f'<geom type="cylinder" fromto="{ lat-0.0075:.4f} 0 0 { lat+0.0075:.4f} 0 0" size="0.012" rgba="{BLK}"/>'   # rodamiento der
              f'<geom type="cylinder" fromto="0 0 0 {fx} {fy} {fz}" size="0.014" rgba="{PLAg}"/>'                         # tubo TUB-1 (gris)
              f'<geom type="sphere" pos="{fx} {fy} {fz}" size="0.020" rgba="0.08 0.08 0.08 1"/>')                          # regaton (esfera negra)
    else:
        g3 = g4 = ""
    # link1 (yaw): cilindro oscuro del pivote, visible solo en modo vis
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
          <joint name="theta3" type="hinge" axis="1 0 0" armature="{ARM_H}" range="-0.5 0.9"/>
          {_inertial(COM3, M3, I3)}
          <geom type="capsule" fromto="0 0 0 {KNEE_OFF[0]} {KNEE_OFF[1]} {KNEE_OFF[2]}" size="0.012" rgba="0.2 0.5 0.8 {a}"/>
          {g3}
          <site name="spr_a_R" pos="{SPR_A_R[0]} {SPR_A_R[1]} {SPR_A_R[2]}" size="0.006" rgba="0.8 0.7 0.3 1"/>
          <site name="spr_a_L" pos="{SPR_A_L[0]} {SPR_A_L[1]} {SPR_A_L[2]}" size="0.006" rgba="0.8 0.7 0.3 1"/>
          <body name="link4" pos="{KNEE_OFF[0]} {KNEE_OFF[1]} {KNEE_OFF[2]}">
            <joint name="theta4" type="hinge" axis="1 0 0" armature="{ARM_K}"
                   stiffness="{p.get('knee_stiff', 0.0)}" springref="{p.get('knee_ref', 0.0)}"
                   damping="{p.get('knee_damp', 0.02)}" range="-1.3 0.4"/>
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
    <!-- 2 resortes de tension serie-elasticos (PDF): se estiran con theta4, dan el torque
         de extension de la rodilla. springlength="0 L0" = resorte de TENSION (solo jala). -->
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
    <jointpos name="enc_hip"  joint="theta3"/>   <!-- encoder cadera (goBILDA 751.8 PPR) -->
    <jointpos name="enc_knee" joint="theta4"/>   <!-- encoder rodilla -->
    <jointvel name="vel_hip"  joint="theta3"/>
    <jointvel name="vel_knee" joint="theta4"/>
    <jointpos name="yaw_q1"   joint="theta1"/>   <!-- yaw pasivo (avance alrededor del poste) -->
    <jointpos name="pitch_q2" joint="theta2"/>   <!-- pitch pasivo (cabeceo/salto) -->
    <framepos name="foot_pos" objtype="site" objname="foot_site"/>   <!-- pos del pie (sensor de pie MODEL 404) -->
    <touch    name="foot_touch" site="foot_site"/>                   <!-- contacto del pie -->
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
    print(f"TOTAL movil = {M1+M2+M3+M4:.3f} kg")
    m = mujoco.MjModel.from_xml_string(make_xml(dict(DEFAULTS, vis=False)))
    print("MJCF compila OK, nq=", m.nq, "nu=", m.nu)
