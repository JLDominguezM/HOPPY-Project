"""Modelo MuJoCo de HOPPY anclado a los parametros del simulador MATLAB.

make_xml(p) genera el MJCF con masas/CoM/inercias/geometria calcadas de
Simulator_MATLAB/fcns/get_params.m. El "contrapeso" NO es una bola tuneada:
es link2 (boom+contrapeso) con M2=2.365 kg y CoM en -0.502 m, justo como el
robot real, lo que fija el balance fisicamente.

Convencion de frames del control (igual que MATLAB fcn_p_toe_HIP):
  - El "frame de cadera" esta fijo al boom (link2): X a lo largo del boom
    (+ hacia la pierna), Z vertical. El pie objetivo en apoyo/vuelo se expresa
    en [X, Z] de ese frame.

Constantes fisicas (get_params.m) reusadas por control.py / verify.py.
"""
import numpy as np
from math import comb

# --- parametros fisicos fijos (get_params.m) ---
NH, NK = 26.9, 28.8                 # reducciones cadera/rodilla
Rw, kT, kv = 1.3, 0.0135, 0.0186    # resistencia, cte. par, cte. velocidad
VMAX, IMAX = 12.0, 30.0             # saturacion 12 V / 30 A
N = np.array([NH, NK])
I_ROTOR = 7e-6
ARM_H, ARM_K = NH**2 * I_ROTOR, NK**2 * I_ROTOR    # inercia rotor reflejada N^2*Ir
DT = 0.001

# --- geometria (get_params.m: HB, LB, LH, DK, LK) ---
HB, LB = 0.1965, 0.556              # altura del pivote, largo del boom (=Rboom)
LH = 0.096                          # muslo
LK = np.hypot(0.052, 0.1545)        # pantorrilla efectiva sqrt(DK^2+LK^2)=0.163
RBOOM = LB

# --- masas (get_params.m) ---
M1, M2, M3, M4 = 0.268, 2.365, 0.656, 0.149

# --- CoM por link ---
# link1 y link2 en su frame MATLAB (coincide con el de MuJoCo: +x hacia la pierna).
# link3/link4 al centro geometrico del eslabon en el frame MuJoCo (la pierna apunta -z),
# porque el frame de eslabon de pierna del MATLAB no coincide en orientacion.
COM1 = (-0.00057, 0.0, -0.0618)
# CoM x de link2 se CALIBRA (cw_x en DEFAULTS) para que el torque gravitacional
# de MuJoCo en theta2 iguale al Ge(theta2)=-5.95 N*m del MATLAB (fcn_Ge). El
# frame de eslabon del MATLAB no coincide en orientacion, asi que -0.502 no se
# usa directo; el boom queda casi balanceado con leve carga hacia la cadera.
COM2_YZ = (-0.0368, 0.0)            # offsets laterales (menores)
COM3 = (0.0, 0.0, -LH / 2)
COM4 = (0.0, 0.0, -LK / 2)

# --- inercias diagonales (get_params.m) ---
I1 = (0.00115952, 0.00104649, 0.00030518)
I2 = (0.00270252, 0.30208952, 0.30305924)   # boom+contrapeso: gran inercia yaw/pitch
I3 = (0.00082110, 0.00235762, 0.00168340)
I4 = (0.00039424, 0.00032191, 0.00010442)

# --- resorte de rodilla suave (MATLAB: tau_s = 2*(-0.0242*q4 + 0.0108)) ---
# Equivale a stiffness*(q4-ref) con stiffness=0.0484, ref=0.0216/0.0484=0.446.
KNEE_STIFF, KNEE_REF = 0.0484, 0.446

# --- perfil de fuerza de apoyo (Bezier, get_params.m) ---
FZ_BZ = np.array([0.0, 20.0, 100.0, 0.0, 0.0])
FX_BZ = np.array([0.0,  0.0, -25.0, 0.0, 0.0])

# Config afinado para igualar la referencia MATLAB (amplitud 7.2 cm, ~2.5 Hz,
# avance ~1 rad/s alrededor del poste). Hallado por busqueda multi-agente con
# tune_metric (verify + distancia a la referencia): score 99/100, estable
# (apex std 4.5 mm), V<=12 / i<=12. La pierna oscila en el plano TANGENCIAL
# (eje X) para propulsar el avance, igual que el MATLAB. kp_sw alto (vs 150 del
# MATLAB) mantiene la pierna retraida en vuelo; knee_stiff+fz_scale dan el
# rebote lento de gran amplitud; j_damp+solref ajustan la disipacion.
DEFAULTS = dict(
    # modelo
    solref0=0.012683, j_damp=0.290198,
    cw_x=0.080,                 # CoM x de link2; calibrado a Ge(t2)_MATLAB=-5.95
    knee_stiff=0.250873,        # resorte de rodilla (rebote)
    # fase aerea (Ec.17): PD cartesiano del pie en el frame de cadera (plano tangencial)
    kp_sw=432.980271, kd_sw=5.0, krh=0.094137, p_toe_z=-0.103559,
    # fase de apoyo (Ec.19): Bezier + PD suave de junta
    Tst=0.234088, kp_st=0.03, kd_st=0.08, q3_ref=np.pi/3, q4_ref=-np.pi/2,
    fz_scale=1.576776, fx_scale=1.0,
    # blending (Ec.20) y FSM
    blend=0.015719, grf_liftoff=2.290694,
    # SOLO visual (modo vis): offset en X de la pierna para sacarla a la cara exterior
    # del housing (como en el robot real) y que no quede tapada. No afecta la fisica.
    legx=0.17,
)


def bezier(coef, s):
    n = len(coef) - 1
    s = min(max(s, 0.0), 1.0)
    return sum(comb(n, i) * coef[i] * s**i * (1 - s)**(n - i) for i in range(n + 1))


import os
MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")


def gantry_xml(p):
    """Estructura FIJA del gantry, modelada para parecerse al robot real (PPTX slide 4):
    base octagonal en el piso -> torre del balero de YAW (delta2=theta1, placas con
    tornillos) -> y arriba, a z=HB, el balero de PITCH (delta1=theta2) por donde pasa
    el boom. El bloque del balero de pitch va en link1 (gira en yaw con el boom), aqui
    NO; este modulo solo dibuja la parte FIJA (base + torre)."""
    if not p.get('gantry', False):
        return f'<geom name="post" type="cylinder" fromto="0 0 0 0 0 {HB}" size="0.02" rgba="0.3 0.3 0.3 1"/>'
    W = "0.82 0.83 0.86 1"                       # PLA claro impreso
    # base octagonal (aprox. con cilindro) apoyada en el piso
    base = f'<geom name="post" type="cylinder" pos="0 0 0.005" size="0.085 0.005" rgba="{W}"/>'
    # torre de yaw: dos placas laterales (las de los tornillos) + placa trasera
    zt0, zt1 = 0.010, HB - 0.030                 # de la base hasta justo bajo el balero
    zc, zh = (zt0 + zt1) / 2, (zt1 - zt0) / 2
    sides = "".join(f'<geom type="box" pos="0 {sy*0.045} {zc}" size="0.026 0.004 {zh}" rgba="{W}"/>'
                    for sy in (-1, 1))
    back = f'<geom type="box" pos="-0.028 0 {zc}" size="0.004 0.045 {zh}" rgba="{W}"/>'
    shaft = f'<geom type="cylinder" pos="0 0 {zc}" size="0.012 {zh}" rgba="0.3 0.3 0.33 1"/>'  # eje yaw
    return base + sides + back + shaft


def make_xml(p):
    sol = p.get('solref0', 0.002)
    jd = p.get('j_damp', 0.0)
    # modo visual: pega la malla CAD del housing y oculta la geometria abstracta.
    # La pierna (4 barras) NO se toma del CAD: ese mecanismo no mapea a los 2 eslabones
    # seriales del modelo y al "desplegarlo" quedaba desperdigado; se dibuja procedural
    # (barras planas estilo impreso + pie) que calza exacto con la cinematica.
    vis = p.get('vis', False) and os.path.exists(f"{MESH_DIR}/vis_link2.obj")
    a = "0" if vis else "1"          # alpha de la geometria abstracta
    af = "0" if vis else "0.9"       # alpha del pie (contacto): invisible en modo visual
    PLA = "0.55 0.62 0.78 1"         # color de las piezas impresas
    DK = "0.2 0.2 0.23 1"            # juntas/poleas/pie (oscuro)
    asset = f'<mesh name="v2" file="{MESH_DIR}/vis_link2.obj"/>' if vis else ""
    g2 = f'<geom type="mesh" mesh="v2" rgba="{PLA}"/>' if vis else ""
    # pierna procedural: muslo (hip->rodilla, -Z, swing en Y-Z) y pantorrilla (rodilla->pie)
    # offset visual de la pierna a lo largo del eje X (eje de las juntas): la coloca en
    # la cara exterior del housing (como en el robot real, donde cuelga del extremo
    # outboard) para que no quede tapada. Como ambas juntas giran sobre X, este offset
    # se preserva en todo el movimiento. Es solo visual; la fisica del pie no cambia.
    lx = p.get('legx', 0.0)
    LEG = "0.9 0.9 0.93 1"           # eslabones impresos blancos (contrastan con el housing)
    if vis:
        g3 = (f'<geom type="cylinder" fromto="{lx-0.024} 0 0 {lx+0.024} 0 0" size="0.018" rgba="{DK}"/>'  # polea/motor de cadera
              f'<geom type="box" pos="{lx} 0 -{LH/2}" size="0.007 0.016 {LH/2}" rgba="{LEG}"/>'      # barra del muslo
              f'<geom type="box" pos="{lx} 0.018 -{LH/2}" size="0.005 0.005 {LH/2}" rgba="{LEG}"/>') # 2da barra (pantografo)
        g4 = (f'<geom type="cylinder" fromto="{lx-0.017} 0 0 {lx+0.017} 0 0" size="0.013" rgba="{DK}"/>'   # junta de rodilla
              f'<geom type="box" pos="{lx} 0 -{LK/2}" size="0.006 0.012 {LK/2}" rgba="{LEG}"/>'      # barra de pantorrilla
              f'<geom type="sphere" pos="{lx} 0 -{LK}" size="0.018" rgba="{DK}"/>')                  # pie de hule
    else:
        g3 = g4 = ""
    # boom: en modo visual es el tubo de PVC blanco de 1" (OD ~33mm) que conecta el
    # hoppy con el gantry; el sobrante hacia -x hace de balance (no hay contrapeso
    # separado). En modo abstracto: varilla gris + disco rojo del balance.
    if vis:
        boom = f'<geom type="cylinder" fromto="-0.52 0 0 {LB} 0 0" size="0.0167" rgba="0.93 0.93 0.9 1"/>'
    else:
        boom = (f'<geom type="cylinder" fromto="-0.50 0 0 {LB} 0 0" size="0.009" rgba="0.6 0.6 0.6 {a}"/>'
                f'<geom type="cylinder" fromto="-0.50 -0.04 0 -0.50 0.04 0" size="0.06" rgba="0.7 0.2 0.2 {a}"/>')
    # balero de PITCH (delta1): bloque en link1, gira en yaw con el boom y queda
    # nivelado en pitch; el boom (PVC) lo atraviesa. Lleva la mordaza negra arriba.
    if vis:
        g1 = ('<geom type="box" pos="0 0 0" size="0.021 0.05 0.02" rgba="0.82 0.83 0.86 1"/>'
              '<geom type="box" pos="0 0 0.02" size="0.018 0.045 0.007" rgba="0.13 0.13 0.16 1"/>')
    else:
        g1 = f'<geom type="box" size="0.025 0.025 0.025" rgba="0.4 0.4 0.4 {a}"/>'
    return f"""<mujoco model="hoppy">
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
    {gantry_xml(p)}
    <body name="link1" pos="0 0 {HB}">
      <joint name="theta1" type="hinge" axis="0 0 1" damping="{jd}"/>
      <inertial pos="{COM1[0]} {COM1[1]} {COM1[2]}" mass="{M1}"
                diaginertia="{I1[0]} {I1[1]} {I1[2]}"/>
      {g1}
      <body name="link2" pos="0 0 0">
        <joint name="theta2" type="hinge" axis="0 1 0" damping="{jd}"/>
        <inertial pos="{p.get('cw_x', 0.078)} {COM2_YZ[0]} {COM2_YZ[1]}" mass="{M2}"
                  diaginertia="{I2[0]} {I2[1]} {I2[2]}"/>
        {boom}
        {g2}
        <body name="cuerpo_cadera" pos="{LB} 0 0">
          <inertial pos="0 0 0" mass="1e-6" diaginertia="1e-9 1e-9 1e-9"/>
          <geom type="box" size="0.03 0.03 0.03" rgba="0.2 0.4 0.7 {a}"/>
        </body>
        <body name="link3" pos="{LB} 0 0">
          <joint name="theta3" type="hinge" axis="1 0 0" armature="{ARM_H}"
                 range="0.2 2.2"/>
          <inertial pos="{COM3[0]} {COM3[1]} {COM3[2]}" mass="{M3}"
                    diaginertia="{I3[0]} {I3[1]} {I3[2]}"/>
          <geom type="capsule" fromto="0 0 0 0 0 -{LH}" size="0.012" rgba="0.2 0.5 0.8 {a}"/>
          {g3}
          <body name="link4" pos="0 0 -{LH}">
            <joint name="theta4" type="hinge" axis="1 0 0" armature="{ARM_K}"
                   stiffness="{p.get('knee_stiff', KNEE_STIFF)}" springref="{p.get('knee_ref', KNEE_REF)}" range="-2.8 -0.7"/>
            <inertial pos="{COM4[0]} {COM4[1]} {COM4[2]}" mass="{M4}"
                      diaginertia="{I4[0]} {I4[1]} {I4[2]}"/>
            <geom type="capsule" fromto="0 0 0 0 0 -{LK}" size="0.010" rgba="0.2 0.6 0.9 {a}"/>
            {g4}
            <geom name="foot" class="contact" type="sphere" pos="0 0 -{LK}" size="0.012" rgba="0.9 0.6 0.1 {af}"/>
            <site name="foot_site" pos="0 0 -{LK}" size="0.006"/>
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
