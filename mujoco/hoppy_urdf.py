"""hoppy_urdf.py — el URDF real de HOPPY como modulo compatible con controller.py.

Toma el URDF importado (load_hoppy_urdf), renombra los joints joint1..4 -> theta1..4
(opcion B: asi controller.py los encuentra sin cambios) y le INYECTA lo que el control
hibrido necesita y el URDF no trae:
  - piso (clase "contact")
  - geom 'foot' + site 'foot_site' en la PUNTA real del regaton de Link4 (medida)
  - body 'cuerpo_cadera' = frame del boom (Link2) en la cadera, para el PD cartesiano
  - actuadores de torque hip(theta3)/knee(theta4)
  - resorte de rodilla, armaduras N^2*Ir y damping (back-EMF) como en twin
  - opcion implicitfast (estable con contacto; twin usa esto, NO RK4)

El frame es compatible: el eje de oscilacion de la pierna cae en X del frame del boom
(verificado: R2.T @ eje = [-1,0,0]), que es lo que controller.py asume. Masas reales del
URDF = 3.43 kg (vs 2.58 del twin) -> las ganancias de fuerza se escalan ~1.33 como primer
guess; el gait fino se re-afina aparte.

Reusa motores goBILDA + Bezier + constantes del twin. NO modifica controller.py,
load_hoppy_urdf.py, twin.py ni verify.py.
"""
import os
import re
import numpy as np
import mujoco

import load_hoppy_urdf as L
import twin
# nombres que controller.py lee como atributos del modulo del modelo:
from twin import (bezier, FZ_BZ, FX_BZ, NH, NK, Rw, kT, kv, VMAX, IMAX, N, DT,
                  ARM_H, ARM_K, DAMP_H, DAMP_K)

RBOOM = 0.661                              # cadera -> eje yaw del URDF (medido)
FOOT_TIP = (0.0227, -0.1824, -0.0274)     # regaton al final del shank en Link4-local (medida)
MASS_RATIO = 3.43 / 2.58                   # ~1.33  (URDF vs twin)
HIP_POS_L2 = "-0.6585 0 0.0425"            # cadera en frame Link2 (pos de Link3 sin su quat)

# DEFAULTS = twin escalando las ganancias de fuerza por la razon de masas (primer guess)
DEFAULTS = dict(twin.DEFAULTS)
for _k in ("kp_sw", "kd_sw", "kp_st", "kd_st", "fz_scale", "fx_scale", "knee_stiff"):
    DEFAULTS[_k] = DEFAULTS[_k] * MASS_RATIO

# ---------------------------------------------------------------------------------------
# FORWARD: controlador HIBRIDO REAL (controller.py = port fiel del MATLAB/paper) con las
# CONSTANTES REALES del simulador MATLAB + boom BALANCEADO con contrapeso (como el HOPPY
# del paper; el modelo MATLAB lleva el CoM del boom en rx2=-0.50). Es el control que correra
# en la LaunchPad F28379D (cpu01_main), NO el FSM tonto de hop_controller.
#
# Resultado: el URDF salta hacia ADELANTE = -theta1 (lado OPUESTO al offset de la pierna,
# el "lado sin pierna"), ~0.58 rad/s, pie despega ~7.6 cm, 66% vuelo, ciclo limite estable,
# V<=12 / i<=9.2 A (igual que el MATLAB). El sentido lo fija el empuje tangencial del apoyo
# (GRF Bezier Fx pico=-25, el valor real del MATLAB); coincide con el sentido del MATLAB.
#
# Constantes reales (= get_params.m / firmware cpu01_main):
#   kp_sw=150, kd_sw=5  (PD cartesiano aereo; firmware Kp_a/Kd_a)
#   krh=0.10            (colocacion de pie Raibert)        p_toe_z=-0.15 (pie 15 cm bajo cadera)
#   Tst=0.35           (tiempo de apoyo nominal)           grf_liftoff=1.5 (umbral de despegue)
#   Fz_bz pico=100, Fx_bz pico=-25 (perfil GRF Bezier del apoyo)  kp_st=0.03 kd_st=0.08 (PD suave)
#   modelo de motor goBILDA: Rw=1.3 kT=0.0135 kv=0.0186 Nh=26.9 Nk=28.8, sat 12 V / 9.2 A
FORWARD = dict(
    DEFAULTS,
    kp_sw=150.0, kd_sw=5.0, krh=0.10, p_toe_z=-0.18, Tst=0.35,  # p_toe_z=-0.18: aterriza mas extendida (pierna mas vertical)
    kp_st=0.03, kd_st=0.08, fz_scale=1.0, fx_scale=1.0, grf_liftoff=1.5,
    q3_ref=0.55, q4_ref=-0.95,        # cuclilla valida en el espacio de juntas del URDF
    cw_mass=2.86, cw_x=0.35,          # contrapeso: brazo REAL del boom fisico (35 cm del pivote),
                                      # masa por BALANCE DE MOMENTOS al ~76% (deja peso efectivo
                                      # en la pierna; el balance 100%=3.77 kg sobrelanza). Ver contrapeso.py
    vx_d=0.0,                         # velocidad Raibert deseada = 0 (el MATLAB puro; Fx fija el avance)
)

_BASE = None


def _base_mjcf():
    """MJCF base del URDF (cinematica + inercia + mallas), cacheado."""
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

    # 2) aumentar los joints (armadura, damping, resorte de rodilla)
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
    # pie: la pierna se mantiene IGUAL que el URDF (malla del CAD). El contacto es el regaton
    # al final del shank (FOOT_TIP). La fisica es un punto en esa punta.
    foot = ('<site name="foot_site" pos="%g %g %g" size="0.03" rgba="1 0 0 0"/>'
            '<geom name="foot" class="contact" type="sphere" size="0.016" pos="%g %g %g" '
            'group="3" rgba="0.9 0.6 0.1 1"/>' % (fx, fy, fz, fx, fy, fz))
    xml = xml.replace(
        '<joint name="theta4" pos="0 0 0" axis="0 0 -1" range="-1.3 0.4" actuatorfrcrange="-10 10"/>',
        '<joint name="theta4" pos="0 0 0" axis="0 0 -1" range="-1.3 0.4" armature="%g" '
        'stiffness="%g" springref="%g" damping="%g"/>' % (ARM_K, ks, kr, kdmp) + foot)

    # 3) opcion + defaults de contacto (tras el <compiler/>); implicitfast por estabilidad
    head = ('<option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>'
            '<default><geom contype="0" conaffinity="0"/>'
            '<default class="contact"><geom contype="1" conaffinity="1" solref="%g 1" '
            'solimp="0.95 0.99 0.001" friction="2.0 0.1 0.1"/></default></default>' % sol)
    xml = re.sub(r'(<compiler[^>]*/>)', lambda mm: mm.group(1) + "\n  " + head, xml, count=1)

    # 4) piso (cuadricula: usa el material 'grid' inyectado al final; SOLO visual.
    #    La fisica/friccion sigue en class="contact" -> intacta).
    xml = xml.replace(
        "<worldbody>",
        '<worldbody>\n    <geom name="floor" class="contact" type="plane" size="3 3 0.1" '
        'material="grid"/>', 1)

    # 5) cuerpo_cadera (frame del boom en la cadera) como hijo de Link2, antes de Link3.
    #    + CONTRAPESO opcional del boom: el HOPPY real (y el modelo MATLAB, rx2=-0.50) llevan
    #    una masa en el lado OPUESTO al hopper (+x local de Link2) para balancear el pitch.
    #    Sin el, el CoM del boom queda 0.55 m hacia el hopper (~13 N*m de gravedad) y el robot
    #    solo bobea sin despegar. cw_mass/cw_x lo reproducen (faithful al diseno del paper).
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

    # 6) sensor de contacto + actuadores de torque
    hg, kg = p.get("hip_gear", 1.0), p.get("knee_gear", 1.0)   # signo del actuador (eje knee del URDF invertido)
    xml = xml.replace(
        "</mujoco>",
        '<sensor><touch name="foot_touch" site="foot_site"/></sensor>'
        '<actuator><motor name="hip" joint="theta3" gear="%g" ctrlrange="-5 5"/>'
        '<motor name="knee" joint="theta4" gear="%g" ctrlrange="-5 5"/></actuator></mujoco>' % (hg, kg))

    # modo rapido (tuner): quita las mallas VISUALES para cargar instantaneo. La dinamica
    # es IDENTICA: las inercias son explicitas en cada body y el contacto es la esfera 'foot'.
    if p.get("fast"):
        xml = re.sub(r'<geom type="mesh"[^>]*/>', '', xml)
        xml = re.sub(r'<asset>.*?</asset>', '<asset/>', xml, flags=re.DOTALL)

    # textura de cuadricula del piso (MuJoCo builtin checker). Se inyecta SIEMPRE -- tambien
    # en modo fast, que vacia el <asset> -- para que el material 'grid' del floor exista. Es
    # PURAMENTE visual: no toca contype/conaffinity/solref/friction (eso vive en class="contact").
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
    print("compila OK | nq=%d nu=%d nsite=%d nsensor=%d ngeom=%d" %
          (m.nq, m.nu, m.nsite, m.nsensor, m.ngeom))
    for nm in ("theta1", "theta2", "theta3", "theta4", "foot_site", "foot", "cuerpo_cadera", "hip", "knee"):
        for obj in (mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_SITE,
                    mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_ACTUATOR):
            if mujoco.mj_name2id(m, obj, nm) >= 0:
                print("  OK existe:", nm); break
        else:
            print("  FALTA:", nm)
