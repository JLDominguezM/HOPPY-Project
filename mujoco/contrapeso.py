"""contrapeso.py — cálculo del contrapeso del boom de HOPPY por BALANCE DE MOMENTOS.

Por qué hace falta:
  El boom es una palanca que pivota en el gantry. De un lado cuelga el HOPPY (pierna +
  motores + housing + electrónica), del otro va el contrapeso. Sin contrapeso, el CoM del
  conjunto queda lejos hacia el hopper y la gravedad genera un momento (~13 N·m en el modelo)
  que el motor real (saturando a 12 V) NO puede vencer para despegar: el robot solo "bobea".

Física (balance de momentos respecto al pivote, boom horizontal):
       M_cw * d_cw  =  M_hopper * d_hopper        (= momento de desbalance / g)
  - d_cw     = brazo del contrapeso (pivote -> contrapeso)            [TU dato: 0.35 m]
  - d_hopper = brazo del hopper     (pivote -> CoM del lado hopper)
  - M_hopper = masa total que cuelga del lado del hopper

OJO — un saltarín NO se balancea al 100%: el balance total deja al pie SIN "peso de cuerpo"
contra el cual rebotar y el empuje sobrelanza el boom (en sim, 100% -> explota a ~68 cm). Se
deja un RESIDUAL (~24%) de desbalance = el peso efectivo que la pierna bota. Sweet spot ~76%.

Uso:
  python3 contrapeso.py                 # reporta para el modelo de sim (geometría del CAD)
  # y edita FISICO abajo con tus medidas para tu boom real (85 cm hopper, 35 cm contrapeso).
"""
import numpy as np

G = 9.81
BAL = 0.76   # fracción de balance objetivo (deja ~24% de peso efectivo en la pierna)


def M_cw(momento_desbalance, d_cw, frac=BAL):
    """Contrapeso [kg] a brazo d_cw [m] para 'frac' del balance de un momento [N*m]."""
    return frac * momento_desbalance / (G * d_cw)


def momento_desde_masa(M_hopper, d_hopper):
    """Momento de desbalance [N*m] = M_hopper [kg] * g * d_hopper [m]."""
    return M_hopper * G * d_hopper


def medir_en_sim():
    """Mide el momento de desbalance real del modelo URDF (sin contrapeso)."""
    import mujoco, hoppy_urdf
    m = mujoco.MjModel.from_xml_string(hoppy_urdf.make_xml(dict(hoppy_urdf.DEFAULTS, fast=True, cw_mass=0)))
    d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    d.qpos[m.jnt_qposadr[jid("theta3")]] = 0.55
    d.qpos[m.jnt_qposadr[jid("theta4")]] = -0.95
    mujoco.mj_forward(m, d)
    piv = d.xanchor[jid("theta2")]
    bL2 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "Link2")
    com = d.subtree_com[bL2]
    Mh = m.body_subtreemass[bL2]
    dh = float(np.hypot(com[0] - piv[0], com[1] - piv[1]))
    return Mh, dh, Mh * G * dh


def reporte(nombre, momento, d_cw):
    full = M_cw(momento, d_cw, 1.0)
    part = M_cw(momento, d_cw, BAL)
    print(f"--- {nombre} ---  (momento de desbalance = {momento:.2f} N*m, brazo d_cw = {d_cw:.2f} m)")
    print(f"  balance TOTAL (100%):  M_cw = {full:.2f} kg   <- sobrelanza un saltarín, NO usar")
    print(f"  balance ~{int(BAL*100)}% (sweet): M_cw = {part:.2f} kg   <- salto limpio (deja peso efectivo)")
    print(f"  regla rápida:  M_cw = {BAL:.2f} * {momento:.1f} / (9.81 * d_cw)")


if __name__ == "__main__":
    print("============ CONTRAPESO DEL BOOM DE HOPPY ============\n")

    # 1) Modelo de simulación (geometría del CAD HOPPY-E0): se mide directo.
    Mh, dh, mom = medir_en_sim()
    print(f"[SIM] medido: M_hopper={Mh:.2f} kg a d_hopper={dh:.2f} m\n")
    reporte("SIM (brazo cw = 0.35 m, el de tu boom)", mom, 0.35)

    # 2) ROBOT FÍSICO — EDITA con tus medidas reales:
    print()
    FISICO = dict(
        M_hopper=2.075,   # kg  <-- MEDIDO: robot (hopper) conectado al tubo/gantry = 2075 g
        d_hopper=0.85,    # m   <-- medida: pivote -> CoM del hopper (~85 cm)
        d_cw=0.35,        # m   <-- medida: pivote -> contrapeso (35 cm)
    )
    mom_f = momento_desde_masa(FISICO["M_hopper"], FISICO["d_hopper"])
    reporte("FÍSICO (con tus números de arriba)", mom_f, FISICO["d_cw"])
    print("\nNotas:")
    print(" - Lo más exacto: pon el boom en el pivote SIN contrapeso y mide qué peso a 35 cm lo")
    print("   deja horizontal = balance 100%; usa ~76% de eso. O pesa el hopper y mide d_hopper.")
    print(" - d_hopper es al CoM del lado hopper (cerca de los motores/housing), no a la punta.")
    print(" - El sweet spot ~76% depende del empuje (fz). Si en físico sobrelanza, sube el")
    print("   contrapeso o baja el empuje; si no despega, bájalo.")
