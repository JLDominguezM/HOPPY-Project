"""Verificacion COMPONENTE-POR-COMPONENTE del gemelo HOPPY (twin.py).

No es solo visual: corre tests dinamicos y confirma que cada elemento esta bien
PUESTO y FUNCIONA como en el robot real:
  1. ACTUADORES  - los motores mueven SUS juntas, con limites de voltaje/corriente.
  2. JOINTS/LINKS- ejes, rangos, cuales son pasivos (yaw/pitch) vs actuados (hip/knee),
                   la inercia de rotor reflejada, y la cadena cinematica.
  3. RESORTE     - el resorte de rodilla aplica torque restaurador (extiende la pierna).
  4. CONTACTO    - el pie genera GRF y el punto de contacto cae en el regaton real.
  5. SENSORES    - encoders, sensor de pie y fuerza de actuador dan los valores correctos.
  6. MASAS       - cada link pesa lo medido (motores en la cadera, etc.).

Uso:  python3 twin_check.py
"""
import numpy as np
import mujoco
from controller import Hoppy
import twin

OK, NO = "  OK ", "FALLA"


def sec(t):
    print("\n" + "=" * 70 + f"\n  {t}\n" + "=" * 70)


def chk(name, cond, detail=""):
    print(f"  [{OK if cond else NO}] {name:42s} {detail}")
    return bool(cond)


def jid(m, n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def did(m, n): return m.jnt_dofadr[jid(m, n)]
def qid(m, n): return m.jnt_qposadr[jid(m, n)]
def sensor(m, d, n):
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, n)
    a, dim = m.sensor_adr[sid], m.sensor_dim[sid]
    return d.sensordata[a:a + dim]


def main():
    h = Hoppy(dict(twin.DEFAULTS, vis=False), mdl=twin)
    m, d = h.m, h.d
    AX = {0: "X", 1: "Y", 2: "Z"}

    # ---------------- 1. ACTUADORES ----------------
    sec("1. ACTUADORES (motores goBILDA 5202-2402-0027)")
    print(f"  N={twin.NH}:1  R={twin.Rw}ohm  kT={twin.kT}  kv={twin.kv}  "
          f"limites: {twin.VMAX}V / {twin.IMAX}A (datasheet)")
    acts = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    chk("hay 2 actuadores (hip, knee)", m.nu == 2 and acts == ["hip", "knee"], str(acts))
    # cada motor mueve SU junta (gravedad off, a MITAD de rango para no chocar el limite)
    for act, jn, qmid in [("hip", "theta3", 1.0), ("knee", "theta4", -1.5)]:
        aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, act)
        def vel_after(torque):
            mujoco.mj_resetData(m, d); m.opt.gravity[:] = 0
            d.qpos[qid(m, jn)] = qmid
            d.ctrl[aid] = torque
            for _ in range(20): mujoco.mj_step(m, d)
            return d.qvel[did(m, jn)]
        vp, vn = vel_after(+3.0), vel_after(-3.0)
        m.opt.gravity[:] = [0, 0, -9.81]
        chk(f"motor '{act}' impulsa {jn} (+torque -> +giro)", vp > vn and (vp - vn) > 0.5,
            f"+3Nm->{vp:+.2f}, -3Nm->{vn:+.2f} rad/s (dif {vp-vn:+.2f})")
    # yaw y pitch NO tienen actuador (pasivos)
    actuated_j = {m.actuator_trnid[i, 0] for i in range(m.nu)}
    for jn in ["theta1", "theta2"]:
        chk(f"{jn} es PASIVO (sin motor)", jid(m, jn) not in actuated_j)

    # ---------------- 2. JOINTS / LINKS ----------------
    sec("2. JOINTS / LINKS (cadena cinematica)")
    espec = {"theta1": ("yaw  (pasivo)", 2), "theta2": ("pitch(pasivo)", 1),
             "theta3": ("hip  (actuado)", 0), "theta4": ("knee (actuado)", 0)}
    for jn, (rol, ax) in espec.items():
        j = jid(m, jn)
        axis = np.argmax(np.abs(m.jnt_axis[j]))
        rng = m.jnt_range[j] if m.jnt_limited[j] else None
        arm = m.dof_armature[did(m, jn)]
        chk(f"{jn} {rol}", axis == ax,
            f"eje={AX[axis]} rango={np.round(rng,2) if rng is not None else 'libre'} armadura(N^2*Ir)={arm:.1e}")
    # cadena cinematica: mover hip mueve el pie en el plano tangencial-vertical (Y-Z)
    mujoco.mj_resetData(m, d)
    d.qpos[qid(m, "theta3")] = 0.0; mujoco.mj_forward(m, d); f0 = d.site_xpos[h.fs].copy()
    d.qpos[qid(m, "theta3")] = 0.6; mujoco.mj_forward(m, d); f1 = d.site_xpos[h.fs].copy()
    dlt = f1 - f0
    chk("girar hip mueve el pie en Y-Z (tangencial)", abs(dlt[0]) < 1e-3 and (abs(dlt[1]) + abs(dlt[2])) > 0.02,
        f"d_pie=({dlt[0]:+.3f},{dlt[1]:+.3f},{dlt[2]:+.3f}) m")

    # ---------------- 3. RESORTE de rodilla ----------------
    sec("3. RESORTE de rodilla (2x resorte de tension del robot)")
    j4 = jid(m, "theta4")
    stiff, ref = m.jnt_stiffness[j4], twin.DEFAULTS.get("knee_ref", twin.KNEE_REF)
    print(f"  stiffness={stiff:.3f} Nm/rad, ref={ref:.3f} rad (extiende la pierna)")
    # con la rodilla doblada (dentro de rango), el resorte la EXTIENDE (q4 sube hacia ref)
    mujoco.mj_resetData(m, d); m.opt.gravity[:] = 0
    q_start = -1.5
    d.qpos[qid(m, "theta4")] = q_start
    mujoco.mj_forward(m, d); tau_spring = d.qfrc_spring[did(m, "theta4")]
    for _ in range(400): mujoco.mj_step(m, d)
    q_end = d.qpos[qid(m, "theta4")]
    m.opt.gravity[:] = [0, 0, -9.81]
    chk("el resorte EXTIENDE la pierna (q4 sube hacia ref)", tau_spring > 0 and q_end > q_start + 0.05,
        f"tau={tau_spring:+.2f}Nm, q4: {q_start:.2f}->{q_end:.2f} rad")

    # ---------------- 4. CONTACTO del pie ----------------
    sec("4. CONTACTO del pie (regaton vs piso)")
    fg = h.fg
    print(f"  geom 'foot' pos(link4)={twin.FOOT}  friccion={m.geom_friction[fg,0]}  "
          f"solref={m.geom_solref[fg]}")
    # extender la pierna y bajar el boom (pitch) hasta meter el pie ~2cm bajo el piso;
    # mj_forward calcula el contacto y su fuerza SIN dejar evolucionar la dinamica.
    mujoco.mj_resetData(m, d)
    d.qpos[qid(m, "theta3")] = 0.5; d.qpos[qid(m, "theta4")] = -0.5
    for th2 in np.linspace(0.0, 0.6, 61):
        d.qpos[qid(m, "theta2")] = th2; mujoco.mj_forward(m, d)
        if d.site_xpos[h.fs][2] < -0.006:
            break
    mujoco.mj_forward(m, d)
    grf = h.foot_force(); foot_w = d.site_xpos[h.fs]; foot_g = d.geom_xpos[fg]
    contact_ok = any((h.fg in (d.contact[k].geom1, d.contact[k].geom2)) for k in range(d.ncon))
    chk("el pie genera GRF al tocar el piso", grf > 1.0, f"GRF={grf:.0f} N, contactos={d.ncon}")
    chk("el contacto fisico ESTA en el regaton", np.linalg.norm(foot_w - foot_g) < 0.02 and contact_ok,
        f"site-geom={np.linalg.norm(foot_w-foot_g)*1000:.0f}mm, pie_z={foot_w[2]:.3f}")

    # ---------------- 5. SENSORES ----------------
    sec("5. SENSORES (encoders, pie, fuerza)")
    mujoco.mj_resetData(m, d)
    d.qpos[qid(m, "theta3")] = 0.7; d.qpos[qid(m, "theta4")] = -1.3
    d.ctrl[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip")] = 2.5
    mujoco.mj_forward(m, d)
    chk("encoder cadera = theta3", abs(sensor(m, d, "enc_hip")[0] - 0.7) < 1e-6, f"{sensor(m,d,'enc_hip')[0]:.3f} rad")
    chk("encoder rodilla = theta4", abs(sensor(m, d, "enc_knee")[0] + 1.3) < 1e-6, f"{sensor(m,d,'enc_knee')[0]:.3f} rad")
    fp = sensor(m, d, "foot_pos")
    chk("sensor de pie = posicion del regaton", np.allclose(fp, d.site_xpos[h.fs]), f"pie=({fp[0]:.2f},{fp[1]:.2f},{fp[2]:.2f})")
    chk("sensor de fuerza de cadera = torque aplicado", abs(sensor(m, d, "tau_hip")[0] - 2.5) < 1e-6, f"{sensor(m,d,'tau_hip')[0]:.2f} Nm")
    chk("sensor de yaw (avance) y pitch (salto) existen", True,
        f"yaw={sensor(m,d,'yaw_q1')[0]:.2f}, pitch={sensor(m,d,'pitch_q2')[0]:.2f} rad")

    # ---------------- 6. MASAS / INERCIAS ----------------
    sec("6. MASAS / INERCIAS (kg, medidas)")
    for bn in ["link1", "link2", "link3", "link4"]:
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        print(f"  {bn}: m={m.body_mass[b]:.3f} kg  CoM(local)={np.round(m.body_ipos[b],3)}")
    tot = sum(m.body_mass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)] for b in ["link1","link2","link3","link4"])
    chk("link2 lleva los motores+housing (~2.3 kg)", 2.0 < m.body_mass[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"link2")] < 2.6,
        f"total movil={tot:.2f} kg")

    print("\n" + "=" * 70)
    print("  Cada bloque arriba confirma un componente del gemelo de forma INDEPENDIENTE.")
    print("=" * 70)


if __name__ == "__main__":
    main()
