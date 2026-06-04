"""Controlador hibrido de HOPPY en MuJoCo (Fases 2-5 de la rubrica).

M1 - Jacobiano del pie (mj_jacSite)
M2 - Actuador por voltaje + back-EMF + saturacion 12V/30A (Ec. 18, guia tecnica)
M3 - Velocidad por derivada filtrada [lambda s/(s+lambda)] (emula encoder 28 CPR)
M4 - FSM 1 kHz: PD cartesiano aereo (Ec.17) + perfil Bezier en apoyo (Ec.19)
     + blending de voltajes 10 ms (Ec.20)
"""
import mujoco
import numpy as np

# ----------------------- parametros fisicos -----------------------
NH, NK = 26.9, 28.8
Rw, kT, kv = 1.3, 0.0135, 0.0186
VMAX, IMAX = 12.0, 30.0
N = np.array([NH, NK])

# ----------------------- parametros de control --------------------
KP_CART = np.array([800.0, 800.0])   # PD cartesiano aereo (N/m)
KD_CART = np.array([15.0, 15.0])     # (N s/m)
TST = 0.15                           # duracion del apoyo (s)
BLEND_T = 0.010                      # blending tras touchdown (s)
LAM = 10.0                           # constante del filtro de velocidad
DT = 0.001                           # 1 ms -> 1 kHz

# coeficientes Bezier del perfil de fuerza (get_params.m)
FZ_BZ = np.array([0.0, 20.0, 100.0, 0.0, 0.0])   # vertical
FX_BZ = np.array([0.0,  0.0, -25.0, 0.0, 0.0])   # horizontal

Q_REF = np.array([np.pi/3, -np.pi/2])  # postura de la pierna en vuelo


def bezier(coef, s):
    """Evalua una curva de Bezier de grado n=len(coef)-1 en s en [0,1]."""
    n = len(coef) - 1
    from math import comb
    s = np.clip(s, 0.0, 1.0)
    return sum(comb(n, i) * coef[i] * s**i * (1 - s)**(n - i) for i in range(n + 1))


class Hoppy:
    def __init__(self, xml="hoppy.xml"):
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        m = self.m
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qa = {n: m.jnt_qposadr[jid(n)] for n in ["theta1", "theta2", "theta3", "theta4"]}
        self.va = {n: m.jnt_dofadr[jid(n)] for n in ["theta1", "theta2", "theta3", "theta4"]}
        self.dof34 = [self.va["theta3"], self.va["theta4"]]
        self.hip_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip")
        self.knee_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee")
        self.foot_sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
        self.foot_gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot")
        self.hip_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link3")
        self.body_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cuerpo_cadera")
        # estado del estimador de velocidad filtrada (M3)
        self.q_prev = None
        self.qd_filt = np.zeros(2)
        # config nominal -> p_ref del pie respecto a la cadera (M1/M4)
        self.p_ref = None

    # ---------------- M1: cinematica y Jacobiano del pie ----------------
    def foot_rel_hip(self):
        d = self.d
        Rhip = d.xmat[self.hip_bid].reshape(3, 3)
        return Rhip.T @ (d.site_xpos[self.foot_sid] - d.xpos[self.hip_bid])

    def foot_jac(self):
        """Jc (3x2) del pie respecto a las juntas (theta3, theta4), en marco mundo."""
        jacp = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, self.d, jacp, None, self.foot_sid)
        return jacp[:, self.dof34]               # 3x2

    # ---------------- M3: velocidad por derivada filtrada ----------------
    def filtered_qd(self):
        q = np.array([self.d.qpos[self.qa["theta3"]], self.d.qpos[self.qa["theta4"]]])
        if self.q_prev is None:
            self.q_prev = q.copy()
        raw = (q - self.q_prev) / DT
        a = LAM * DT / (1 + LAM * DT)           # filtro pasa bajas discreto lambda s/(s+lambda)
        self.qd_filt = self.qd_filt + a * (raw - self.qd_filt)
        self.q_prev = q.copy()
        return self.qd_filt

    # ---------------- M2: actuador por voltaje + back-EMF + saturacion ----
    def torque_from_voltage(self, V, qd_real):
        """De voltaje comandado a torque real aplicado (Ec.5-6,18). Satura 12V/30A."""
        V = np.clip(V, -VMAX, VMAX)             # saturacion de voltaje
        i = (V - kv * N * qd_real) / Rw         # corriente (con back-EMF)
        i = np.clip(i, -IMAX, IMAX)             # saturacion de corriente
        tau = kT * N * i                        # torque electromagnetico en la junta
        return tau, V, i

    def voltage_from_torque(self, tau_des, qd_est):
        """Ec.18: torque deseado -> voltaje, compensando back-EMF (vel. estimada)."""
        return (Rw / (kT * N)) * tau_des + kv * N * qd_est

    # ---------------- contacto ----------------
    def foot_force(self):
        fz = 0.0
        for k in range(self.d.ncon):
            c = self.d.contact[k]
            if self.foot_gid in (c.geom1, c.geom2):
                f6 = np.zeros(6)
                mujoco.mj_contactForce(self.m, self.d, k, f6)
                fz += f6[0]
        return fz

    def in_contact(self):
        return self.d.site_xpos[self.foot_sid][2] <= 0.013 or self.foot_force() > 1.0

    # ---------------- M4: simulacion del salto ----------------
    def run(self, n_hops=8):
        m, d = self.m, self.d
        d.qpos[self.qa["theta3"]], d.qpos[self.qa["theta4"]] = Q_REF
        mujoco.mj_forward(m, d)
        self.p_ref = self.foot_rel_hip().copy()    # pie nominal (postura aerea)

        phase = "stance"   # arranca en el suelo: primer empujon para lanzar
        t_td = 0.0         # tiempo de touchdown
        hops = 0
        airborne = False   # latch: exige despegar antes de un nuevo touchdown
        log = {k: [] for k in ["t", "phase", "body_z", "foot_z", "tau3", "tau4",
                               "V3", "V4", "i3", "i4", "Fz", "qd3_real", "qd3_filt"]}
        t = 0.0
        max_steps = int(n_hops * 1.5 / DT) + 2000
        for step in range(max_steps):
            qd_real = np.array([d.qvel[self.dof34[0]], d.qvel[self.dof34[1]]])
            qd_est = self.filtered_qd()             # M3 (la usa el control)
            Jc = self.foot_jac()                    # M1

            # ---- controladores: torque deseado en theta3,theta4 ----
            # AEREO: PD cartesiano (Ec.17), pie respecto a cadera
            p_foot = self.foot_rel_hip()
            Rhip = d.xmat[self.hip_bid].reshape(3, 3)
            err_w = Rhip @ (self.p_ref - p_foot)            # error en mundo
            vfoot_w = Jc @ qd_est                            # vel del pie (vel estimada)
            F_air = KP_CART_full * err_w - KD_CART_full * vfoot_w
            tau_air = Jc.T @ F_air

            # APOYO: perfil Bezier de fuerza (Ec.19)
            s = (t - t_td) / TST if phase == "stance" else 0.0
            Fz = bezier(FZ_BZ, s)
            Fx = bezier(FX_BZ, s)
            F_st = np.array([Fx, 0.0, -Fz])                  # GRF: el pie empuja ABAJO -> reaccion lanza el cuerpo
            tau_st = Jc.T @ F_st

            # ---- voltajes (Ec.18) ----
            V_air = self.voltage_from_torque(tau_air, qd_est)
            V_st = self.voltage_from_torque(tau_st, qd_est)

            # ---- blending de voltajes (Ec.20) ----
            if phase == "stance":
                alpha = min(1.0, (t - t_td) / BLEND_T)
                V = alpha * V_st + (1 - alpha) * V_air
            else:
                V = V_air

            # ---- torque real con saturacion 12V/30A (M2) ----
            tau, Vsat, isat = self.torque_from_voltage(V, qd_real)
            d.ctrl[self.hip_act] = tau[0]
            d.ctrl[self.knee_act] = tau[1]

            # ---- FSM (con latch de despegue para no castanetear) ----
            foot_z = d.site_xpos[self.foot_sid][2]
            if phase == "aerial":
                if foot_z > 0.05:
                    airborne = True
                if airborne and self.in_contact():
                    phase = "stance"; t_td = t; airborne = False
            elif phase == "stance" and (t - t_td) >= TST:
                phase = "aerial"; hops += 1

            # ---- log ----
            log["t"].append(t); log["phase"].append(0 if phase == "aerial" else 1)
            log["body_z"].append(d.xpos[self.body_bid][2])
            log["foot_z"].append(d.site_xpos[self.foot_sid][2])
            log["tau3"].append(tau[0]); log["tau4"].append(tau[1])
            log["V3"].append(Vsat[0]); log["V4"].append(Vsat[1])
            log["i3"].append(isat[0]); log["i4"].append(isat[1])
            log["Fz"].append(self.foot_force())
            log["qd3_real"].append(qd_real[0]); log["qd3_filt"].append(qd_est[0])

            mujoco.mj_step(m, d)
            t += DT
            if np.any(np.isnan(d.qpos)):
                print(f"  ** inestable en t={t:.3f} **"); break
        return {k: np.array(v) for k, v in log.items()}, hops


# las ganancias cartesianas actuan en 3D (plano de la pierna)
KP_CART_full = np.array([KP_CART[0], 0.0, KP_CART[1]])
KD_CART_full = np.array([KD_CART[0], 0.0, KD_CART[1]])


if __name__ == "__main__":
    h = Hoppy()
    print("Corriendo control hibrido...")
    log, hops = h.run(n_hops=8)
    bz = log["body_z"]
    print(f"Saltos detectados: {hops}")
    print(f"Altura del cuerpo: min={bz.min():.3f} max={bz.max():.3f} rango={bz.max()-bz.min():.3f} m")
    print(f"Voltaje max |V3|={np.abs(log['V3']).max():.2f}V |V4|={np.abs(log['V4']).max():.2f}V (limite 12)")
    print(f"Corriente max |i3|={np.abs(log['i3']).max():.2f}A |i4|={np.abs(log['i4']).max():.2f}A (limite 30)")
    print(f"Fuerza pico en pie: {log['Fz'].max():.1f} N")
    print(f"Fases: {int(log['phase'].sum()*DT*1000)} ms en apoyo de {log['t'][-1]*1000:.0f} ms totales")
