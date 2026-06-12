"""Controlador hibrido de HOPPY, port fiel del simulador MATLAB.

Una sola implementacion compartida por control.py (corrida+graficas),
view.py (visor en vivo) y render.py (video), para que la ley de control no
vuelva a divergir entre archivos.

Replica (rubrica + Simulator_MATLAB):
  Ec.17  Aereo: PD cartesiano del pie en el frame de cadera (boom/link2),
         objetivo p_d = [Krh*vx, -0.15], u = J_hip^T * F_sw.
  Ec.18  Actuador por voltaje + back-EMF + saturacion 12 V / 30 A.
  Ec.19  Apoyo: u = -J_hip^T * [Fx; Fz] (Bezier) + PD suave de junta.
  Ec.20  Blending aereo->apoyo en ~10 ms.
  Fase5  Velocidad por derivada filtrada (lambda ~ 10), emula encoder.
  FSM    touchdown = contacto del pie;  liftoff = GRF_z < 1.5 N (o s>=1).
"""
import numpy as np
import mujoco

import tune_eval

LAMBDA = 10.0   # ancho de banda del filtro de velocidad (Fase 5)


class Hoppy:
    def __init__(self, params=None, mdl=None):
        # mdl = modulo del modelo (tune_eval = abstracto validado; twin = gemelo real).
        # Todas las constantes (make_xml, bezier, motor, RBOOM, DT...) salen de ahi.
        M = mdl if mdl is not None else tune_eval
        self.mdl = M
        self.bezier = M.bezier
        self.FZ_BZ, self.FX_BZ = M.FZ_BZ, M.FX_BZ
        self.N, self.Rw, self.kT, self.kv = M.N, M.Rw, M.kT, M.kv
        self.VMAX, self.IMAX, self.DT, self.RBOOM = M.VMAX, M.IMAX, M.DT, M.RBOOM
        self.p = dict(M.DEFAULTS)
        if params:
            self.p.update(params)
        self.m = mujoco.MjModel.from_xml_string(M.make_xml(self.p))
        self.d = mujoco.MjData(self.m)
        jid = lambda n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qadr = {n: self.m.jnt_qposadr[jid(n)] for n in ("theta1", "theta2", "theta3", "theta4")}
        self.vadr = {n: self.m.jnt_dofadr[jid(n)] for n in ("theta1", "theta2", "theta3", "theta4")}
        self.dof34 = [self.vadr["theta3"], self.vadr["theta4"]]
        self.aid = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ("hip", "knee")}
        self.fs = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
        self.fg = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "foot")
        self.bframe = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "cuerpo_cadera")
        self.reset()

    def reset(self):
        d = self.d
        mujoco.mj_resetData(self.m, d)
        d.qpos[self.qadr["theta3"]] = self.p["q3_ref"]
        d.qpos[self.qadr["theta4"]] = self.p["q4_ref"]
        d.qpos[self.qadr["theta2"]] = -0.06   # arranque: cadera algo arriba, cae suave
        mujoco.mj_forward(self.m, d)
        self.qd_filt = np.zeros(2)
        self.q_prev = np.array([d.qpos[self.qadr["theta3"]], d.qpos[self.qadr["theta4"]]])
        self.phase = "aerial"
        self.t = 0.0
        self.t_td = 0.0
        self.t_lo = -1.0

    # --- utilidades ---
    def foot_force(self):
        f = 0.0
        for k in range(self.d.ncon):
            c = self.d.contact[k]
            if self.fg in (c.geom1, c.geom2):
                f6 = np.zeros(6)
                mujoco.mj_contactForce(self.m, self.d, k, f6)
                f += f6[0]                       # componente normal
        return f

    def _hip_frame(self):
        """R2 (frame del boom/link2) y posicion de la cadera."""
        R2 = self.d.xmat[self.bframe].reshape(3, 3)
        hip = self.d.xpos[self.bframe]
        return R2, hip

    def _foot_jac_hip(self, R2):
        Jw = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, self.d, Jw, None, self.fs)
        Jh = R2.T @ Jw[:, self.dof34]            # 3x2 en frame de cadera (boom)
        # la pierna oscila en el plano TANGENCIAL-vertical (Y-Z del boom):
        # Y = direccion de avance alrededor del poste, Z = vertical
        return Jh[[1, 2], :]                     # filas y,z -> 2x2

    # --- un paso de control (no avanza la fisica) ---
    def control_step(self):
        d, p = self.d, self.p
        q34 = np.array([d.qpos[self.qadr["theta3"]], d.qpos[self.qadr["theta4"]]])
        qd_real = np.array([d.qvel[self.dof34[0]], d.qvel[self.dof34[1]]])
        # velocidad por derivada filtrada (Fase 5, emula encoder 28 CPR)
        af = LAMBDA * self.DT / (1 + LAMBDA * self.DT)
        self.qd_filt += af * ((q34 - self.q_prev) / self.DT - self.qd_filt)
        self.q_prev = q34.copy()
        qf = self.qd_filt

        R2, hip = self._hip_frame()
        foot = d.site_xpos[self.fs]
        p_hip = R2.T @ (foot - hip)
        p_xz = np.array([p_hip[1], p_hip[2]])    # [tangencial Y, vertical Z]
        Jhip = self._foot_jac_hip(R2)
        v_xz = Jhip @ qf

        # --- AEREO (Ec.17) ---
        # vx = velocidad tangencial (avance alrededor del poste) = dtheta1 * Rboom.
        # vx_d = velocidad de avance DESEADA (Raibert): con vx_d=0 el pie se coloca para
        # llevar vx->0 (salta en sitio); con vx_d!=0 regula vx hacia vx_d (AVANZA). El
        # default 0 deja el comportamiento original/MATLAB intacto.
        vx = d.qvel[self.vadr["theta1"]] * self.RBOOM
        p_d = np.array([p["krh"] * (vx - p.get("vx_d", 0.0)), p["p_toe_z"]])
        F_sw = p["kp_sw"] * (p_d - p_xz) + p["kd_sw"] * (-v_xz)
        u_air = Jhip.T @ F_sw

        # --- APOYO (Ec.19) ---
        s = min(max((self.t - self.t_td) / p["Tst"], 0.0), 1.0)
        Fz = self.bezier(self.FZ_BZ, s) * p["fz_scale"]
        Fx = self.bezier(self.FX_BZ, s) * p["fx_scale"]
        q_d = np.array([p["q3_ref"], p["q4_ref"]])
        tau_fb = p["kp_st"] * (q_d - q34) + p["kd_st"] * (-qf)
        u_st = -Jhip.T @ np.array([Fx, Fz]) + tau_fb

        # --- blending (Ec.20) ---
        if self.phase == "stance":
            al = min(1.0, (self.t - self.t_td) / p["blend"])
            u = al * u_st + (1 - al) * u_air
        else:
            u = u_air

        # --- voltaje + back-EMF + saturacion (Ec.18) ---
        N, Rw, kT, kv = self.N, self.Rw, self.kT, self.kv
        V = (Rw / (kT * N)) * u + kv * N * qf
        V = np.clip(V, -self.VMAX, self.VMAX)
        i = np.clip((V - kv * N * qd_real) / Rw, -self.IMAX, self.IMAX)
        tau = kT * N * i
        d.ctrl[self.aid["hip"]], d.ctrl[self.aid["knee"]] = tau

        # --- FSM ---
        grf = self.foot_force()
        foot_z = foot[2]
        contact = (grf > 1.0) or (foot_z <= 0.013)
        if self.phase == "aerial":
            if contact and (self.t - self.t_lo) > 0.02:
                self.phase = "stance"
                self.t_td = self.t
        else:  # stance
            ramped = (self.t - self.t_td) > 0.03
            if (ramped and grf < p["grf_liftoff"]) or (self.t - self.t_td) >= p["Tst"]:
                self.phase = "aerial"
                self.t_lo = self.t

        return dict(t=self.t, phase=(1 if self.phase == "stance" else 0),
                    body_z=hip[2], foot_z=foot_z,
                    theta1=d.qpos[self.qadr["theta1"]], theta2=d.qpos[self.qadr["theta2"]],
                    q3=q34[0], q4=q34[1], grf=grf, Fz_des=Fz, Fx_des=Fx,
                    tau3=tau[0], tau4=tau[1], V3=V[0], V4=V[1], i3=i[0], i4=i[1],
                    qd3_real=qd_real[0], qd3_filt=qf[0])

    def step(self):
        log = self.control_step()
        mujoco.mj_step(self.m, self.d)
        self.t += self.DT
        return log


def simulate(params=None, t_total=8.0, mdl=None):
    """Corre la simulacion y devuelve los logs como arrays."""
    h = Hoppy(params, mdl=mdl)
    DT = h.DT
    keys = None
    L = {}
    nan = False
    for _ in range(int(t_total / DT)):
        rec = h.step()
        if keys is None:
            keys = list(rec)
            L = {k: [] for k in keys}
        for k in keys:
            L[k].append(rec[k])
        if np.any(np.isnan(h.d.qpos)):
            nan = True
            break
    out = {k: np.array(v) for k, v in L.items()}
    out["_nan"] = nan
    return out
