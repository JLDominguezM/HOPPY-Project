"""hop_controller.py - controlador de salto FSM simple para el HOPPY URDF real.

INDEPENDIENTE de controller.py (el control hibrido del twin NO transfiere al URDF; ~800
configs de busqueda dieron 0 cm). Basado en el test mecanico: torque directo hip=-, knee=+
desde cuclillas -> el cuerpo sube +23.6 cm. Aqui se coordina ese empuje en ciclo.

FSM de 3 estados:
  0 CARGA  : PD a la pose agachada (q3_crouch, q4_crouch). Pasa a EMPUJE cuando
             t_estado > T_crouch Y hay contacto (GRF > umbral_grf).
  1 EMPUJE : torque directo hip = -TAU_HIP, knee = +TAU_KNEE (la direccion que salta).
             Pasa a VUELO cuando despega (GRF < umbral_liftoff) o t_estado > T_push.
  2 VUELO  : PD a la pose neutra (q3_land, q4_land). Pasa a CARGA al aterrizar
             (GRF > umbral_touchdown).

Mapa verificado (gear=1): ctrl+ -> +theta en hip y knee, asi que PD estandar
(tau = kp*(q*-q) - kd*qd) y EMPUJE usa hip=-TAU_HIP, knee=+TAU_KNEE.

Arquitectura: recibe (model, data) de afuera; step() calcula control y fija d.ctrl pero
NO llama mj_step (lo hace el loop externo). Contacto via mj_contactForce (geom 'foot'),
no el sensor touch. Limita el torque al ctrlrange real del modelo.
"""
import numpy as np
import mujoco

CARGA, EMPUJE, VUELO = 0, 1, 2
_NAME = {CARGA: "CARGA", EMPUJE: "EMPUJE", VUELO: "VUELO"}

HOP_DEFAULTS = dict(
    # crouch MENOS profundo (q4_crouch>=-1.0 evita el limite -1.3 y que Link4 atraviese el
    # piso) + TAU mas bajo y cadencia mas lenta (anti bang-bang) + T_vuelo_min (cadencia).
    T_crouch=0.141, T_push=0.192,
    q3_crouch=0.65, q4_crouch=-0.85,        # barrido: mas profundo limpio (Link4_min=0.115>0, ~11 cm)
    q3_land=0.50, q4_land=-0.60,
    TAU_HIP=3.5, TAU_KNEE=3.5,
    kp_flight=80.0, kd_flight=2.0,
    T_vuelo_min=0.08,                        # tiempo minimo en VUELO antes de re-cargar
    umbral_grf=5.0, umbral_liftoff=2.0, umbral_touchdown=5.0,
)


class HopController:
    def __init__(self, params, model, data):
        self.m, self.d = model, data
        self.p = dict(HOP_DEFAULTS)
        if params:
            self.p.update(params)
        self.DT = float(model.opt.timestep)
        jid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.q = {n: model.jnt_qposadr[jid(n)] for n in ("theta1", "theta2", "theta3", "theta4")}
        self.v = {n: model.jnt_dofadr[jid(n)] for n in ("theta1", "theta2", "theta3", "theta4")}
        self.aid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ("hip", "knee")}
        self.fg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot")
        # limite real del actuador (ctrlrange) para no comandar fuera de rango
        a_hip = self.aid["hip"]
        self.ctrl_lim = float(model.actuator_ctrlrange[a_hip][1]) if model.actuator_ctrllimited[a_hip] else 1e3
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[self.q["theta3"]] = self.p["q3_crouch"]
        self.d.qpos[self.q["theta4"]] = self.p["q4_crouch"]
        mujoco.mj_forward(self.m, self.d)
        self.state = CARGA
        self.t = 0.0
        self.t_state = 0.0

    def foot_force(self):
        f = 0.0
        for k in range(self.d.ncon):
            c = self.d.contact[k]
            if self.fg in (c.geom1, c.geom2):
                f6 = np.zeros(6)
                mujoco.mj_contactForce(self.m, self.d, k, f6)
                f += f6[0]                     # componente normal
        return f

    def _pd(self, q3t, q4t):
        p, d = self.p, self.d
        kp, kd = p["kp_flight"], p["kd_flight"]
        tau_h = kp * (q3t - d.qpos[self.q["theta3"]]) - kd * d.qvel[self.v["theta3"]]
        tau_k = kp * (q4t - d.qpos[self.q["theta4"]]) - kd * d.qvel[self.v["theta4"]]
        return tau_h, tau_k

    def step(self):
        p, d = self.p, self.d
        grf = self.foot_force()
        if self.state == CARGA:
            tau_h, tau_k = self._pd(p["q3_crouch"], p["q4_crouch"])
            if self.t_state > p["T_crouch"] and grf > p["umbral_grf"]:
                self.state = EMPUJE
                self.t_state = 0.0
        elif self.state == EMPUJE:
            tau_h, tau_k = -p["TAU_HIP"], +p["TAU_KNEE"]    # direccion probada del salto
            if grf < p["umbral_liftoff"] or self.t_state > p["T_push"]:
                self.state = VUELO
                self.t_state = 0.0
        else:  # VUELO
            tau_h, tau_k = self._pd(p["q3_land"], p["q4_land"])
            if grf > p["umbral_touchdown"] and self.t_state > p.get("T_vuelo_min", 0.08):
                self.state = CARGA
                self.t_state = 0.0
        lim = self.ctrl_lim
        d.ctrl[self.aid["hip"]] = float(np.clip(tau_h, -lim, lim))
        d.ctrl[self.aid["knee"]] = float(np.clip(tau_k, -lim, lim))
        self.t += self.DT
        self.t_state += self.DT
        return dict(t=self.t, estado=_NAME[self.state], grf=grf,
                    tau_hip=float(d.ctrl[self.aid["hip"]]), tau_knee=float(d.ctrl[self.aid["knee"]]),
                    q3=float(d.qpos[self.q["theta3"]]), q4=float(d.qpos[self.q["theta4"]]),
                    theta1=float(d.qpos[self.q["theta1"]]), theta2=float(d.qpos[self.q["theta2"]]))
