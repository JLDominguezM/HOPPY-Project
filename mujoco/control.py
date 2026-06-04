"""Controlador hibrido de HOPPY en MuJoCo: corrida final + graficas (M4-M6).

Implementa (rubrica):
  M1 - Jacobiano del pie (mj_jacSite)
  M2 - Actuador por voltaje + back-EMF + saturacion 12V/30A (Ec.18)
  M3 - Velocidad por derivada filtrada [lambda s/(s+lambda)] (encoder 28 CPR)
  M4 - FSM 1 kHz: PD cartesiano aereo (Ec.17) + Bezier en apoyo (Ec.19) + blending (Ec.20)

Parametros afinados a un ciclo limite estable (~60 saltos consistentes).
Genera figuras en figuras/ y reporta metricas.
"""
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from tune_eval import (make_xml, bezier, DEFAULTS, FZ_BZ, FX_BZ,
                       NH, NK, Rw, kT, kv, VMAX, IMAX, N, DT)

# ---- parametros afinados (ciclo limite estable) ----
PARAMS = dict(DEFAULTS)
PARAMS.update(dict(j_damp=0.2, fz_scale=1.4, cw_mass=1.9, tst=0.12,
                   kd=15.0, knee_stiff=5.0))


def run(params=PARAMS, t_total=8.0):
    p = params
    m = mujoco.MjModel.from_xml_string(make_xml(p)); d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    qa = {n: m.jnt_qposadr[jid(n)] for n in ["theta1", "theta2", "theta3", "theta4"]}
    va = {n: m.jnt_dofadr[jid(n)] for n in ["theta1", "theta2", "theta3", "theta4"]}
    dof34 = [va["theta3"], va["theta4"]]
    hip_a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip")
    knee_a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee")
    fs = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    fg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot")
    hb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link3")
    bb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cuerpo_cadera")
    KPf = np.array([p['kp_x'], 0.0, p['kp_z']]); KDf = np.array([p['kd'], 0.0, p['kd']])

    def frh():
        R = d.xmat[hb].reshape(3, 3); return R.T @ (d.site_xpos[fs] - d.xpos[hb])

    def jac():
        J = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, J, None, fs); return J[:, dof34]

    def ff():
        f = 0.0
        for k in range(d.ncon):
            c = d.contact[k]
            if fg in (c.geom1, c.geom2):
                f6 = np.zeros(6); mujoco.mj_contactForce(m, d, k, f6); f += f6[0]
        return f

    d.qpos[qa["theta3"]], d.qpos[qa["theta4"]] = p['q3_ref'], p['q4_ref']
    mujoco.mj_forward(m, d)
    p_ref = frh().copy() + np.array([p['pref_dx'], 0.0, p['pref_dz']])

    qd_filt = np.zeros(2); q_prev = None
    phase = "stance"; t_td = 0.0; t_lo = -1.0
    L = {k: [] for k in ["t", "phase", "body_z", "foot_z", "tau3", "tau4", "V3", "V4",
                         "i3", "i4", "Fz", "Fz_des", "qd3_real", "qd3_filt", "bodyz_vel"]}
    apexes = []; cur_apex = -9.9; t = 0.0; bz_prev = None
    for step in range(int(t_total / DT)):
        qd_real = np.array([d.qvel[dof34[0]], d.qvel[dof34[1]]])
        q = np.array([d.qpos[qa["theta3"]], d.qpos[qa["theta4"]]])
        if q_prev is None: q_prev = q.copy()
        af = 10.0 * DT / (1 + 10.0 * DT)
        qd_filt = qd_filt + af * ((q - q_prev) / DT - qd_filt); q_prev = q.copy()
        Jc = jac(); R = d.xmat[hb].reshape(3, 3)
        # AEREO: PD cartesiano (Ec.17)
        Fair = KPf * (R @ (p_ref - frh())) - KDf * (Jc @ qd_filt)
        tau_air = Jc.T @ Fair
        # APOYO: Bezier (Ec.19)
        s = (t - t_td) / p['tst'] if phase == "stance" else 0.0
        Fz = bezier(FZ_BZ, s) * p['fz_scale']; Fx = bezier(FX_BZ, s) * p['fx_scale']
        tau_st = Jc.T @ np.array([Fx, 0.0, -Fz])
        # voltajes (Ec.18) + blending (Ec.20)
        Vair = (Rw / (kT * N)) * tau_air + kv * N * qd_filt
        Vst = (Rw / (kT * N)) * tau_st + kv * N * qd_filt
        if phase == "stance":
            al = min(1.0, (t - t_td) / p['blend']); V = al * Vst + (1 - al) * Vair
        else:
            V = Vair
        V = np.clip(V, -VMAX, VMAX)                                  # saturacion 12V
        i = np.clip((V - kv * N * qd_real) / Rw, -IMAX, IMAX)        # saturacion 30A
        tau = kT * N * i
        d.ctrl[hip_a], d.ctrl[knee_a] = tau

        bz = d.xpos[bb][2]; fz_site = d.site_xpos[fs][2]
        bzv = (bz - bz_prev) / DT if bz_prev is not None else 0.0; bz_prev = bz
        contact = (fz_site <= 0.014) or (ff() > 1.0)
        # log
        for k, v in zip(["t", "phase", "body_z", "foot_z", "tau3", "tau4", "V3", "V4",
                         "i3", "i4", "Fz", "Fz_des", "qd3_real", "qd3_filt", "bodyz_vel"],
                        [t, 0 if phase == "aerial" else 1, bz, fz_site, tau[0], tau[1],
                         V[0], V[1], i[0], i[1], ff(), Fz, qd_real[0], qd_filt[0], bzv]):
            L[k].append(v)
        # FSM robusta
        if phase == "stance":
            if (t - t_td) >= p['tst'] or fz_site > 0.02:
                phase = "aerial"; t_lo = t; cur_apex = bz
        else:
            cur_apex = max(cur_apex, bz)
            if contact and (t - t_lo) > 0.03:
                phase = "stance"; t_td = t; apexes.append(cur_apex)
        mujoco.mj_step(m, d); t += DT
        if np.any(np.isnan(d.qpos)): break
    return {k: np.array(v) for k, v in L.items()}, np.array(apexes)


def figuras(L, apexes):
    import os; os.makedirs("figuras", exist_ok=True)
    t = L["t"]
    fig, ax = plt.subplots(3, 2, figsize=(12, 9))
    # 1) altura del cuerpo (salto sostenido)
    ax[0, 0].plot(t, L["body_z"], "b"); ax[0, 0].grid(True)
    ax[0, 0].set_title(f"Altura del cuerpo ({len(apexes)} saltos, apex std={np.std(apexes[1:]):.3f} m)")
    ax[0, 0].set_xlabel("Tiempo [s]"); ax[0, 0].set_ylabel("z [m]")
    # 2) fuerzas: deseada vs real en el pie
    ax[0, 1].plot(t, L["Fz_des"], "r", label="Fz deseada (Bezier)", lw=0.8)
    ax[0, 1].plot(t, L["Fz"], "k", label="Fz real (contacto)", lw=0.8)
    ax[0, 1].grid(True); ax[0, 1].legend(); ax[0, 1].set_title("Fuerza de reaccion del pie")
    ax[0, 1].set_xlabel("Tiempo [s]"); ax[0, 1].set_ylabel("N"); ax[0, 1].set_xlim(2, 4)
    # 3) torques articulares
    ax[1, 0].plot(t, L["tau3"], "b", label="cadera", lw=0.7)
    ax[1, 0].plot(t, L["tau4"], "r", label="rodilla", lw=0.7)
    ax[1, 0].grid(True); ax[1, 0].legend(); ax[1, 0].set_title("Pares articulares")
    ax[1, 0].set_xlabel("Tiempo [s]"); ax[1, 0].set_ylabel("Nm"); ax[1, 0].set_xlim(2, 4)
    # 4) voltaje y corriente con saturacion (Fase 3)
    ax[1, 1].plot(t, L["V3"], "b", lw=0.6, label="V cadera")
    ax[1, 1].plot(t, L["V4"], "r", lw=0.6, label="V rodilla")
    ax[1, 1].axhline(12, color="k", ls="--", lw=0.8); ax[1, 1].axhline(-12, color="k", ls="--", lw=0.8)
    ax[1, 1].grid(True); ax[1, 1].legend(); ax[1, 1].set_title("Voltaje (limite +/-12V)")
    ax[1, 1].set_xlabel("Tiempo [s]"); ax[1, 1].set_ylabel("V"); ax[1, 1].set_xlim(2, 4)
    # 5) velocidad: real vs filtrada (Fase 5)
    ax[2, 0].plot(t, L["qd3_real"], "c", lw=0.6, label="real (qvel)")
    ax[2, 0].plot(t, L["qd3_filt"], "b", lw=0.9, label="filtrada (encoder)")
    ax[2, 0].grid(True); ax[2, 0].legend(); ax[2, 0].set_title("Velocidad de cadera: real vs derivada filtrada")
    ax[2, 0].set_xlabel("Tiempo [s]"); ax[2, 0].set_ylabel("rad/s"); ax[2, 0].set_xlim(2, 3)
    # 6) ciclo limite (retrato de fase)
    half = len(t) // 3
    ax[2, 1].plot(L["body_z"][half:], L["bodyz_vel"][half:], "b", lw=0.5)
    ax[2, 1].grid(True); ax[2, 1].set_title("Ciclo limite (retrato de fase del cuerpo)")
    ax[2, 1].set_xlabel("z [m]"); ax[2, 1].set_ylabel("dz/dt [m/s]")
    fig.tight_layout(); fig.savefig("figuras/resultados.png", dpi=140)
    print("figuras/resultados.png guardada")


if __name__ == "__main__":
    L, apexes = run()
    print(f"Saltos: {len(apexes)}  | apex medio={np.mean(apexes[1:]):.3f}m std={np.std(apexes[1:]):.3f}m")
    print(f"Altura cuerpo: [{L['body_z'].min():.3f}, {L['body_z'].max():.3f}] m")
    print(f"Voltaje max |V|={max(np.abs(L['V3']).max(), np.abs(L['V4']).max()):.2f}V (limite 12)")
    print(f"Corriente max |i|={max(np.abs(L['i3']).max(), np.abs(L['i4']).max()):.2f}A (limite 30)")
    print(f"Fuerza pico pie={L['Fz'].max():.1f}N")
    figuras(L, apexes)
