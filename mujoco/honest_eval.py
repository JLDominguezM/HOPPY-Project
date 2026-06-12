"""Metrica HONESTA: cuenta solo saltos reales (pie despega >4cm) sin colapso."""
import numpy as np, mujoco
from tune_eval import make_xml, bezier, FZ_BZ, FX_BZ, DEFAULTS, NH,NK,Rw,kT,kv,VMAX,IMAX,N,DT

def honest(up, t_total=8.0):
    p=dict(DEFAULTS); p.update(up)
    m=mujoco.MjModel.from_xml_string(make_xml(p)); d=mujoco.MjData(m)
    jid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)
    qa={n:m.jnt_qposadr[jid(n)] for n in["theta1","theta2","theta3","theta4"]}
    va={n:m.jnt_dofadr[jid(n)] for n in["theta3","theta4"]};dof=[va["theta3"],va["theta4"]]
    ha=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,"hip");ka=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,"knee")
    fs=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,"foot_site");fg=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"foot")
    hb=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"link3");bb=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"cuerpo_cadera")
    def frh():R=d.xmat[hb].reshape(3,3);return R.T@(d.site_xpos[fs]-d.xpos[hb])
    def jac():J=np.zeros((3,m.nv));mujoco.mj_jacSite(m,d,J,None,fs);return J[:,dof]
    def ff():
        f=0.
        for k in range(d.ncon):
            c=d.contact[k]
            if fg in(c.geom1,c.geom2):f6=np.zeros(6);mujoco.mj_contactForce(m,d,k,f6);f+=f6[0]
        return f
    d.qpos[qa["theta3"]],d.qpos[qa["theta4"]]=p['q3_ref'],p['q4_ref'];mujoco.mj_forward(m,d)
    pref=frh().copy()+np.array([p['pref_dx'],0,p['pref_dz']])
    KPf=np.array([p['kp_x'],0,p['kp_z']]);KDf=np.array([p['kd'],0,p['kd']])
    qf=np.zeros(2);qp=None;phase="stance";ttd=0.;tlo=-1.;t=0.
    real_clears=[];cur_clear=0.;th2_log=[];bz_log=[];footz_log=[];force_log=[]
    for k in range(int(t_total/DT)):
        qr=np.array([d.qvel[dof[0]],d.qvel[dof[1]]]);q=np.array([d.qpos[qa["theta3"]],d.qpos[qa["theta4"]]])
        if qp is None:qp=q.copy()
        af=10*DT/(1+10*DT);qf=qf+af*((q-qp)/DT-qf);qp=q.copy();J=jac();R=d.xmat[hb].reshape(3,3)
        ta=J.T@(KPf*(R@(pref-frh()))-KDf*(J@qf))
        s=(t-ttd)/p['tst'] if phase=="stance" else 0.;Fz=bezier(FZ_BZ,s)*p['fz_scale'];Fx=bezier(FX_BZ,s)*p['fx_scale']
        ts=J.T@np.array([Fx,0,-Fz]);Va=(Rw/(kT*N))*ta+kv*N*qf;Vs=(Rw/(kT*N))*ts+kv*N*qf
        V=(min(1,(t-ttd)/p['blend'])*Vs+(1-min(1,(t-ttd)/p['blend']))*Va) if phase=="stance" else Va
        V=np.clip(V,-VMAX,VMAX);i=np.clip((V-kv*N*qr)/Rw,-IMAX,IMAX);d.ctrl[ha],d.ctrl[ka]=kT*N*i
        fz=d.site_xpos[fs][2];con=(fz<=0.014)or(ff()>1)
        if phase=="stance":
            if(t-ttd)>=p['tst'] or fz>0.02:phase="aerial";tlo=t;cur_clear=fz
        else:
            cur_clear=max(cur_clear,fz)
            if con and(t-tlo)>0.03:
                phase="stance";ttd=t;real_clears.append(cur_clear)
        th2_log.append(abs(d.qpos[qa["theta2"]]));bz_log.append(d.xpos[bb][2])
        footz_log.append(d.site_xpos[fs][2]);force_log.append(ff())
        mujoco.mj_step(m,d);t+=DT
        if np.any(np.isnan(d.qpos)):break
    th2=np.array(th2_log);bz=np.array(bz_log);rc=np.array(real_clears)
    fzz=np.array(footz_log);frc=np.array(force_log)
    ss=slice(int(3/DT),None)
    collapse = bz[ss].mean()<0.10 or th2[ss].mean()>0.40
    real_hops = int(np.sum(rc>0.04))
    # body_hop_cm = excursion REAL del cuerpo (no la pierna) en regimen
    body_hop_cm = float((bz[ss].max()-bz[ss].min())*100) if bz[ss].size else 0.0
    return dict(real_hops=real_hops, max_clear_mm=float(rc.max()*1000) if len(rc) else 0,
                body_z_ss=float(bz[ss].mean()), body_hop_cm=body_hop_cm,
                th2_ss=float(th2[ss].mean()), collapse=bool(collapse),
                peak_force_N=float(frc.max()) if frc.size else 0.0,
                min_foot_z_mm=float(fzz.min()*1000),    # <0 => el pie penetra el piso
                t_end=float(t))

if __name__=="__main__":
    import itertools, json
    from multiprocessing import Pool
    # FACTIBILIDAD: ¿algun cambio da saltos reales sin colapso?
    grid=dict(knee_stiff=[3,15,40,80], spring_ref=[-1.8,-1.4,-1.0], fz_scale=[1.0,2.0,3.5],
              cw_mass=[1.6,2.0,2.4], post_h=[0.20,0.26])
    keys=list(grid);combos=[dict(zip(keys,v)) for v in itertools.product(*grid.values())]
    def ev(p):
        try:r=honest(p);r["p"]=p;return r
        except: return {"real_hops":-1,"p":p}
    with Pool(6) as pool:res=pool.map(ev,combos)
    res=[r for r in res if r];res.sort(key=lambda r:(not r["collapse"], r["real_hops"], r["max_clear_mm"]),reverse=True)
    print(f"{len(combos)} combos. TOP 10 por saltos REALES sin colapso:")
    for r in res[:10]:
        print(f"  hops_reales={r['real_hops']:2} clear={r['max_clear_mm']:4.0f}mm body_z={r['body_z_ss']:.2f} th2={r['th2_ss']:.2f} colapso={str(r['collapse'])[:1]} :: {json.dumps(r['p'])}")
