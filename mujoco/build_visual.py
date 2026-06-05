"""Genera mallas visuales (OBJ) del CAD real alineadas a los cuerpos del modelo.

Estructura del ensamble (confirmada): el SALTARIN (housing de motores + pierna de
4 barras plegada hacia arriba) esta en low-X; en high-X esta el GANTRY (balero del
pivote) con el TUBO DEL BOOM atravesandolo. Aqui solo se extraen las piezas MOVILES
del saltarin -> link2 = caja de motores; link3 = muslo; link4 = pantorrilla+pie. El
gantry (fijo) sale de build_gantry.py y el boom se dibuja como tubo PVC en make_xml.
La pierna se re-posa (de plegada-arriba a extendida-abajo) a la pose nominal del
modelo. Salida: meshes/vis_link{2,3,4}.obj
"""
import numpy as np, trimesh, mujoco, os
from tune_eval import make_xml, DEFAULTS, HB

def rot_a_to_b(a, b):
    a=a/np.linalg.norm(a); b=b/np.linalg.norm(b); v=np.cross(a,b); c=float(np.dot(a,b))
    if c < -0.9999:
        ax=np.cross(a,[1,0,0]);
        if np.linalg.norm(ax)<1e-6: ax=np.cross(a,[0,1,0])
        ax/=np.linalg.norm(ax); return 2*np.outer(ax,ax)-np.eye(3)
    K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]); return np.eye(3)+K+K@K/(1+c)

def frame_align(sd, su, td, tu):
    def basis(d,u):
        d=d/np.linalg.norm(d); u=u-d*np.dot(u,d); u=u/np.linalg.norm(u); return np.column_stack([d,u,np.cross(d,u)])
    return basis(td,tu)@basis(sd,su).T

# --- 1. pose nominal del modelo (mundo) ---
m=mujoco.MjModel.from_xml_string(make_xml(DEFAULTS)); d=mujoco.MjData(m)
jid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)
qa={n:m.jnt_qposadr[jid(n)] for n in["theta3","theta4"]}
d.qpos[qa["theta3"]],d.qpos[qa["theta4"]]=np.pi/3,-np.pi/2; mujoco.mj_forward(m,d)
bid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)
fs=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,"foot_site")
hipw=d.xpos[bid("link3")].copy(); kneew=d.xpos[bid("link4")].copy(); footw=d.site_xpos[fs].copy()
R3=d.xmat[bid("link3")].reshape(3,3).copy(); R4=d.xmat[bid("link4")].reshape(3,3).copy()
pivot=np.array([0,0,HB]); cw_model=np.array([-0.45,0,HB])

# --- 2. CAD: piezas ---
S=trimesh.load("/tmp/hoppy.glb")
parts=[]
for n in S.graph.nodes_geometry:
    T,gn=S.graph[n]; g=S.geometry[gn].copy().apply_transform(T); parts.append((g, g.centroid.copy()))
C=np.array([p[1] for p in parts])

# saltarin = low-X ; contrapeso = high-X
jumper = C[:,0] < 1.2
cwc = C[~jumper].mean(0)                       # centroide del contrapeso (high-X)
# pierna = mecanismo plegado ARRIBA del housing (low-X, Z alto)
leg = jumper & (C[:,2] > 1.475)
legC = C[leg]
# cadera = base de la pierna (Z minimo del cluster pierna); pie = punta (Z maximo)
hipc  = legC[legC[:,2].argsort()[:5]].mean(0)
footc = legC[legC[:,2].argsort()[-5:]].mean(0)
kneec = 0.5*(hipc+footc)                        # rodilla aprox al medio
print(f"#jumper(low-X)={jumper.sum()} #pierna(arriba)={leg.sum()} #contrapeso(high-X)={(~jumper).sum()}")
print(f"HIP={np.round(hipc,3)} KNEE={np.round(kneec,3)} FOOT={np.round(footc,3)} CW={np.round(cwc,3)}")
print(f"pierna plegada: cadera->pie={np.round(footc-hipc,3)} (apunta +Z = plegada arriba)")

# --- 3. alinear y exportar por grupo ---
def export(idxs, kind):
    if not len(idxs): print(f"  {kind}: 0 piezas!"); return
    full=trimesh.util.concatenate([parts[i][0] for i in idxs]); V=full.vertices.copy()
    if kind=="body":
        R=frame_align(cwc-hipc, [0,0,1.], cw_model-hipw, [0,0,1.])
        Vw=(R@(V-hipc).T).T+hipw; loc=Vw-pivot
    elif kind=="thigh":
        R=rot_a_to_b(kneec-hipc, kneew-hipw); s=np.linalg.norm(kneew-hipw)/np.linalg.norm(kneec-hipc)
        Vw=(R@(s*(V-hipc)).T).T+hipw; loc=(R3.T@(Vw-hipw).T).T
    else:
        R=rot_a_to_b(footc-kneec, footw-kneew); s=np.linalg.norm(footw-kneew)/np.linalg.norm(footc-kneec)
        Vw=(R@(s*(V-kneec)).T).T+kneew; loc=(R4.T@(Vw-kneew).T).T
    full.vertices=loc
    out=f"meshes/vis_{ {'body':'link2','thigh':'link3','shank':'link4'}[kind] }.obj"
    full.export(out); print(f"  {out}: {len(full.vertices)} verts")

idx=np.arange(len(parts))
thigh_i = idx[leg & (C[:,2] <= kneec[2])]      # mitad baja de la pierna = muslo
shank_i = idx[leg & (C[:,2] >  kneec[2])]      # mitad alta (con el pie) = pantorrilla+pie
body_i  = idx[jumper & ~leg]                   # SOLO la caja de motores (low-X); el
                                               # gantry (high-X) y el tubo del boom NO
                                               # van en link2 (movil) -> gantry fijo +
                                               # boom PVC se dibujan aparte en make_xml
os.makedirs("meshes",exist_ok=True)
export(body_i,"body"); export(thigh_i,"thigh"); export(shank_i,"shank")
print("listo: mallas en meshes/vis_link*.obj")
