"""Carga el URDF HOPPY-E0-final (exportado de SolidWorks) en MuJoCo.

El URDF crudo NO carga directo en MuJoCo por dos razones:
  1. usa rutas 'package://...' que MuJoCo no entiende,
  2. Link2/Link3 tienen >200k caras (limite de MuJoCo).
Este loader lo resuelve: decima las mallas a meshes_mj/ (si faltan) e inyecta un
bloque <mujoco><compiler meshdir=... strippath=true balanceinertia=true/> para que
MuJoCo resuelva las mallas y la inercia.

Uso:
  python3 load_hoppy_urdf.py              # compila e imprime info del modelo
  python3 load_hoppy_urdf.py --render     # guarda figuras/urdf_import.png
  python3 load_hoppy_urdf.py --save-mjcf  # exporta urdf/HOPPY-E0-final.mjcf.xml
  python3 load_hoppy_urdf.py --viewer     # viewer interactivo (necesita display)

NOTA: este modelo es solo cinematica + inercia + mallas (nu=0, sin piso ni contacto
ni actuadores). Para hacerlo SALTAR hay que anadir piso, contacto de pie y actuadores
+ control (siguiente etapa).
"""
import os
import sys
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "HOPPY-E0-final")
URDF = os.path.join(PKG, "urdf", "HOPPY-E0-final.urdf")
SRCMESH = os.path.join(PKG, "meshes")
MESHDIR = os.path.join(PKG, "meshes_mj")
MAXFACES = 200000           # limite duro de MuJoCo
TARGET = 60000              # caras objetivo al decimar


def ensure_meshes():
    """Crea meshes_mj/ con cada STL bajo el limite de caras (decima las grandes)."""
    import trimesh
    import shutil
    os.makedirs(MESHDIR, exist_ok=True)
    for f in sorted(os.listdir(SRCMESH)):
        if not f.lower().endswith(".stl"):
            continue
        out = os.path.join(MESHDIR, f)
        if os.path.exists(out):
            continue
        m = trimesh.load(os.path.join(SRCMESH, f))
        if len(m.faces) >= MAXFACES:
            m = m.simplify_quadric_decimation(face_count=TARGET)
            m.export(out)
            print("decimado %-12s -> %d caras" % (f, len(m.faces)))
        else:
            shutil.copy(os.path.join(SRCMESH, f), out)
            print("copiado  %-12s" % f)


def build():
    """Devuelve el MjModel del URDF listo para MuJoCo."""
    if not os.path.isdir(MESHDIR):
        ensure_meshes()
    xml = open(URDF).read()
    inj = ('<mujoco><compiler meshdir="%s" strippath="true" balanceinertia="true" '
           'discardvisual="false"/></mujoco>' % MESHDIR)
    xml = xml.replace('name="HOPPY-E0-final">', 'name="HOPPY-E0-final">' + inj, 1)
    return mujoco.MjModel.from_xml_string(xml)


def main():
    m = build()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print("compilado OK | nbody=%d njnt=%d ngeom=%d nmesh=%d nq=%d nu=%d | masa movil=%.3f kg"
          % (m.nbody, m.njnt, m.ngeom, m.nmesh, m.nq, m.nu, sum(m.body_mass[1:])))
    for i in range(m.njnt):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        print("  %-8s range=%s" % (n, m.jnt_range[i].round(2)))

    if "--save-mjcf" in sys.argv:
        out = os.path.join(PKG, "urdf", "HOPPY-E0-final.mjcf.xml")
        try:
            mujoco.mj_saveLastXML(out, m)
            print("MJCF exportado ->", out)
        except Exception as e:
            print("no se pudo exportar MJCF:", repr(e)[:120])

    if "--render" in sys.argv:
        import imageio
        ren = mujoco.Renderer(m, 480, 640)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = m.stat.center
        cam.distance = 1.5 * float(m.stat.extent)
        cam.azimuth, cam.elevation = 50, -18
        ren.update_scene(d, cam)
        os.makedirs(os.path.join(HERE, "figuras"), exist_ok=True)
        imageio.imwrite(os.path.join(HERE, "figuras", "urdf_import.png"), ren.render())
        print("render -> figuras/urdf_import.png")

    if "--viewer" in sys.argv:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(m, d) as v:
            v.cam.lookat[:] = m.stat.center
            v.cam.distance = 1.5 * float(m.stat.extent)
            v.opt.label = mujoco.mjtLabel.mjLABEL_JOINT
            while v.is_running():
                mujoco.mj_forward(m, d)
                v.sync()


if __name__ == "__main__":
    main()
