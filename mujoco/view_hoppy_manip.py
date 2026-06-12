"""Visor MANIPULABLE del HOPPY (URDF real de SolidWorks) - mueve cada junta con sliders.

Toma el URDF importado (load_hoppy_urdf), lo exporta a MJCF y le agrega:
  - 4 actuadores de POSICION (joint1..4) -> aparecen como sliders en el panel Control
  - gravedad OFF + contactos OFF  -> posar libremente (puro cinematico)
  - un piso de referencia
Sirve para validar visualmente que las conexiones/cinematica del URDF son correctas
ANTES de pasar al salto (que requiere piso real + contacto de pie + control).

Uso:
  python3 view_hoppy_manip.py            # headless: compila + render de una pose de prueba
  python3 view_hoppy_manip.py --viewer   # GUI interactiva con sliders de Control [necesita display]
"""
import os
import sys
import mujoco
import load_hoppy_urdf as L

OPT = ('<option gravity="0 0 0"><flag contact="disable"/></option>'
       '<visual><global offwidth="1280" offheight="960"/></visual>'
       '<default><joint damping="1"/></default>')   # damping de junta: disipa y mata la vibracion del knee
FLOOR = '<geom name="floor" type="plane" size="3 3 0.1" pos="0 0 0" rgba="0.5 0.5 0.55 1"/>'
POS_ACT = ('<actuator>'
           '<position name="yaw"   joint="joint1" kp="120" dampratio="1" ctrlrange="-6.3 6.3"/>'
           '<position name="pitch" joint="joint2" kp="120" dampratio="1" ctrlrange="-0.5 0.5"/>'
           '<position name="hip"   joint="joint3" kp="120" dampratio="1" ctrlrange="-0.5 0.9"/>'
           '<position name="knee"  joint="joint4" kp="120" dampratio="1" ctrlrange="-1.3 0.4"/>'
           '</actuator>')


def build_manip():
    """Modelo del URDF + actuadores de posicion + gravedad/contacto off + piso."""
    m0 = L.build()                                    # asegura meshes_mj y compila el URDF
    mjcf = os.path.join(L.PKG, "urdf", "HOPPY-E0-final.mjcf.xml")
    mujoco.mj_saveLastXML(mjcf, m0)                   # MJCF base (cinematica + inercia + mallas)
    xml = open(mjcf).read()
    xml = xml.replace('<mujoco model="HOPPY-E0-final">',
                      '<mujoco model="HOPPY-E0-final">\n  ' + OPT, 1)
    xml = xml.replace('<worldbody>', '<worldbody>\n    ' + FLOOR, 1)
    xml = xml.replace('</mujoco>', '  ' + POS_ACT + '\n</mujoco>')
    return mujoco.MjModel.from_xml_string(xml)


def main():
    m = build_manip()
    d = mujoco.MjData(m)
    print("manip OK | nu=%d (sliders Control) | gravity=%s contact_off=%s"
          % (m.nu, list(m.opt.gravity), bool(m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_CONTACT)))

    if "--viewer" in sys.argv:
        from mujoco import viewer as mjv
        mjv.launch(m, d)                              # GUI completa: panel Control con sliders
        return

    # headless: lleva el robot a una pose de prueba (crouch) con los servos y renderiza
    import imageio
    d.ctrl[:] = [0.0, 0.0, 0.5, -1.0]                 # yaw, pitch, hip, knee
    for _ in range(1500):
        mujoco.mj_step(m, d)
    print("pose de prueba (ctrl=hip 0.5, knee -1.0) -> qpos =", d.qpos.round(3))
    ren = mujoco.Renderer(m, 700, 900)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = m.stat.center
    cam.distance = 1.5 * float(m.stat.extent)
    cam.azimuth, cam.elevation = 50, -18
    ren.update_scene(d, cam)
    os.makedirs(os.path.join(L.HERE, "figuras"), exist_ok=True)
    imageio.imwrite(os.path.join(L.HERE, "figuras", "urdf_manip_pose.png"), ren.render())
    print("render -> figuras/urdf_manip_pose.png")


if __name__ == "__main__":
    main()
