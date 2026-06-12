# Índice de material para la PRESENTACIÓN

Mapa de evidencia por fase de la rúbrica: qué figura/video usar, dónde está, y la
historia de iteración que se puede contar con cada una. Todas las rutas de figuras
son relativas a `mujoco/figuras/`. El material del robot físico vive fuera del repo
(ver última sección).

## Arco narrativo sugerido

1. El problema: HOPPY (paper UIUC) y nuestro rediseño físico tipo ave.
2. Modelado en MuJoCo: de cero a gemelo digital (iteraciones de la pierna).
3. Física honesta: actuadores reales, contacto duro, validación contra MATLAB.
4. Control híbrido del paper: vuelo + apoyo + FSM, salto forward en sim.
5. Del simulador al robot: firmware, calibración en banco, bugs cazados.
6. El robot salta solo (video) y qué sigue.

## Fase 1 - Modelo mecánico (20 pts)

| Evidencia | Figura/archivo | Historia |
|---|---|---|
| Gantry 4 DoF + contrapeso | `twin_armado_hoppy.png`, `twin_robot_lateral.png`, `gantry_real.png` | El gantry se extrajo del STEP real del CAD |
| Iteraciones del gemelo CAD | `cad_overlay_nominal.png` -> `cad_overlay_v2.png` -> `cad_overlay_check.png`, `montaje.png`, `montaje_cad.png` | Overlay cosmético rechazado -> gemelo estructural con dims/masas/inercias medidas del CAD |
| La saga de la pierna (la mejor historia de iteración) | `leg_MESH_4barras.png` -> `twin_leg_v2.png` -> `leg_proc2_stand.png` / `leg_proc2_crouch.png` | 6 intentos documentados en `mujoco/HANDOFF.md` §2: la malla CAD del 4-barras es IMPOSIBLE de articular con 1 junta (las placas cruzan la rodilla) -> pierna procedural que nunca se separa |
| El 4-barras real | `pivotes_4barras.png`, `pivotes_esquema.png`, `diagrama_esquema.png` | Pivotes medidos del GLB del ensamble |
| Armature y damping | (se cuenta con `ablacion.png`, Fase 2) | N_H=26.9, N_K=28.8; damping = kT²N²/Rw (back-EMF), justificación física |
| Resorte paralelo de rodilla | (idem) | El resorte serie-elástico real (Ks=1.67 kN/m) vs el de junta: trade-off documentado en `mujoco/HANDOFF.md` §3 |

## Fase 2 - Actuadores y restricciones físicas (10 pts)

| Evidencia | Figura/archivo | Historia |
|---|---|---|
| Ablación completa | `ablacion.png` (genera `ablacion.py`) | Sin armature salta 67% más (irreal); sin saturación el actuador pide 13.2 A con un motor de 9.2 A. La saturación es lo que mantiene la sim honesta |
| Modelo de motor | `controller.py` (Ec.18) | No es un clip de torque: voltaje -> back-EMF -> corriente -> torque, con los límites del datasheet goBILDA (12 V / 9.2 A) |

## Fase 3 - Contacto pie-suelo (15 pts)

| Evidencia | Figura/archivo | Historia |
|---|---|---|
| Contacto duro | `foot_revertido.png` | El pie físico es un punto en el regatón real del shank; se probó punta/spike y se descartó por cambiar la geometría real |
| Integrador justificado | `integradores.png` (genera `comparacion_integradores.py`) | implicitfast vs el RK4 recomendado: mismo salto, 15% más rápido, y integra implícito el damping/armature |
| Detección touchdown/liftoff | (señales de Fase 5) | El criterio de la sim (GRF > umbral) es EL MISMO del robot real (SoftPot con umbral): calibrado físico aire=0 / rozando=200 / cargado=2870 / presionado=4095 |

## Fase 4 - Control híbrido (40 pts)

| Evidencia | Figura/archivo | Historia |
|---|---|---|
| Validación contra MATLAB | `comparacion_matlab.png` | Port fiel del simulador del paper: PASS 12/12 en `verify.py` |
| Salto del modelo abstracto | `salto.mp4`, `resultados.png` | Primera versión validada (score 99/100) |
| Salto del gemelo CAD | `twin_salto.mp4`, `twin_hero.png` | 11 saltos sostenidos, motores al límite 12 V / 9.2 A |
| Salto forward del URDF real | `hop_urdf_limpio.mp4`, `render_forward_secuencia.png`, `hop_apex_lateral.png`, `hop_apex_34.png` | El control real del paper en el URDF de SolidWorks: avanza, 66% de vuelo, ciclo límite estable |
| Por qué avanza (discriminador) | `diag_forward_vs_actual.png`, `avance.png` | La métrica vieja era engañosa; el avance lo fija el empuje tangencial Fx del apoyo (hallazgo con el paper) |
| Iteración de controladores | `diagnostico_salto.png` | El primer FSM propio daba 18.8 cm IRREALES (atravesaba el piso); el diagnóstico de 8 señales llevó al salto limpio de 11.1 cm y después al control real del paper |
| Lección de métricas | `honest_eval.py` | Una métrica floja contaba "50 saltos" con la pierna aleteando: por eso verify.py exige GRF real, vuelo y no-aleteo |

## Fase 5 - Sensores y señales (15 pts)

| Evidencia | Figura/archivo | Historia |
|---|---|---|
| Señales completas (gemelo) | `señales_rubrica.png` (genera `plot_signals.py`) | 7 paneles: posiciones y velocidades articulares Y cartesianas (estimadas con la cadena de encoder J·qd_filt), GRF, torques con límites, FSM |
| Señales del URDF forward | `señales_forward.png`, `señales_urdf.png` | Mismo análisis sobre el modelo real |
| Emulación de encoder | `encoder_signals.png` | Velocidad por derivada filtrada (lambda=10), nunca qvel directo |

## Más allá de la rúbrica - EL ROBOT FÍSICO (el cierre de la presentación)

Material fuera del repo (recolectar antes de armar las diapositivas):

- **Videos/fotos de WhatsApp**: el robot en banco, la pose cero, y EL VIDEO DE LOS
  PRIMEROS SALTOS (2026-06-11, ~14 ciclos a 1.3 Hz; analizado cuadro a cuadro).
- **Screenshots de CCS Expressions** (`~/Pictures/Screenshots/`, 2026-06-10 y 11):
  la calibración en vivo, el dry-run de la ley de apoyo calzando 1% con la
  predicción, y la telemetría de los 33 saltos (n_hops, t_air_last).
- Historias con número (bitácoras en `Microcontroller/HANDOFF_FIRMWARE.md`):
  - El mapeo de pwm del ejemplo entregaba el 5% del torque (bug heredado).
  - El abs() entero del driver: zona muerta de 1.2 V y duty en escalones de 10%.
  - El "empuje débil" que era escape geométrico (la pierna se alinea con la
    fuerza y la ley correctamente deja de empujar) y el test isométrico de 3 kg.
  - Morfología tipo ave: la IK espejo y el mapa del 4-barras calibrado en banco.
  - 33 saltos autónomos con sensor de contacto real; los acoples patinando con
    los impactos de aterrizaje (piezas reimpresas en camino).

## Tabla resumen de los 3 modelos (buena diapositiva)

| Modelo | Salta | Geometría | Rol en la historia |
|---|---|---|---|
| Abstracto (`tune_eval`) | si, 7.2 cm | simplificada | validado vs MATLAB 12/12 |
| Gemelo CAD (`twin`) | si, 8.8 cm | CAD procedural | dims/masas/inercias reales |
| URDF real (`hoppy_urdf`) | si, ~7.6 cm de vuelo, avanza | SolidWorks exacto | el que corre el control del robot |
