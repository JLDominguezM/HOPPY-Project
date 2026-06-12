# HANDOFF - Simulación MuJoCo de HOPPY (estado al 2026-06-05)

Documento para retomar el trabajo en otra sesión. Estado **actual** exacto, qué se
logró, qué falta, cómo se hizo y dónde está cada cosa. Lee la §0 primero.

> **ACTUALIZACIÓN 2026-06-09:** la sim forward quedó lista (`mujoco/CONTROL_FORWARD.md`,
> `view_hop_urdf.py --viewer`) y ya se pasó al **robot físico**: la puesta a punto del firmware
> está COMPLETA y se escribió el controlador aéreo. Ver **`../Microcontroller/HANDOFF_FIRMWARE.md`**.b

---

## 0. ESTADO ACTUAL EN UNA PANTALLA

Hay **DOS modelos** en `mujoco/`:

1. **Abstracto + overlay** (`tune_eval.py` + `controller.py`): afinado a los parámetros
   del MATLAB original (`Simulator_MATLAB`), con mallas CAD pegadas encima. Cumple la
   rúbrica, validado contra MATLAB (12/12). **Intacto, histórico** (ver §7).
2. **GEMELO DIGITAL ESTRUCTURAL** (`twin.py`) ← **donde está todo el trabajo reciente.**
   Los eslabones SON la geometría real del rediseño (dims/masas/inercias/actuador medidos
   del CAD/STEP + datasheets + `List of nominal parameters.pdf`).

**El gemelo HOY (config en `twin.DEFAULTS`, `view_twin.py` con pierna procedural):**
- **SALTA de verdad:** 11 saltos sostenidos, cuerpo **+8.8 cm**, **pie despega 6.3 cm
  (fase de vuelo real)**, **avanza 0.59 rad/s** (≈ una vuelta al poste cada ~11 s),
  ciclo estable, motores al límite **12 V / 9.2 A**.
- **Pierna PROCEDURAL gris detallada** (placas IMP-8/9 + tubo TUB-1 + regatón negro,
  color housing) que **NUNCA se separa** al articular + 2 resortes-tendón dorados (visuales).
- Gantry fijo + boom PVC + housing con masas/inercias/actuadores/sensores reales.
- OJO: **El resorte 100% real (Ks=1.67 kN/m) NO permite saltar** (deja la rodilla casi
  rígida). El rebote del salto lo da un resorte de junta más suave (config seed23). Ver §3.
- OJO: La **malla CAD 4-barras exacta NO se puede articular** sin partirse (las placas
  cruzan la rodilla); queda solo para `view_twin_static` (reposo). Ver §3.

**Correr el gemelo:**
```bash
cd ~/ImplementacionRobotica/HOPPY-Project/mujoco
python3 view_twin.py          # SALTO en vivo (pierna procedural, cámara sigue al hoppy)
python3 view_twin_static.py   # ensamble CONGELADO con la malla CAD detallada (inspeccionar)
python3 twin_check.py         # verificación componente-por-componente (19/19 histórico)
python3 -c "import twin; from verify import verify; verify(dict(twin.DEFAULTS), mdl=twin)"
MUJOCO_GL=egl python3 render_twin.py   # video figuras/twin_salto.mp4
python3 build_twin_meshes.py  # regenera meshes/twin_*.obj del GLB
python3 twin_tune.py 350 1    # re-afina la marcha (búsqueda aleatoria honesta)
```
`controller.py`, `verify.py`, `simulate()` están PARAMETRIZADOS: default = modelo
abstracto; pasar `mdl=twin` usa el gemelo. `Hoppy(params, mdl=twin)`.

---

## 1. CONFIG ACTUAL DEL GEMELO (valores exactos en `twin.py`)

**Geometría (m), medida del CAD (eje vertical = Y_cad, ver §4):**
```
HB=0.250   pivote yaw+pitch sobre el piso
LB=0.687   boom: pivote -> cadera (X)
DB=0.187   offset lateral de la cadera (cara FRONTAL del housing, entre los 2 motores)
HIP_DZ=-0.036   cadera 3.6 cm bajo el eje del boom
RBOOM=hypot(LB,DB)=0.712   radio cadera->eje yaw (avance = dtheta1*RBOOM)
LH=0.096  DK=0.052  LK=0.1545  L=hypot(LK,DK)=0.163   (HOPPY ref, confirmado CAD)
KNEE_OFF=(-0.0018, 0.0586, -0.0583)   rodilla en link3-local (union tubo-placas)
FOOT=(0.0048, -0.1126, -0.1717)       pie en link4-local (pantorrilla con offset DK)
BOOM_X0,BOOM_X1=-0.39,0.81   extremos del tubo PVC   BOOM_OD=0.0334 (1")
```

**Masas (kg):** M_HOUSING=0.791, M_BOOM=0.481, M_MOTORS=0.870 (2× goBILDA 435 g),
M_ELEC=0.150, M_THIGH=0.130, M_SHANK=0.100, M_YAW=0.060. link2 compuesto ≈ 2.29 kg.
(El PDF da masas "oficiales" link3=0.656, link4=0.149 - twin usa estimaciones por STL.)

**Actuador goBILDA 5202-2402-0027 (datasheet):** N=26.9 (cadera y rodilla; **OJO:** el PDF
dice N_K=28.8 para la rodilla, twin usa 26.9 en ambas - discrepancia menor pendiente),
Rw=1.3, kT=0.0135, kv=0.0186, **VMAX=12, IMAX=9.2 A** (no 30). I_ROTOR=7e-6, armadura N²·Ir.

**Rangos de junta:** theta3 (cadera) `range="-0.5 0.9"`, theta4 (rodilla) `range="-1.3 0.4"`.

**`twin.DEFAULTS` (la marcha que SALTA - seed23):**
```
solref0=0.0191, j_damp=0.1494,
knee_stiff=0.0948, knee_ref=0.0088, knee_damp=0.0, spring_scale=0.0,
kp_sw=249.60, kd_sw=5.0839, krh=0.1187, p_toe_z=-0.1828,
Tst=0.285, kp_st=0.03, kd_st=0.08, q3_ref=0.1572, q4_ref=-0.4607,
fz_scale=3.1838, fx_scale=3.3653, blend=0.0136, grf_liftoff=3.8109, vx_d=-0.3385,
```
- `knee_stiff=0.0948` = resorte de JUNTA (da el rebote del salto).
- `spring_scale=0.0` = los 2 tendones-resorte son **VISUALES** (no aplican fuerza).
  Para fuerza real del resorte serie-elástico, ver §3.
- `vx_d` = velocidad de avance deseada (Raibert), el knob que destraba el avance (§5).

**Pierna procedural (la del salto):** `make_xml` con `vis=True` + `leg_proc=True`. Geoms
grises (placas IMP-8/9 siguiendo KNEE_OFF, tubo TUB-1 siguiendo FOOT, regatón negro) +
eje de cadera/rodilla oscuros. El resorte NO se dibuja aquí (lo dan los tendones dorados).

---

## 2. EL VIAJE DE LA PIERNA (qué se intentó y por qué, en orden)

El usuario fue iterando sobre la pierna; cada versión tuvo un detalle que él cazó. Resumen
para no repetir caminos:

1. **Overlay cosmético** → rechazado ("solo pusiste los objetos encima"). Se pasó a gemelo
   estructural (los eslabones SON la geometría).
2. **Cadera a ojo** (centro-abajo del housing) → el usuario: "está mal dónde está la pierna".
   Se midió del CAD: cadera en la **cara FRONTAL** (LB=0.687, DB=0.187, HIP_DZ=-0.036).
3. **Pierna recta** (DK embebido en L, como el MATLAB) → la malla CAD **se desfasa al doblar**
   (gira sobre un pivote falso). Se pasó a **geometría planar real** (vectores KNEE_OFF/FOOT).
4. **Pivote de rodilla mal** (en la base del muslo, 5 cm bajo el rodamiento) → se subió al
   rodamiento (union tubo-placas). Pero seguía desfasándose un poco.
5. **Pivote en P4** (rodamiento real del STEP, model 0.688,0.171,0.123) + **resorte
   serie-elástico real** como tendones → el resorte se estira/conecta bien PERO **no salta**
   (rodilla casi rígida) y **se sigue separando** la malla en la rodilla.
6. **Causa raíz de la separación (la clave):** las placas **IMP-8/9 miden 150 mm** y
   **CRUZAN la rodilla cinemática** (que está a LH=96 mm de la cadera). Cualquier modelo
   "muslo rígido + pantorrilla rígida + 1 junta" **parte esas placas**: la mitad de abajo
   rota con la pantorrilla y se separa de la de arriba. **Es físicamente inevitable con la
   malla CAD** - el knee real es un 4-barras (las barras cruzan la junta).
7. **Decisión final del usuario: pierna PROCEDURAL bien hecha + que salte de verdad.**
   - `view_twin` usa la **procedural gris** (generada para calzar las juntas → nunca se
     separa). La malla CAD detallada queda en `view_twin_static` (reposo, donde sí conecta).
   - Se revirtió la rodilla a la **union tubo-placas** (KNEE_OFF=(-0.0018,0.0586,-0.0583))
     + resorte de junta suave + gait seed23 → **salta 8.8 cm con vuelo y avanza 0.59 rad/s**.

**Lección:** para una malla CAD detallada que articule perfecto se necesita modelar el
**4-barras con lazo cerrado** (cada barra IMP-8/9/12/13 como cuerpo móvil + equality
`connect`). No se construyó porque los pivotes exactos NO se extraen confiable de los
archivos disponibles (ver §6). La procedural lo evita (se genera a la medida de las juntas).

---

## 3. EL TRADE-OFF FIDELIDAD ↔ SALTO (aprendizaje honesto clave)

El knee del HOPPY es un **4-barras SERIE-ELÁSTICO** (no 2 juntas rígidas). Hay tensión real
entre modelarlo fiel y que salte. El usuario eligió **salto** (tras presentarle el trade-off).

**El resorte real (del PDF, pág. 4):** 2× RESORTE DE TENSIÓN, **Ks=1.67 kN/m, L0=80 mm**,
`Ls(θ4)=a2·θ4²+a1·θ4+a0` (a2=-12.09, a1=10.75, a0=112.4 mm), `τ_s=(2a2θ4+a1)·Ks·(L0-Ls)`.
Se modeló como **2 tendones `<spatial>`** muslo(link3)↔palanca(link4), `stiffness=Ks·spring_scale`,
`springlength="0 L0"` (resorte de tensión: solo jala). Verificado: se estiran 87→114 mm con
θ4 (a -0.6 rad) y quedan SIEMPRE conectados. Constantes en twin.py: `KS_SPRING=1670`,
`L0_SPRING=0.080`, sitios `SPR_A_{R,L}` (link3) y `SPR_B_{R,L}` (link4).

**Hallazgo:** con `spring_scale=1.0` (Ks real) la rodilla queda **casi rígida** → el cuerpo
bobea ~16 mm **SIN vuelo** (el pie no despega), casi no avanza. El HOPPY real salta con este
resorte usando el controlador sofisticado del paper; el port MATLAB + búsqueda aleatoria **NO
lo logra**. El tuner siempre pide bajar el resorte (`spring_scale`→0.16-0.42) y aun así no
hay vuelo limpio.

**Por eso el estado final usa `spring_scale=0.0`** (tendones VISUALES) + el resorte de JUNTA
`knee_stiff=0.0948` para el rebote → salta. Tres caminos a futuro si se quiere fidelidad:
- **(a)** Resorte serie-elástico fiel `spring_scale=1.0` → bobea sin vuelo (máx fidelidad).
- **(b)** Mejor controlador (no búsqueda aleatoria): replicar el control híbrido del paper
  para saltar CON el resorte real. Es la opción "correcta" pero es trabajo (no elegida).
- **(c)** Lo actual: resorte de junta suave (salta) + tendones visuales. Elegido por el usuario.

`view_twin.py` ya documenta cómo ver el resorte estirándose (`spring_scale=0.3`), aunque con
eso baja el salto.

---

## 4. DATOS REALES EXTRAÍDOS DEL CAD (para no re-medir)

**Archivos CAD:** `~/ImplementacionRobotica/HOPPY-Project/CAD/Final_Assembly/`
- `List of nominal parameters.pdf` ← **oro**: dims (HB=196.5, LB=556, DB=48, LH=96, LK=154.5,
  DK=52 mm del HOPPY ORIGINAL), masas/inercias link1-4, actuador (N_H=26.9, N_K=28.8, Rw=1.3,
  kT=0.0135, kv=0.0186, I_max=30 A del original), y el modelo del resorte (§3).
- `Link3_Assembly.STEP`, `Link4_Assembly.STEP` ← la pierna posicionada y nombrada (por número
  de catálogo goBILDA, NO IMP). **Cargan con `import cascadio` + `trimesh.load(...)`** (~1-2 min).
- STLs individuales nombrados en `mujoco/meshes/IMP-*.STL` (en su frame local, sin posicionar).
- `/tmp/hoppy.glb` = ensamble completo (328 piezas, nombres genéricos NAUO, **en metros**).
  Regenerar: `cascadio.step_to_glb('/tmp/HOPPY-E0.STEP', '/tmp/hoppy.glb', tol_linear=2.0)`.

**OJO: EJE VERTICAL = Y_cad (no Z).** El GLB de glTF es **Y-UP**. `build_twin_meshes.py` y
`twin.py` usan `model_z = (y_cad - py) + HB`, `model_x = -(x_cad - px)`, `model_y = z_cad - pz`,
con `PIVOT_CAD=[1.559, 1.312, 1.349]`. Con Z-up todo salía **rotado 90°** (bug que el usuario
cazó). Con Y-up la base del gantry cae al piso y el pivote queda a 0.250 m (coincide con la foto).

**Mecanismo del knee (del STEP Link3/Link4):**
- Motor de rodilla `5202-0002-0019` (goBILDA 5202) en link3.
- Placas **IMP-8/9** (long ~150 mm, las que cruzan la rodilla), ejes/rodamientos
  `2800-0004-0014` ×6, `1108-0001-0001` ×4 (placas planas 56 mm).
- Pantorrilla = **tubo `4100-1214-0200` (TUB-1)**, 200 mm, en link4, **offset DK=52 mm** del
  eje de la rodilla (el tubo está a y=-52 mm del rodamiento `3411-0014-0024` en frame Link4).
- Sensor lineal `Linear_Disp_Sensor_404R1KL` + palanca, 2 resortes de tensión.

**Pivotes del 4-barras (model frame, del GLB):** P1 (0.186,0.204), P2 (0.155,0.164),
P3 (0.198,0.153), P4 (0.171,0.123); anclas del resorte A=(0.165,0.179), B=(0.247,0.152).
Diagrama: `figuras/pivotes_esquema.png`, `figuras/diagrama_esquema.png`. Útiles si algún día
se modela el lazo cerrado exacto.

---

## 5. AVANCE ALREDEDOR DEL POSTE (resuelto - el knob `vx_d`)

El gemelo saltaba **en sitio** (no avanzaba). Investigación 3-agentes (2026-06-04) + paper
(arXiv 2010.14580): HOPPY **avanza por la componente HORIZONTAL de la fuerza de apoyo** (Fx
tangencial), no por foot-placement. El bug: el control de vuelo usaba `p_d_x = krh·vx` SIN
velocidad deseada → amortiguaba el yaw a 0 (en sitio por diseño). **Fix:** se agregó `vx_d`
(Raibert) en `controller.py`: `p_d = [krh·(vx − vx_d), p_toe_z]` (default 0 = comportamiento
original intacto). Con `vx_d≠0` regula vx hacia vx_d y **avanza**. Está en `twin.DEFAULTS`
(`vx_d=-0.3385`) y en las RANGES de `twin_tune`. **El salto en sitio era un bug de control,
NO falta de contrapeso.**

**Contrapeso (opcional, fiel al original pero NO al robot físico actual):** `twin.make_xml`
acepta `cw_mass`/`cw_x`. El rediseño está MUY desbalanceado (CoM de link2 a ~0.60 m hacia el
hoppy → ~15 N·m de gravedad). Para anular el cabeceo: contrapeso×brazo = **1.54 kg·m**
(≈2 kg @0.65 m). Con contrapeso el salto mejora (12 cm, menos saturación) pero el paper dice
que es muleta para motores débiles. `twin.DEFAULTS` queda **SIN contrapeso** (fiel al físico).

---

## 6. POR QUÉ NO SE MODELÓ EL LAZO CERRADO EXACTO

El usuario pidió "lazo cerrado exacto". No se construyó porque **los pivotes exactos de cada
barra no se extraen confiable**:
- El GLB tiene nombres genéricos (NAUO###), no IMP/catálogo.
- El STEP nombra por catálogo goBILDA, no por IMP-8/9/12/13.
- El emparejamiento automático STL↔GLB por volumen/dimensiones **falla** (teselan distinto).
- Los STL individuales (IMP-*.STL) están en frame local, sin posicionar.
Para hacerlo bien se necesitaría: abrir el CAD y leer las coordenadas de los mates/pivotes
(el usuario **no tiene SolidWorks**), o un emparejamiento geométrico mucho más robusto.
Mientras tanto, la **pierna procedural** evita el problema (se genera a la medida de las juntas).

---

## 7. MODELO ABSTRACTO (histórico, intacto) - `tune_eval.py`

La **simulación física original** está LISTA y verificada: cumple las 5 fases de la rúbrica y
replica el MATLAB. Modelo anclado a `Simulator_MATLAB/fcns/get_params.m` (HB=0.1965, LB=0.556,
LH=0.096, LK=0.163). Controlador `controller.py` (FSM 1 kHz; aéreo PD cartesiano del pie;
apoyo `u=-J^T[Fx;Fz]` Bézier + PD; voltaje+back-EMF+saturación; blending 10 ms). Config en
`tune_eval.DEFAULTS` (score 99/100). `verify.py` PASS 12/12 vs `ref_matlab.csv`. Correr:
`python3 control.py`, `python3 verify.py`, `python3 view.py`, `MUJOCO_GL=egl python3 render.py`.

**Lección histórica:** una métrica floja que "contaba saltos" por despegue del pie daba "50
saltos estables" con la pierna ALETEANDO sobre un boom flotante. Por eso `verify.py` exige
empuje real (GRF carga), vuelo, no-aleteo (clearance ≤ 2× subida) y ciclo estable. **No
confiar en métricas que cuentan despegue del pie.**

---

## 8. GUÍA DE ARCHIVOS (`mujoco/`)

| Archivo | Qué es |
|---|---|
| **`twin.py`** | **Gemelo: modelo MJCF (`make_xml`), constantes reales, inercias compuestas, `DEFAULTS`, tendones-resorte, pierna procedural/malla.** |
| `controller.py` | Clase `Hoppy` (controlador compartido) + `simulate()`. Parametrizado: `mdl=twin`. Jacobiano de MuJoCo en vivo (agnóstico a longitudes). |
| `build_twin_meshes.py` | Extrae `meshes/twin_{gantry,housing,thigh,shank}.obj` del GLB (Y-up, frames de junta, excluye bobinas SPRING_IDX). |
| `twin_tune.py` | Re-afina la marcha del gemelo (búsqueda aleatoria, score honesto en `verify.analyze`). RANGES incluye spring_scale, knee_damp, vx_d, q3/q4_ref... |
| `twin_check.py` | Verificación componente-por-componente (actuadores/joints/resorte/contacto/sensores/masas). |
| `verify.py` | Suite de chequeos + `analyze()`. Parametrizado (`mdl=`). |
| `view_twin.py` | Visor interactivo del SALTO (pierna procedural, cámara sigue al hoppy). |
| `view_twin_static.py` | Ensamble CONGELADO con la malla CAD detallada (inspeccionar). |
| `render_twin.py` | Video `figuras/twin_salto.mp4`. |
| `tune_eval.py`, `control.py`, `view.py`, `render.py`, `ref_matlab.csv` | Modelo abstracto histórico (§7). |
| `ablacion.py` | Rúbrica Fase 2.2: influencia de armature/damping/resorte/saturación (figuras/ablacion.png). |
| `comparacion_integradores.py` | Justificación de implicitfast vs el RK4 recomendado (figuras/integradores.png). |
| `plot_signals.py` | Señales de la rúbrica Fase 5, 7 paneles (figuras/señales_rubrica.png). |
| `meshes/twin_*.obj`, `meshes/IMP-*.STL` | Mallas (gitignored las twin_*, regenerables). |
| `figuras/` | Renders, diagramas de pivotes, comparaciones de pierna. |

(2026-06-11: se borraron `control_legacy.py`, `view_debug.py`, `build_model.py`,
`sweep.py` y `tune_metric.py` - código muerto superado; recuperable del historial
de git. El índice de material para la presentación vive en `../PRESENTACION.md`.)

---

## 9. TODO / PRÓXIMOS PASOS

1. **(Pulido)** Ajustar look de la pierna procedural si el usuario quiere (colores, grosor
   tubo/placas, tamaño de los resortes-tendón). Editar el branch `elif vis:` en `twin.make_xml`.
2. **(Fidelidad del resorte)** Si se quiere el serie-elástico real saltando: implementar el
   control híbrido del paper (no búsqueda aleatoria) - opción (b) de §3.
3. **(Malla CAD que articule)** Modelar el 4-barras con lazo cerrado (IMP-8/9/12/13 como
   cuerpos + equality `connect`). Requiere los pivotes exactos (§6) - pedir al usuario las
   coordenadas del CAD o un emparejamiento robusto.
4. **(Menor)** Reconciliar N_K: el PDF dice 28.8 (rodilla), twin usa 26.9. Pesar componentes
   reales para las masas (twin estima por STL; el PDF da link3=0.656, link4=0.149).
5. **(Contrapeso)** Si se decide ponerlo físicamente: ~2 kg @0.65 m en el balance del boom.

---

## 10. GIT Y REPORTE (reglas del usuario)

- Repo: `~/ImplementacionRobotica/HOPPY-Project` → `origin = github.com/JLDominguezM/HOPPY-Project`.
- Rama **`mujoco-simulacion`** (commit `14be646`) = la simulación abstracta estable. **NO
  pusheado, NO mergeado a main** (queda local a petición del usuario).
- El trabajo del **gemelo** (`twin*.py`, `build_twin_meshes.py`, `meshes/twin_*`, figuras)
  **NO está commiteado** aún.
- **Autor siempre JLDominguezM, SIN atribución de IA** (nada de "Co-Authored-By: Claude" ni
  firmas del modelo en commits/tags/PR). Mensajes en imperativo, como escritos por persona.
- **El reporte LaTeX** (`~/ImplementacionRobotica/reporte/*.tex`) va aparte y **NO se commitea**.
- El correo del sistema `daniel-hinojosa09@outlook.com` es un ALIAS engañoso - el usuario es
  **José Luis Domínguez Morales** (A01285873@tec.mx). Nunca usar el alias para autoría.

---

## Sesión Jun 5-6 2026 - URDF Real + Salto Limpio

### Lo que se hizo (en orden cronológico)

1. **Auditoría contra rúbrica final** (Jun 5)
   - NK corregido a 28.8 (rodilla), damping kT²N²/Rw añadido
   - verify.py calibrado por modelo (IMAX dinámico, q4 no-crítico)
   - Gráficas Fase 5 completadas (6 subplots: pos/vel/GRF/torques/FSM)

2. **Mejora visual de la pierna procedural** (Jun 5)
   - twin.py: 2 cápsulas negras paralelas (IMP-8/9), motores oscuros,
     tubo gris TUB-1 r=0.014, regatón esfera negra, housing con malla CAD
   - view_twin_static.py / view_twin.py: encuadre del robot completo

3. **Importación del URDF real desde SolidWorks** (Jun 6)
   - Plugin sw2urdf → HOPPY-E0-final.urdf exportado
   - load_hoppy_urdf.py: decima mallas a meshes_mj/ (<60k caras)
   - Joints renombrados theta1-4 para compatibilidad con controller.py
   - Cinemática verificada con view_hoppy_manip.py (sliders por joint)
   - Fix: knee vibraba → kp aumentado; yaw sin límites (es balero)

4. **Primer intento: controller.py con el URDF** (Jun 6 madrugada)
   - hoppy_urdf.py creado (análogo a twin.py para el URDF)
   - view_hoppy_jump.py: viewer del URDF con controlador híbrido
   - RESULTADO: 0.0 cm - el gait del twin no transfiere al URDF
     (masas distintas: twin=2.58 kg vs URDF=3.43 kg, geometría real)
   - Test mecánico: torque constante hip=−5/knee=+5 → +23.6 cm
     → confirmó que el mecanismo SÍ puede saltar

5. **Controlador propio del URDF: hop_controller.py** (Jun 6)
   - FSM 3 estados: CARGA → EMPUJE → VUELO
   - EMPUJE: torque directo hip=−TAU, knee=+TAU (signos del test mec.)
   - hop_tune.py: random search 5 params, 300 iters
   - RESULTADO v1: 18.8 cm - pero irreal (atravesaba piso 4 cm,
     rodilla golpeaba límite −1.3, control bang-bang)

6. **Diagnóstico y corrección del salto** (Jun 6)
   - diagnostico_salto.png: 8 subplots (joints, pie, altura, GRF,
     torques, FSM, velocidades, retrato de fase hip-knee)
   - Fix: q4_crouch −1.156 → −0.85 (máximo limpio sin penetración)
   - Fix: TAU 5.0 → 3.5 Nm, T_vuelo_min=0.08 s (anti bang-bang)
   - Penalización en hop_tune: fitness -= 50*|Link4_z| si Link4 < 0
   - Barrido q4_crouch: cliff en −0.90 (penetra), óptimo en −0.85
   - RESULTADO FINAL: 11.1 cm limpio (excursión del cuerpo; pie despega
     ~3.8 cm), Link4_min=+0.115 m, 0% penetración, rodilla sin golpear límite

### Estado actual de los 3 modelos

| Modelo | Salta | Altura | Controlador | Geometría |
|---|---|---|---|---|
| Abstracto (tune_eval) | si | 7.2 cm | Híbrido rúbrica | Simplificada |
| Gemelo CAD (twin.py) | si | 8.8 cm | Híbrido rúbrica | CAD procedural |
| URDF real (hoppy_urdf) | si | 11.1 cm | FSM propio (hop_controller) | CAD SolidWorks real |

> Las alturas son **excursión vertical del cuerpo** (pico − valle del ciclo). El despegue
> real del pie sobre el piso es menor (URDF ≈ 3.8 cm, gemelo ≈ 6.3 cm) - ver §0.

### Archivos clave añadidos esta sesión

```
mujoco/hoppy_urdf.py         - URDF como módulo compatible con controller.py
mujoco/hop_controller.py     - FSM 3 estados para el URDF
mujoco/hop_tune.py           - tuner random search con penalización física
mujoco/view_hop_urdf.py      - viewer del URDF saltando
mujoco/view_hoppy_manip.py   - viewer con sliders por joint
mujoco/load_hoppy_urdf.py    - loader URDF→MuJoCo con decimación de mallas
mujoco/HOPPY-E0-final/       - paquete URDF + meshes del SolidWorks
figuras/hop_urdf_limpio.mp4  - video 10 s salto limpio
figuras/hop_apex_lateral.png - render apex vista lateral
figuras/hop_apex_34.png      - render apex vista 3/4
figuras/diagnostico_salto.png - diagnóstico completo del ciclo
figuras/señales_rubrica.png  - 6 subplots rúbrica Fase 5
```

### Discrepancias conocidas (pendientes o aceptadas)

- spring_scale=0.0 en twin.py: resorte real desactivado, rebote por
  knee_stiff (documentado en §3 anterior)
- N_K twin: corregido a 28.8 esta sesión
- Masa link3 twin: 0.130 vs 0.656 kg PDF (motor agrupado en link2)
- URDF: controlador propio (no el híbrido de la rúbrica) - el híbrido
  no transfiere por diferencia de masas/geometría
- q4 gemelo: opera en [-0.17, +0.46] (no singular por geometría 4-barras)
  → chequeo no-crítico en verify.py (documentado)

### Próximos pasos sugeridos

1. Adaptar hop_controller al controlador híbrido de la rúbrica
   (requiere re-afinar Jacobiano y Bézier para la geometría del URDF)
2. Activar el resorte real (spring_scale=1.0) con un controlador
   que lo aproveche
3. Comparación cuantitativa sim vs robot físico cuando esté ensamblado
