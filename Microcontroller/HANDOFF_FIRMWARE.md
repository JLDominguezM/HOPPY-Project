# HOPPY — Handoff del firmware / puesta a punto física (2026-06-09)

Estado para retomar el robot físico. La **simulación** (salto forward) está hecha y documentada
en `mujoco/CONTROL_FORWARD.md`. Este doc cubre el **robot físico + firmware**.

## Dónde estamos (resumen)
- **PUESTA A PUNTO FÍSICA: COMPLETA** ✅ — encoders y motores verificados con signos correctos,
  la pierna llega a sus poses bajo control, seguro (con la pierna sujeta en el aire).
- **Sensor de pie:** lee bien (ISR arreglado), con un pendiente mecánico (ver abajo) diferido al
  salto en piso.
- **Controlador AÉREO (colocación de pie por IK):** escrito y la IK **verificada** (valores
  coinciden con el cálculo a mano). Movimiento probándose.
- **Falta para el salto en piso:** balancear el boom (contrapeso ~3.83 kg @ 35 cm — el user lo
  está consiguiendo; solo se necesita para el salto real, NO para aéreo/apoyo en banco).

## Archivos (firmware)
- **`blinky_rtos_flash/cpu01/cpu01_main.c`** = el firmware ACTIVO (lo que se flashea). Base: el
  ejemplo de Kevin Murphy (`cpu01_main_EXAMPLE.c`) + todos los cambios de abajo.
- `cpu01_main_PRUEBA_LENTA.c` = copia fuente de la versión de puesta a punto (respaldo).
- `cpu01_main_FASE1_PD_backup.c.bak` = respaldo del ejemplo de la Fase 1 (PD demo). `.bak` para
  que CCS no lo compile. Para volver a la Fase 1, copiar su contenido sobre `cpu01/cpu01_main.c`.
- Toolchain (de la Fase 1): CCS 12.8.1, compilador C2000 22.6.1, SYS/BIOS 6.76.04.02, XDS100v2.
  Flashear = botón Debug (bug verde) → se detiene en `main` → Resume (F8). Edita en
  `cpu01/cpu01_main.c` (NO dejar `.c` extra en esa carpeta = símbolos duplicados).

## Cambios hechos al firmware (TODOS verificados en banco), con líneas aprox.
1. **Bug LKF** (`init_values`, ~L228): el ejemplo declara `LKF` y NUNCA lo asigna (queda 0 → IK y
   Jacobiano rotos). Fix: `LKF = sqrt(DK*DK + LK*LK);` = 0.16349 (pantorrilla efectiva).
2. **Polaridad encoder de cadera** (`init_EQEP(&eqep1...)`, ~L492): `-1 → 1`. El encoder leía
   "muslo adelante" como NEGATIVO; el firmware espera adelante = +q0. Rodilla (eqep2, L501) queda
   en `1` (ya estaba bien: flexión = −q1, verificado).
3. **Signo del motor de cadera** (`prevent_saturation`, ~L350): `u[0] = -(10/64.125)*u[0]` →
   `+(10/64.125)*u[0]`. Con `-` el motor de cadera corría al revés del encoder (se disparaba).
4. **Signo del motor de rodilla** (`prevent_saturation`, ~L351): queda `+(10/69.727)*u[1]` (el
   original). Se PROBÓ `-` y empujaba hacia EXTENSIÓN (mal, confirmado a mano) → revertido a `+`.
5. **Sensor de pie — ADC directo** (`get_phase`, ~L211): el `ADCD_ISR` del ejemplo **NO está
   enganchado** al RTOS (en `cpu01_rtos.cfg` solo hay Hwi para SCI; ninguno para ADCD) → nunca
   corre → `analog_in` se quedaba en 0. Fix: leer el resultado DIRECTO:
   `analog_in[0] = AdcdResultRegs.ADCRESULT0;` (la conversión ya se dispara cada ms al final de
   `DoEveryMilliSecond`). Sensor de pie en **ADCIN-D0**.
6. **Controlador AÉREO** (`calculate_traj`, ~L226 + prototipo `void pose_to_joint_space(void);`
   ~L97): reemplaza la rampa de waypoints. Calcula el objetivo del pie `pos_des` (cicla ±5 cm
   tangencial a `pos_des[1]=-0.22` de profundidad), llama `pose_to_joint_space()` (IK → `q_ref` en
   convención FK) y convierte a convención del encoder: `q_ref[0]-=beta_off; q_ref[1]+=beta_off;`.

## Variables de prueba (globales, afinables EN VIVO en CCS Expressions)
- `MOTORS_OFF` (L179): **1** = solo calcula la IK sin mover (verificar `q_ref`); **0** = mueve.
- `Kp_test = 120` (L180): P alto, así el límite domina el torque. `Kd_test = 0` (L181): sin
  amortiguamiento → sin zumbido (la derivada cruda del encoder es ruidosa).
- `u_lim_test = 4.0` (L182): límite de pseudo-pwm (de 10). La rodilla necesita ~4 para vencer su
  resorte; la cadera (sin resorte) usa poco. Subir en vivo si la rodilla no alcanza.
- `beta_off = 0.70` / `beta_off_k = 0.70`: offsets del cero (tubo vertical) a la convención FK,
  separados cadera/rodilla. **ESTIMADOS ~40°**; afinar en vivo (pie adelante → subir `beta_off`;
  pie muy alto/corto → subir `beta_off_k`).
- `traj_amp = 0.05`, `traj_x0 = 0.0`, `traj_depth = -0.22`, `traj_T = 8000`: la trayectoria aérea
  de prueba, afinable EN VIVO (`calculate_traj` recalcula `pos_des` cada ms con estas — **NO**
  editar `pos_des` directo, se sobreescribe). `traj_amp=0` = pie quieto (modo calibración).

## Convenciones (importante)
- **q0 = cadera**, **q1 = rodilla** (rad). FK: q=0,0 = pierna recta hacia abajo. **+q0 = pie
  adelante**, **−q1 = rodilla flexionada**. Escala verificada (~45° real → q0≈0.79 rad).
- **CERO físico** = tubo (pantorrilla) **vertical** + rodilla en su extensión natural. Se zera
  sujetando la pierna ahí al dar Resume (el firmware fija `q=0` en esa pose). Como la rodilla NO
  se extiende del todo, en convención FK esa pose es `(+β, −β)` con β≈40° → de ahí `beta_off`.
- Sensor de pie: suelto=0, presionado hasta 4095, ruido de movimiento ~17. `analog_limit=2048`
  separa limpio. Fase: `phase = (analog_in[0] >= analog_limit)`.

## Pendiente del sensor de pie (DIFERIDO, mecánico)
Al apoyar el regatón **recto** contra el piso NO sube; solo en **ángulo** sube (y bajo umbral). Es
acople mecánico regatón↔SoftPot. **Diferido**: el impacto real del aterrizaje (con peso + velocidad)
es muy distinto a apretar con el dedo, así que se evalúa/afina en el salto en piso (umbral o ajuste
mecánico). El SoftPot da POSICIÓN del punto de presión, no fuerza.

## VERIFICADO esta sesión
- Encoders: cadera adelante→+q0 ✅, rodilla flexión→−q1 ✅, escala ✅.
- Motores: ambos empujan HACIA el objetivo (sin disparos) ✅. La pierna cicla agacharse↔extenderse
  siguiendo objetivos ✅.
- IK aérea: `q_ref` coincide con el cálculo a mano (ej: `pos_des[0]=0.029` → `q_ref[0]=0.160`,
  `q_ref[1]=-0.434`) ✅.
- Sensor de pie: `analog_in[0]` cambia (0↔4095), `phase` cambia al presionar firme ✅.

## ⚠️ MORFOLOGÍA REAL DE LA PIERNA (aclarada 2026-06-10 — INVALIDA supuestos anteriores)

**La pierna del rediseño es TIPO AVE, espejo del HOPPY original:**
- La "rótula" apunta hacia ATRÁS; **flexionar la rodilla manda el pie hacia ADELANTE**.
- El tubo NUNCA se alinea con el muslo: en el tope de extensión ya forma ~134° con el muslo
  (~46° "doblado" hacia adelante de fábrica).
- Consecuencias aplicadas al firmware (2026-06-10, todas en `cpu01_main.c`):
  1. **IK rama espejo** en `pose_to_joint_space`: `q_ref[0] = atan2 − acos` (muslo queda ATRÁS
     de la línea cadera-pie) y `q_ref[1] = +acos` (rodilla efectiva POSITIVA).
  2. **Mapa del 4-barras con signo volteado**: `KA=−0.454, KB=−1.534, KC≈+0.80` (KC = ángulo
     efectivo en el cero, afinar con plomada). Inversa con rama `−sqrt`.
  3. **POSE CERO NUEVA (la vieja "tubo vertical+extensión natural" es FÍSICAMENTE IMPOSIBLE
     en esta pierna):** MUSLO VERTICAL (aplomar con teléfono en las placas del muslo) +
     rodilla en su TOPE DE EXTENSIÓN (resortes relajados, fácil de sostener). Re-zero en vivo:
     escribir 0 en `EQep1Regs.QPOSCNT` y `EQep2Regs.QPOSCNT` sosteniendo esa pose.
- Pose IK de referencia: pie en (0,−0.22) → `q_ref=(−0.751,−0.256)` (muslo atrás, postura de ave).
- OJO Etapa 2: revisar signos del mapeo en `mujoco/CONTROL_FORWARD.md` contra esta morfología
  (la sim MuJoCo usa el URDF real así que su cinemática SÍ es la de ave; el Jacobiano del
  firmware ya usa `q_fk` correcto).

## Sesión 2026-06-10 — Etapa 1: calibración aérea → HALLAZGO MECÁNICO (rodilla desacoplada)

**Estado: Etapa 1 PAUSADA por reparación mecánica.** El acople del eje del motor de rodilla
está BARRIDO: el motor gira (encoder feliz) pero patina dentro de la pieza que mueve el
4-barras → la pantorrilla NO sigue al motor. Confirmado con: (a) fotos a q_test[1]=0/−0.4/−0.8/−1.2
→ el tubo queda en ~40° en TODAS; (b) test de la mano: la rodilla se mueve sin que q_now[1] cambie;
(c) el usuario vio el eje girar patinando. Reparar (prisionero sobre el plano del eje D + Loctite,
o reemplazar la pieza si el barreno está redondeado) y re-verificar con JOINT_MODE.

**Lo que SÍ quedó verificado/calibrado esta sesión:**
- **Cadera: CALIBRADA.** Offset real ≈ 0 (no 0.70) y escala correcta (motor 26.9:1 como asume
  eqep1). Verificado con IK inversa de mediciones con regla + foto (muslo a 42° cuando comanda 0.746).
- **`beta_off` y `beta_off_k` reales ≈ 0.0** (el estimado de 40° era incorrecto; la pose cero
  ≈ pose recta de la FK). Pendiente re-confirmar rodilla tras la reparación.
- **Ganancias de banco afinadas** (ya como default en el código): `Kp_test={400,600}` vencen
  fricción de cadera y resorte de rodilla con error <0.01 rad, sin oscilar, `u_lim_test=4` sobra
  (el mapeo ×10/64-70 hace que la saturación casi nunca muerda — subir Kp, no u_lim).
- La IK del firmware verificada de nuevo en target: pos_des (0,−0.22) → q_ref (0.0507,−0.4628). ✓
- OJO firmware: eqep2 asume 806.4 cnt/rad (motor 19.2×banda 1.5 del HOPPY original); el robot REAL
  usa motor 26.9:1 + 4-barras SIN banda → la escala efectiva motor→rodilla la pone el 4-barras
  (no constante). Tras la reparación, medir la relación efectiva con JOINT_MODE + fotos de perfil
  y decidir si basta corregir el número o hace falta mapa no lineal.

**Agregado al firmware esta sesión** (`cpu01_main.c`):
- `JOINT_MODE` + `q_test[2]`: objetivos de junta crudos (encoder), sin IK — para calibración.
- `traj_amp/traj_x0/traj_depth/traj_T`: trayectoria aérea afinable EN VIVO (antes pos_des se
  sobreescribía cada ms y "cambiarlo en vivo" no hacía nada).
- `beta_off_k` separado de `beta_off`. Defaults seguros: `MOTORS_OFF=1`, `traj_amp=0`.
- Defaults del código aún traen `beta_off=beta_off_k=0.70`: al retomar, ponerlos en 0.0
  (cadera confirmada; rodilla por confirmar tras reparación).

## ✅ ETAPA 1 CERRADA (2026-06-10, noche)

Verificado en banco con la pierna sujeta: pose IK pie en (0,−0.22) → **plomada ≈ 0 cm**,
diagonal ~22-24 cm (medida al regatón), `pos_now` coincide con la física, y el barrido
aéreo (`traj_amp=0.05`) mueve el pie entre poses. La pierna coloca el pie donde se le ordena.

Cronología de la sesión (para contexto): (1) acople motor-rodilla barrido → reparado por el
usuario; (2) encoder rodilla con falso contacto (PHE, cuentas perdidas) → curado al re-asentar
conectores; verificado con test de topes duros (5 ciclos, ±0.03 rad); (3) detectada la
MORFOLOGÍA TIPO AVE (sección arriba) → IK espejo + mapa con signo + pose cero nueva → todo
cuadró a la primera. Los encoders alimentan a 4.78V (pin 5V, diagrama original) — los GPIO del
F28379D no son 5V-tolerantes; funciona, pero es deuda técnica si un día se queman entradas.

## NEXT STEPS (en orden)
1. ~~Terminar Etapa 1 (aéreo, banco, pierna sujeta)~~ ✅ HECHA (ver sección de cierre arriba).
   Los knobs vivos quedaron: `KC` (offset fino del pie), `traj_amp/x0/depth/T` (trayectoria),
   `Kp_test={400,600}`, `u_lim_test`. `beta_off_k` YA NO EXISTE (lo reemplazó el mapa KA/KB/KC).
2. **Etapa 2 (apoyo, banco):** escribir el control de APOYO — GRF Bézier `Fx=−25, Fz=100` vía
   `u = -Jᵀ·[Fx;Fz]` (usar `update_jacobian`) + PD suave; `Tst=0.35`. Probar que la pierna empuja
   (apoyada/sujeta). Agregar **filtro de velocidad** (low-pass de qdot) para poder usar `Kd` sin
   zumbido. Mapeo de constantes en `mujoco/CONTROL_FORWARD.md`.
3. **Etapa 3 (salto en piso):** unir aéreo+apoyo con la FSM (fase por sensor de pie). Necesita el
   **boom balanceado** (contrapeso ~3.83 kg @ 35 cm). Ahí se afina el umbral del sensor de contacto
   y las ganancias. Empezar con la pierna sujeta, luego soltar.
4. La dirección de avance (forward) la fija el signo del empuje tangencial `Fx` (como en la sim).

## Referencias
- Constantes reales y mapeo al firmware: `mujoco/CONTROL_FORWARD.md`.
- Contrapeso (cálculo): `mujoco/contrapeso.py` (físico ~3.83 kg @ 35 cm con hopper=2075 g @ 0.85 m).
- Sim forward validada: `mujoco/view_hop_urdf.py --viewer` (controlador `controller.py` +
  `hoppy_urdf.FORWARD`).
