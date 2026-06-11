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

## ✅ ETAPA 2 CERRADA (2026-06-11): control de apoyo verificado en banco,
## fuerza real medida ~3 kg (~30 N) en el pie — ver bitácora al final de la sección

Se escribió en `cpu01_main.c` el control de APOYO (Ec.19 del paper, port fiel de
`controller.py`/`mujoco/CONTROL_FORWARD.md`), verificado numéricamente contra la sim:
- **`bezier4()`** (4to orden, idéntico bit-a-bit a `twin.bezier`) con los puntos reales
  `Fz_bz=[0,20,100,0,0]`, `Fx_bz=[0,0,−25,0,0]` → pico REAL del perfil ≈ 42.5 N a s=0.5.
- **`calculate_control()` reescrito**: `tau_fk = −Jᵀ·[Fx;Fz] + Kp_st·(qd_st_fk−q_fk) − Kd_st·qfk_dot`
  (espacio FK) y mapeo FK→motor: cadera 1:1, **rodilla MULTIPLICADA por `f'(e)=2·KA·e+KB`**
  (trabajo virtual del 4-barras: `τ_m = τ_FK·f'(e)`; el comentario viejo decía "dividir" — era
  INCORRECTO, ya corregido en `update_jacobian`). Como `f'<0`, el signo del motor sale solo.
  `qd_st_fk` = pose FK capturada al iniciar el empuje (análogo del q_d de touchdown del MATLAB).
- **Blending aéreo→apoyo** (Ec.20): `blend_ms=10`. Al terminar el empuje, el PD aéreo (Etapa 1,
  intacto) recoge la pierna a la cuclilla.
- **Filtro de velocidad** (= Fase 5 de la sim, λ=10 rad/s): `qdot_f` (1er orden) alimenta TODOS
  los términos D (aéreo y apoyo) → ya se puede subir `Kd_test` sin zumbido. `qfk_dot` = velocidad
  FK (rodilla vía regla de la cadena). `vel_lambda` afinable en vivo (subir a 20–30 si la D va lenta).
- **`MOTORS_OFF` ahora va AL FINAL**: con motores apagados TODO se calcula igual → dry-run de la
  ley mirando `F_des`, `tau_fk`, `u_st`, `u_air` en Expressions.

**Knobs nuevos (Expressions):** `STANCE_TEST` (habilita), `st_go` (1 = UN empuje, se auto-limpia),
`st_auto`+`st_period` (cíclico, ms), `st_sensor` (dispara al pisar el sensor, flanco 0→1 de `phase`),
`Tst=0.35`, `fz_scale`/`fx_scale` (escala de perfiles; el SIGNO de fx fija el sentido de avance),
`Kp_st=0.03`/`Kd_st=0.08`, `blend_ms=10`, `vel_lambda=10`. (Ojo: `Kp_s`/`Kd_s` del ejemplo siguen
ahí pero NO se usan.) Defaults seguros: `MOTORS_OFF=1`, `STANCE_TEST=0`, `st_go=0`.

**Valores esperados (calculados con la réplica exacta del firmware; la pose de referencia
reproduce el (−0.751, −0.256) verificado en Etapa 1):**
- Cuclilla del protocolo: pie en (0, −0.20) → `q_ref` encoder = **(−0.945, −0.489)**,
  `q_fk=(−0.945, 1.441)`, `f'(e)=−1.090`, `J=[[0.200, 0.144],[0.000, 0.078]]`.
- Empuje (fz/fx_scale=1) en esa pose, a mitad (s=0.5): `F_des=(−9.4, 42.5)` N,
  `tau_fk=(+1.88, −1.96)` N·m, `u_st=(+1.88, +2.14)` N·m → pwm ≈ (5.6, 6.0) con el mapeo
  Ec.18 (los empujes piden `u_lim_test` 8–10). **Ambos u_st POSITIVOS** (cadera adelante,
  rodilla a extensión); `tau_fk[1]` negativo con `u_st[1]` positivo es la firma del `f'<0` —
  si no se ve así, el mapa está mal.

**Protocolo de banco (pierna sujeta, en orden):**
1. Flashear, pose cero de Etapa 1 (muslo vertical + rodilla en tope), Resume.
2. **Dry-run** (motores quietos): sostener el pie ~20 cm bajo la cadera. `STANCE_TEST=1`,
   `Tst=5.0` (lento para verlo), `st_go=1` → ver `F_des[1]` ir 0→~42→0 y `u_st` ≈ (+1.9, +2.1)
   máx con los signos de arriba. Regresar `Tst=0.35`.
3. **Regresión Etapa 1**: `STANCE_TEST=0`, `traj_depth=−0.20`, `MOTORS_OFF=0` → la pierna va a
   la cuclilla. Probar el filtro: `Kd_test={5,5}` no debe zumbar (antes con la cruda zumbaba).
4. **Primer empuje real**: pie apoyado al piso/tabla (housing sujeto). `STANCE_TEST=1`, `st_go=1`
   → ~0.35 s: extiende empujando ABAJO y un poco hacia ADELANTE (+x; el pie tiende a patinar
   adelante), luego regresa solo a la cuclilla. Débil → subir `fz_scale` en vivo (1.5–3).
5. **Ciclo**: `st_auto=1` (`st_period=3000`) → empuje cada 3 s, ver repetibilidad.
6. **Preview Etapa 3** (opcional): `st_auto=0`, `st_sensor=1` → pisar el sensor dispara el
   empuje (de paso evalúa el pendiente mecánico del sensor de pie).

### Bitácora banco 2026-06-10 (noche) — Etapa 2: qué pasó y DÓNDE SE PAUSÓ

**Verificado en banco (la ley funciona exacta):**
- **Dry-run ✓ EXACTO**: capturas a `st_t≈0.5` y `≈1.6` de un empuje lento (Tst=5) calzaron con
  la réplica numérica al 1%: pico `F_des=(−8.7, 41.7)` vs (−9.4, 42.5) predicho; `u_st=(−5.39,
  +8.01)` vs (−5.4, +8.1) para la pose colgada-en-tope; firma `f'<0` visible en vivo
  (`u_st[1]/tau_fk[1] = −1.534 = KB` exacto en los residuos).
- **Regresión aérea ✓**: con motores ON la pierna sostiene la cuclilla, pie a 4 mm de la
  plomada (`pos_now=(0.004, −0.2205)` para target (0,−0.22)).
- **El empuje dispara y mueve la pierna en el aire ✓** (st_go/blend/recuperación OK).

**HALLAZGO 1 — el mapeo pwm del ejemplo entregaba ~5% del torque.** El `u×10/64.125` del
ejemplo trata "64 N·m" como pwm completo, pero pwm completo = 12 V = ~3.4 N·m del goBILDA a
rotor parado → los torques reales del MATLAB (2–5 N·m) salían como ~5% del voltaje (el empuje
ni se sentía; en el aire sí movía porque ahí basta cualquier torque). **FIX APLICADO y
flasheado**: `prevent_saturation` ahora es la Ec.18 física (`V = Rw/(kT·N)·u + kv·N·qdot_f`,
pwm=10·V/12, constantes Rw/kT/kv/NH=26.9/NK=28.8 como globales) y `Kp_test={20.9, 30.9}` N·m/rad
(equivalentes EXACTOS de los {400,600} de banco: mismo pwm/rad ±0.1%, verificado — `u[1]` en
cuclilla lee ≈−0.56 igual que antes). Nota: NK=28.8 es la reducción efectiva en unidades de e
(la escala 806.4 del eqep2 convierte el 26.9 físico en 28.8 = el N_K del PDF; todo consistente).

**HALLAZGO 2 — test de fuerza con báscula: ~1/3 de lo esperado y CEROS CORRIDOS.**
Test isométrico (pie en báscula de baño, `fx_scale=0`, `Tst=3`, housing presionado): esperado
2.5–4.5 kg pico; midió **0.5 → 1.0 → 1.2 kg** en iteraciones sucesivas (mejorando con
metodología), insensible a `u_lim_test` 8→10 (OJO: ese "test" no discriminaba nada — la
demanda era ~8.9 pwm, el cambio era solo +11%). Sin contrapeso el techo con el cuerpo LIBRE es
~1.6 kg (peso efectivo del boom desbalanceado: 13 N·m/0.85 m) — pero clavando el housing
siguió bajo. **Al final se chequeó el cero: en la pose cero física `q_now ≈ (0.5, 0.3)` — LOS
DOS ENCODERS CORRIDOS.** Ahí se pausó la sesión.

**Dos hipótesis abiertas (el protocolo de abajo discrimina):**
- **(A) El cero quedó mal desde el Resume del re-flasheo** (la pose cero no se sujetó exacta).
  Explicaría TODO sin nada roto: cuclilla comandada = otra pose física (rodilla casi en tope →
  sin recorrido → solo empuja la cadera → ~1 kg), Jacobiano chueco (la báscula se arrastraba),
  y "no regresa a la posición inicial" = regresa a la cuclilla corrida.
- **(B) Los acoples motor→eje patinan bajo carga** (el de rodilla ya se reparó una vez,
  2026-06-10 mañana; el empuje le mete ~3 N·m vs ~0.5 de la Etapa 1). El corrimiento se
  acumularía empuje a empuje. Cadera también corrida apunta un poco a (A) o a patinaje en
  los transitorios de recuperación (PD a 8 pwm).

**PROTOCOLO DE REANUDACIÓN (primero esto, en orden):**
1. Rayar con marcador una línea eje↔maza en los DOS acoples (testigo de patinaje).
2. Re-zero en vivo: pose cero sujetada → `EQep1Regs.QPOSCNT=0`, `EQep2Regs.QPOSCNT=0` →
   `q_now=(0,0)`. Mover la pierna a mano y regresar → debe repetir (0,0)±0.03.
3. `MOTORS_OFF=0` → cuclilla. VERIFICAR FÍSICAMENTE que se ve bien (plomada + profundidad).
4. UN empuje isométrico (báscula, `fx_scale=0`, `fz_scale=1`, `Tst=3`, `u_lim_test=8`,
   housing clavado) → anotar pico. Esperado si el cero era el problema: **2.5–4 kg**.
5. `MOTORS_OFF=1` → pose cero a mano → leer `q_now` + revisar rayas:
   - (0,0) y rayas alineadas → era (A): nada roto, el pico del paso 4 es el número real.
   - corrido otra vez / raya desalineada → (B): reparar acople (prisionero en el plano D +
     Loctite CURADO, segundo prisionero a 90° o pasador) y repetir.
6. Si el pico sigue ≤1.5 kg con cero bueno y acoples firmes: medir eficiencia del tren
   (fricción 4-barras/engranes) — sospechoso siguiente.

**Estado de knobs al pausar** (tras re-flasheo se resetean a defaults seguros: `MOTORS_OFF=1`,
`STANCE_TEST=0`, `fz_scale=1`, `Tst=0.35`): en la sesión quedaron en vivo `fx_scale=0`,
`Tst=3.0`, `u_lim_test=10`, `traj_depth≈−0.18` (altura de la báscula).

### Bitácora banco 2026-06-11 — RESOLUCIÓN del misterio de la fuerza y CIERRE de Etapa 2

**1. Mecánica DESCARTADA** (hipótesis B muerta): cero firme tras empujes, rayas de marcador
alineadas, y el tope duro de la rodilla reproduce en `e=+0.3506` exacto entre empujes.
- **Dato nuevo del mecanismo:** la pose cero ("tope de extensión, resortes relajados") es el
  REPOSO del resorte, NO el tope duro — el tope duro real está **+0.35 rad más allá** (e>0 =
  zona de resortes flojos). El mapa FK extrapola ahí; en marcha no se usa (el apoyo es flexión).

**2. BUG REAL ENCONTRADO Y ARREGLADO — `abs()` entero en el driver PWM**
(`custom_support/source/F28379D_EPwm.c`, `set_EPWM1A/1B_VNH5019`): `abs(u)` truncaba el float →
**zona muerta de ±1.2 V** (|u|<1 = 0% duty: la cuclilla se "sostenía" con el motor APAGADO, por
fricción) y **duty en escalones del 10%** (8.87→8; por eso subir u_lim 8→10 "no hacía nada").
Fix: `(u<0?-u:u)`. Tras el fix, el PD sostiene la cuclilla con error 0.009/0.0001 rad
(antes 0.025/0.0065).

**3. EL HALLAZGO CONCEPTUAL — el "empuje débil" era ESCAPE GEOMÉTRICO, no falta de torque.**
Con `u=−Jᵀ·F`, si el pie no está bloqueado (pie patina, cuerpo sube, aire), la pierna se
extiende y se ALINEA con la dirección de la fuerza pedida → en esa configuración la ley pide
~cero torque (la línea de fuerza pasa por las juntas; es el equilibrio correcto). Los 0.5–1.2 kg
de las primeras básculas eran ESTE artefacto. Sostener el housing a mano NO basta (el empuje
mete ~4 kg extra). **Instrumentación agregada a `cpu01_main.c`**: picos congelados del último
empuje — `pk_Fz`, `pk_pwm[2]` (pwm final enviado), `pk_e_min/max` (viaje de la rodilla).
**Criterio de validez del test isométrico: `pk_pwm[1]≈9–10` y `pk_e_max` lejos del tope** —
si no, la pierna escapó y la lectura de la báscula no vale.

**4. MEDICIÓN FINAL (cierra la etapa): ~3 kg (~30 N) de empuje vertical en el pie.**
Setup válido: báscula+tabla elevadas con libros hasta cuclilla profunda (`traj_depth≈−0.17`,
`e≈−0.74`, ahí el 4-barras AMPLIFICA el torque), **boom amarrado a la mesa** (no manos),
`fx_scale=0`, `u_lim_test=10`, `Tst=3`. Resultado ≈ rango de diseño (3–4 kg esperados con
pérdidas; la sim salta con picos comandados de ~40 N). La cadena ley→Ec.18→driver→motor→
4-barras→pie queda VALIDADA punta a punta.

**Qué queda para Etapa 3:** el contrapeso (~4 kg @ 35 cm — sin él NO despega, confirmado
también empíricamente: el techo del empuje con cuerpo libre es el peso efectivo ~1.6 kg),
unir aéreo+apoyo con la FSM por sensor de pie (`st_sensor=1` ya es el embrión funcionando)
y afinar `analog_limit` con datos reales de contacto.

## NEXT STEPS (en orden)
1. ~~Terminar Etapa 1 (aéreo, banco, pierna sujeta)~~ ✅ HECHA (ver sección de cierre arriba).
   Los knobs vivos quedaron: `KC` (offset fino del pie), `traj_amp/x0/depth/T` (trayectoria),
   `Kp_test={400,600}`, `u_lim_test`. `beta_off_k` YA NO EXISTE (lo reemplazó el mapa KA/KB/KC).
2. ~~Etapa 2 (apoyo, banco)~~ ✅ **CERRADA 2026-06-11**: ley exacta + fuerza real ~3 kg
   isométrica en el pie (ver bitácora). En el camino se arreglaron 2 bugs de torque: mapeo
   pwm→Ec.18 física y `abs()` entero en el driver PWM (zona muerta + escalones del 10%).
3. **Etapa 3 (salto en piso):** unir aéreo+apoyo con la FSM (fase por sensor de pie — `st_sensor`
   ya es el embrión). Necesita el **boom balanceado** (contrapeso ~3.83 kg @ 35 cm). Ahí se afina
   el umbral del sensor de contacto y las ganancias. Empezar con la pierna sujeta, luego soltar.
4. La dirección de avance (forward) la fija el signo del empuje tangencial `Fx` (como en la sim):
   el robot avanzará hacia −x de la pierna (el análogo del −theta1 de la sim); si va al revés,
   voltear el signo de `fx_scale`.

## Referencias
- Constantes reales y mapeo al firmware: `mujoco/CONTROL_FORWARD.md`.
- Contrapeso (cálculo): `mujoco/contrapeso.py` (físico ~3.83 kg @ 35 cm con hopper=2075 g @ 0.85 m).
- Sim forward validada: `mujoco/view_hop_urdf.py --viewer` (controlador `controller.py` +
  `hoppy_urdf.FORWARD`).
