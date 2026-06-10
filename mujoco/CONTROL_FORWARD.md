# HOPPY URDF — salto hacia ADELANTE con el controlador REAL

`python3 view_hop_urdf.py --viewer` ahora salta **hacia adelante** (al lado **sin** pierna)
usando el **controlador híbrido real** (port fiel del simulador MATLAB del paper = el control
que correrá en la LaunchPad F28379D), con las **constantes reales** y el boom **balanceado**.

- Antes: FSM tonto (`hop_controller.py`), giraba `+theta1` = hacia la pierna.
- Ahora: control real (`controller.py` + `hoppy_urdf.FORWARD`), gira `−theta1` = al lado sin
  pierna, ~0.58 rad/s, pie despega ~7.6 cm, 66% de vuelo, ciclo límite estable, **8/8 checks**.
- `--fsm` sigue mostrando el viejo para comparar. Diagrama: `figuras/diag_forward_vs_actual.png`,
  señales `figuras/señales_forward.png`, render `figuras/render_forward_secuencia.png`.
- **Pie/pose:** `p_toe_z=-0.18` (aterriza más extendida, pantorrilla ~43° de la vertical, antes
  ~50°). La pierna se mantiene **idéntica al URDF real del CAD** (4 barras, resortes, sensor); el
  contacto físico es un punto en el **regatón al final del shank** (`FOOT_TIP`), la geometría
  real — así la cinemática del sim coincide con el robot y las ganancias transfieren. **Sensor de
  pie:** el firmware detecta fase por `analog_in[0] >= analog_limit` (cuánto se retrae/comprime el
  sensor cruza un umbral); el sim la detecta por **fuerza de contacto > umbral**, el análogo
  directo — el mecanismo que importa para el control está modelado, aunque la malla no muestre el
  sensor moviéndose. Ver `figuras/foot_revertido.png`. (Se probó añadir una punta/spike o
  simplificar la malla, pero eso cambiaba la geometría real → descartado.)

## Por qué ahora SÍ va hacia adelante (y antes no)

1. **El sentido lo fija el empuje tangencial del apoyo** (el GRF Bézier `Fx`, pico **−25 N**,
   valor real del MATLAB), NO la geometría ni el `vx_d`. Con `Fx=−25` el cuerpo avanza `−theta1`,
   exactamente como el MATLAB de referencia (que avanza a −0.565 m/s). La métrica vieja
   ("pie tangencial mismo signo que la velocidad") era **engañosa**: clasificaba el salto
   realista del propio MATLAB como "pierna-adelante". El discriminador correcto es el barrido
   del pie touchdown→liftoff (aterriza adelante, empuja atrás = Raibert forward).

2. **El boom hay que BALANCEARLO** (contrapeso). Sin balanceo el CoM del boom queda ~0.55 m
   hacia el hopper → **~13 N·m** de gravedad sobre el pitch que el motor (con su límite real de
   12 V) **no puede vencer**: el cuerpo solo "bobea" sin que el pie despegue (lo confirmamos:
   con el motor real saturaba a 12 V y 0.1 cm de despegue). El HOPPY del paper y el modelo
   MATLAB (`rx2=−0.50`) están balanceados por diseño; por eso saltan.

   **Cálculo del contrapeso (balance de momentos)** — ver `contrapeso.py`:
   `M_cw · d_cw = M_hopper · d_hopper`. Desbalance medido en sim = **12.9 N·m** (M_hopper=2.38 kg
   a 0.55 m). A tu brazo real **d_cw = 0.35 m**: balance total = 3.77 kg, pero un saltarín NO se
   balancea al 100% (deja al pie sin "peso de cuerpo" → sobrelanza, en sim explota a 68 cm). El
   sweet spot es **~76% de balance = 2.86 kg @ 0.35 m** (deja ~3 N·m de peso efectivo en la
   pierna). Eso usa `hoppy_urdf.FORWARD` y salta limpio. Margen 2.6–3.2 kg.

   > **IMPORTANTE para el físico:** el robot real necesita ese balanceo. El BOM del gantry no
   > trae contrapeso. Para TU boom (hopper a ~85 cm, no 55 cm como el CAD), el momento es mayor:
   > con `contrapeso.py` (M_hopper medido y d_hopper=0.85, d_cw=0.35) sale **~4.4 kg @ 35 cm**
   > (~76%). Lo más exacto: **pesa el lado hopper** y/o mide qué peso a 35 cm deja el boom
   > horizontal sin contrapeso (= 100%), y usa ~76% de eso. Sin balancear, NO despegará.

## Constantes reales (verificadas en sim) → firmware `cpu01_main.c`

Todas salen de `Simulator_MATLAB/fcns/get_params.m` y `dyn_aerial/dyn_stance.m`. Están en
`hoppy_urdf.FORWARD` (y `forward_best.json`). Lazo a **1 kHz**, fase por **sensor de contacto**.

| Concepto | Sim / MATLAB | Firmware `cpu01_main.c` |
|---|---|---|
| Dimensiones pierna | LH=0.096, DK=0.052, LK=0.1545 | `LH .0960`, `DK .0520`, `LK .1550` (ya están) |
| Fase | contacto del pie | `get_phase()` (`analog_in[0] >= analog_limit`) |
| **Aéreo**: PD cartesiano del pie | `Kp_sw=150, Kd_sw=5` (N/m), `u=Jᵀ·F` | equivale a `pose_to_joint_space()`→`q_ref` + PD `Kp_a/Kd_a` |
| Colocación de pie (Raibert) | `p_des=[Krh·vx, −0.15]`, `Krh=0.10` | `pos_des[0]=Krh*vx; pos_des[1]=-0.15;` antes de `pose_to_joint_space()` |
| **Apoyo**: GRF Bézier | `Fz_bz=[0,20,100,0,0]`, `Fx_bz=[0,0,−25,0,0]`, `Tst=0.35` | escribir en `calculate_control()` (rama `else`): `u = −Jᵀ·[Fx;Fz] + tau_fb` |
| PD suave de apoyo | `Kp_st=0.03, Kd_st=0.08`, `q_d=[π/3,−π/2]` | `tau_fb = Kp_st*(q_d−q) + Kd_st*(−qdot)` |
| Despegue | `GRFz < 1.5 N` | en `get_phase()` o por `GRFz` |
| Motor goBILDA | Rw=1.3, kT=0.0135, kv=0.0186, Nh=26.9, Nk=28.8 | encoders `753.2 / 806.4`; sat `±10` (≡ ±64/±70 N·m) |
| Saturación | 12 V / 9.2 A | `prevent_saturation()` (mapea N·m→[−10,10]) |

Notas de mapeo:
- El aéreo del MATLAB es **cartesiano** (`Jᵀ·F_sw`); el firmware-ejemplo lo hace como **PD de
  junta a un objetivo IK** (`pose_to_joint_space` + `Kp_a`). Son equivalentes: el `Kp_a=3000`
  del ejemplo es la versión en espacio de junta del `Kp_sw=150` cartesiano. Usa el que prefieras;
  el cartesiano es el del paper.
- El **signo del empuje tangencial `Fx`** define el sentido de avance. Si en el físico avanza
  al revés, invierte el signo de `Fx_bz` (o de la columna tangencial de `Jᵀ`).
- `q_d=[π/3,−π/2]` es la pose nominal del MATLAB; en el URDF la cuclilla equivalente es
  `theta3≈0.55, theta4≈−0.95` (otro cero de encoder). Verifica el cero de tus encoders.
