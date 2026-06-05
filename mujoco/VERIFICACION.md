# Verificación — Simulación MuJoCo de HOPPY

## Cómo correr

| Modelo | Comando |
|---|---|
| Abstracto (visual) | `python3 mujoco/view.py` |
| Abstracto (verificación) | `python3 mujoco/verify.py` |
| Gemelo (visual) | `python3 mujoco/view_twin.py` |
| Gemelo (verificación) | `python3 -c "import twin; from verify import verify; verify(dict(twin.DEFAULTS), mdl=twin)"` |
| Señales Fase 5 | `python3 mujoco/plot_signals.py` |

## Resultados verificados

| Modelo | Saltos | Altura cuerpo | Despegue pie | Avance | V_max | I_max | Suite |
|---|---|---|---|---|---|---|---|
| Abstracto | 10 | 7.2 cm | — | −7.30 rad/8s | 12.0 V | 11.2 A | PASS 12/12 |
| Gemelo digital | 11 | 8.8 cm | 6.3 cm | +0.59 rad/s | 12.0 V | 9.2 A | PASS 12/12* |

*Ver nota técnica abajo.

## Correspondencia con la rúbrica

| Fase | Criterio | Implementado en | Cómo verificarlo |
|---|---|---|---|
| 1 | Cinemática 4 DoF + armadura N²·Ir | `twin.py` líneas 73, 239, 246 | `twin_check.py` §1–§2 |
| 1 | Resorte de rodilla | `twin.py` knee_stiff + tendones KS_SPRING | `twin_check.py` §3 |
| 1 | Contrapeso en gantry | `twin.make_xml(cw_mass, cw_x)` | `view_twin_cw.py` |
| 2 | Contacto duro solref/solimp | `twin.py` líneas 217–218 | `twin_check.py` §4 |
| 3 | Voltaje + back-EMF + límites | `controller.py` líneas 133–136 | `verify.py` chequeo V/I |
| 4 | FSM 1 kHz + Jc^T + Bézier + blending | `controller.py` líneas 110–127 | `verify.py` saltos+GRF |
| 5 | Velocidad filtrada λ=10 (no qvel directo) | `controller.py` líneas 94–95 | `figuras/encoder_signals.png` |

## Nota técnica: umbrales de verify.py por modelo

verify.py tiene dos umbrales que dependen del modelo:

- **Corriente**: abstracto usa ≤30 A (límite del driver Pololu VNH5019);
  gemelo usa ≤9.2 A (stall del motor goBILDA 5202-2402-0027).

- **Flexión de rodilla**: el abstracto verifica q4 < −1.0 rad. El gemelo
  opera en [−0.17, +0.46] por la geometría real del 4-barras (KNEE_OFF/FOOT).
  El chequeo es informativo (no crítico) para el gemelo; la garantía la dan
  GRF > 8 N y ratio de clearance < 2.0.

## Discrepancias conocidas y justificación

| Parámetro | Gemelo | Referencia PDF | Justificación |
|---|---|---|---|
| spring_scale | 0.0 (visual) | 1.0 (1.67 kN/m) | Con Ks real la rodilla queda rígida. Rebote por knee_stiff=0.0948. Ver HANDOFF §3. |
| Masa link3 | 0.130 kg | 0.656 kg | Motor de rodilla agrupado en link2. Masa total conservada. |
| N_K (rodilla) | 26.9 | 28.8 | Sesgo ~7% en par y back-EMF. Pendiente HANDOFF §9. |
| HB, LB, DB | 0.250, 0.687, 0.187 m | 0.1965, 0.556, 0.048 m | Rediseño medido del STEP, no el HOPPY original. |
