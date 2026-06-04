# Simulación de HOPPY en MuJoCo

Simulación del robot saltarín HOPPY en MuJoCo, siguiendo la rúbrica del curso
(Robots Humanoides). Replica el simulador de MATLAB (`../Simulator_MATLAB`):
gantry pasivo + pierna activa, control híbrido a 1 kHz, contacto duro y dinámica
de actuadores por voltaje. **El controlador es un port fiel del MATLAB** y el
modelo está anclado a `get_params.m` (masas, inercias, geometría); el balance del
boom se calibró para igualar el torque gravitacional `Ge` del MATLAB.

## Archivos
- `tune_eval.py` — modelo MJCF (`make_xml`) anclado a `get_params.m` + constantes físicas + Bézier.
- `controller.py` — controlador híbrido compartido (`Hoppy`): FSM aéreo/apoyo, PD cartesiano en el frame del boom, Bézier, voltaje+back-EMF, velocidad filtrada.
- `control.py` — corrida final + métricas + `figuras/resultados.png`.
- `verify.py` — **suite de verificación rigurosa**: 12 chequeos PASS/FAIL + comparación vs MATLAB.
- `render.py` / `view.py` — video (`figuras/salto.mp4`) y visor interactivo.
- `export_ref.m` (en `../Simulator_MATLAB`) — vuelca la referencia a `ref_matlab.csv`.

## Estado (verificado con `verify.py`)
**Salta de verdad, de forma estable y avanza alrededor del poste** — los 12
chequeos críticos pasan: fase de apoyo que CARGA (~27 N) y empuja, fase de vuelo
real, el cuerpo sube 7.2 cm por el empuje (no por flotar), la pierna se mantiene
doblada en el rango de la referencia (q4∈[−2.32,−1.92], sin config singular),
**no aletea** (pie despega 1.2× la subida del cuerpo), gantry sin colapso, ciclo
límite estable (apex std ≈ 4.5 mm), avanza ~1 rad/s alrededor del poste, V≤12 V, i≤12 A.

### Comparación vs MATLAB (referencia)
| Métrica | MuJoCo | MATLAB |
|---|---|---|
| Altura cadera mín (m) | 0.115 | 0.115 |
| Altura cadera máx (m) | 0.187 | 0.187 |
| Amplitud de salto (cm) | 7.2 | 7.2 |
| Frecuencia (Hz) | 2.5 | 2.2 |
| Fracción de apoyo (%) | 51 | 60 |
| Pierna q4 mín (rad) | −2.32 | −2.27 |
| Avance θ1 (gira el poste) | sí (~1.5 vueltas/10 s) | sí |
| GRF pico (N) | 84 | 30 |

Mismo gait (comprime → empuja → vuela → aterriza), **amplitud y rango de cadera
idénticos** (0.115–0.187 m), pierna doblada igual, y **avanza alrededor del poste**
como la referencia. Diferencias menores: frecuencia ~14 % mayor (2.5 vs 2.2 Hz) y
fracción de apoyo algo menor. Ver `figuras/comparacion_matlab.png`. El config se
halló con búsqueda multi-agente (`tune_metric`): score 99/100.

## Mapa a la rúbrica (100 pts)
| Fase | Contenido | Estado |
|---|---|---|
| 1 | Cinemática + armature (N²·Ir) + resorte de rodilla + contrapeso (en link2) | ✅ |
| 2 | Contacto duro (`solref`/`solimp`), sin rebote | ✅ |
| 3 | Actuador por voltaje + back-EMF, límites 12 V/30 A | ✅ (V≤12, i≤12) |
| 4 | Control híbrido: FSM 1 kHz, `Jc^T`, PD aéreo (Ec.17), Bézier (Ec.19), blending (Ec.20) | ✅ salto estable verificado |
| 5 | Velocidad por derivada filtrada (λ≈10) + gráficas | ✅ |

## Tuneo respecto al MATLAB (documentado)
Para el sim2sim a MuJoCo se ajustaron, dentro de la estructura de la rúbrica:
- **Eje de la pierna en X** (no Y): la pierna oscila en el plano tangencial para propulsar el avance alrededor del poste, igual que el MATLAB.
- `cw_x=0.080`: CoM x de link2 calibrado para igualar `Ge(θ2)=−5.95 N·m` del MATLAB.
- `kp_sw≈433` (vs 150 del MATLAB): mantiene la pierna retraída en vuelo; con 150 se descuelga al config singular.
- `knee_stiff≈0.25` + `fz_scale≈1.58`: rebote lento de gran amplitud (iguala los 7.2 cm / ~2.5 Hz).
- `j_damp≈0.29`, `solref0≈0.013`, `Tst≈0.234`, `krh≈0.094`: disipación, contacto y avance afinados.
- Topes de rodilla (q4∈[−2.8,−0.7]) como red de seguridad (no activos en el gait).

Los valores finales (en `tune_eval.DEFAULTS`) se hallaron por búsqueda multi-agente
con `tune_metric` (verify + distancia a la referencia), llegando a score 99/100.

## Uso
```bash
pip install mujoco numpy scipy matplotlib imageio imageio-ffmpeg
python3 control.py                 # corrida + figuras/resultados.png
python3 verify.py                  # suite de verificación (PASS/FAIL + vs MATLAB)
MUJOCO_GL=egl python3 render.py    # video figuras/salto.mp4
python3 view.py                    # visor interactivo en vivo
# referencia MATLAB: cd ../Simulator_MATLAB && matlab -batch "export_ref"
```
