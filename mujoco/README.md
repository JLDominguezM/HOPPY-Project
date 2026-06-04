# Simulación de HOPPY en MuJoCo

Simulación del robot saltarín HOPPY en MuJoCo, siguiendo la rúbrica del curso
(Robots Humanoides). Replica la física del simulador de MATLAB: gantry pasivo +
pierna activa, control híbrido a 1 kHz, contacto duro y dinámica de actuadores
por voltaje.

## Archivos
- `hoppy.xml` — modelo MJCF (4 GDL, armature, resorte de rodilla, contrapeso, contacto duro).
- `build_model.py` — generador parametrico del modelo + tuner de balance del gantry.
- `control.py` — controlador híbrido (en construcción): FSM aéreo/apoyo, PD cartesiano, Bézier.
- `figuras/` — gráficas y video de resultados.

## Mapa a la rúbrica (100 pts)
| Fase | Contenido | Pts | Estado |
|---|---|---|---|
| 1 | Cinemática + armature (N²·Ir) + resorte + contrapeso (XML) | 15 | ✅ modelo listo |
| 2 | Contacto duro (`solref`/`solimp`), sin rebote | 10 | ✅ validado |
| 3 | Actuadores por voltaje + back-EMF (Ec.18), límites 12V/30A | 15 | pendiente |
| 4 | Control híbrido: FSM 1kHz, `Jc^T`, PD aéreo, Bézier stance, blending 10ms | 45 | pendiente |
| 5 | Velocidad por derivada filtrada (λ≈10) + gráficas | 15 | pendiente |

## Parámetros (de get_params.m y la guía técnica)
- Reducciones: NH=26.9, NK=28.8; inercia rotor Ir=7e-6 → armature=N²·Ir.
- Motor: Rw=1.3 Ω, kT=0.0135, kv=0.0186; límites Vmax=12 V, Imax=30 A.
- Largos: Rboom=0.556, muslo=0.096, pantorrilla=0.1545 m.

## Uso
```bash
pip install mujoco numpy scipy matplotlib
python3 build_model.py    # tuner de balance
python3 control.py        # correr la simulación (cuando esté listo)
```
