# Notas para correr HOPPY en Linux (MATLAB)

Fork del proyecto original [RoboDesignLab/HOPPY-Project](https://github.com/RoboDesignLab/HOPPY-Project),
adaptado para correr el simulador en **Linux** con **MATLAB R2026a**.

## Cómo correr el simulador

```bash
cd Simulator_MATLAB
matlab            # abre la interfaz; dentro ejecutar:  MAIN
```

O sin pantalla (headless):

```bash
cd Simulator_MATLAB
matlab -batch "MAIN"
```

Genera la animación 3D del robot saltando y exporta `video.avi`.

**Requisito de toolbox:** Optimization Toolbox (`quadprog`), usada por la
saturación de motor (`p.isControlSaturate = 1` en `MAIN.m`).

## Cambios respecto al original (parches Linux)

En `Simulator_MATLAB/fcns/animateRobot.m`:

1. **Perfil de video.** El perfil `'MPEG-4'` de `VideoWriter` solo existe en
   Windows/Mac. En Linux se usa `'Motion JPEG AVI'` (salida `video.avi`). El
   cambio es portable (condicional `ispc || ismac`), así que sigue funcionando
   en Windows/Mac generando `video.mp4`.

2. **Tamaño de frame fijo.** `getframe` puede variar ±1 px entre cuadros (sobre
   todo sin pantalla), y `VideoWriter` exige tamaño constante. Se fuerza cada
   frame a 600×1400 por remuestreo con indexación (sin necesidad de Image
   Processing Toolbox).
