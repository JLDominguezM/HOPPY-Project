# STLs para imprimir — Estructura de HOPPY

Esta carpeta contiene **solo las piezas estructurales fabricables** del robot, exportadas
del ensamble original (goBilda) a STL. El resto de componentes del BOM **no se imprimen**
(baleros, bandas, poleas, resortes, ejes, tornillería, tubos, motores, micro y drivers):
ver la lista "No imprimir" más abajo y el [BOM completo](../../HOPPY_BoM.xlsx).

> ⚠️ **Aviso de ingeniería.** HOPPY fue diseñado en aluminio a propósito: *"we intentionally
> avoid plastic 3D printed parts for their short working life under impact loads"*. El robot
> salta y golpea el suelo, así que las piezas impresas pueden agrietarse. Para un prototipo
> de curso es viable. El material ideal sería tenaz (PETG/ABS/nylon), pero **en este proyecto
> el requisito es imprimir en PLA** → ver la sección **"Impresión en PLA"** para los ajustes
> que maximizan su resistencia y las precauciones de uso.

## Especificaciones de impresión recomendadas

| Parámetro | Valor | Por qué |
|---|---|---|
| **Material** | **PETG/ABS/Nylon** ideal · **PLA** en este proyecto (ver sección PLA) | Los primeros son tenaces; el PLA es frágil pero usable con los ajustes de abajo. |
| Boquilla | 0.4 mm | Estándar |
| Altura de capa | 0.20 mm | Balance resistencia/tiempo |
| Perímetros (paredes) | **4** (≥1.6 mm) | La resistencia real viene de las paredes, no del relleno |
| Relleno | **40–60 %** (giroide) en piezas de carga; 100 % en piezas chicas | Rigidez sin peso extra |
| Capas top/bottom | 5 | Cierre sólido |
| Soportes | Solo en voladizos | La mayoría son planas |
| Adhesión | Brim si despega | Piezas con poca base |

### Orientación (¡importante!)
Imprime las placas, brackets y mounts **acostados (planos sobre la cama)**, de modo que la
carga de impacto quede **a lo largo de las capas, no perpendicular** a ellas (las capas se
deslaminan si el esfuerzo las separa). Es el factor #1 de durabilidad.

### Tolerancias
Los agujeros suelen salir ~0.2–0.4 mm chicos. Para los **M4**, pasa una broca de 4 mm o
ajusta el escalado de agujeros en el slicer. Haz una **pieza de prueba** (un bracket chico)
antes de imprimir todo el lote para validar el ajuste.

## Impresión en PLA (requisito de este proyecto)

El PLA es **rígido pero frágil**: falla por impacto y por delaminación entre capas. No lo
podemos cambiar, así que la estrategia es **maximizar la unión entre capas y el grosor de
pared**, e imprimir con la orientación correcta. Ajustes recomendados:

| Parámetro | Valor PLA | Por qué |
|---|---|---|
| **Temp. boquilla** | **210–220 °C** (extremo alto del PLA) | Mejor fusión entre capas = menos delaminación al impacto |
| Temp. cama | 60 °C | Adhesión |
| **Velocidad de perímetros** | **lenta, 30–40 mm/s** | Capas mejor soldadas (la velocidad mata la resistencia) |
| **Ventilador** | **40–70 %** (no 100 %) | Demasiado enfriamiento debilita la unión entre capas; baja el fan en piezas de carga |
| **Perímetros (paredes)** | **5** (≈2.0 mm) | El PLA aguanta por pared, no por relleno |
| **Relleno** | **50–70 %** (giroide/cúbico) | Más material continuo que absorbe impacto |
| Altura de capa | 0.16–0.20 mm | Capa más fina = más uniones y mejor detalle |
| Capas top/bottom | 6 | Cierre robusto |
| **Tipo de PLA** | **PLA+ / Tough PLA** si tienen | Bastante más tenaz que el PLA estándar |

**Orientación (lo más importante en PLA):** imprime cada pieza **acostada y plana**, para que
el golpe del aterrizaje quede **a lo largo de las capas, no perpendicular** (perpendicular =
delamina y se parte). Vale la pena reorientar pieza por pieza.

**Precauciones de uso con PLA:**
- El PLA **se reblandece con el calor** (~55–60 °C). No dejes el robot al sol ni cerca de
  los motores calientes mucho tiempo; revisa que no se deformen los mounts.
- Empieza las pruebas con **fuerza de salto baja** y sube de a poco; espera **reimprimir
  piezas agrietadas** (sobre todo pillow blocks, mounts de la pata y el pie).
- Ten **repuestos impresos** de las piezas más castigadas (pata, pie, mounts inferiores).
- Aplica **thread locker / aprieta seguido** los tornillos: la vibración + PLA aflojan.

> Si más adelante consiguen PETG aunque sea para las **piezas críticas de la pata inferior**
> (las que reciben el impacto directo), valdría mucho la pena combinarlo con el PLA del resto.

## Piezas a imprimir (25 únicas — exactamente las del BOM)

| STL | Descripción | Cant. | Notas |
|---|---|---:|---|
| `1140-0006-0144` | Baseplate 144 mm (base del gantry) | 1 | Si tu cama < 144 mm, pártela o usa lámina |
| `1123-0048-0144` | Pattern plate 1×5 (144 mm) | 4 | Estructura principal de los links |
| `1121-0003-0096` | U-channel 3-hole 96 mm | 1 | |
| `1120-0001-0048` | U-channel 1-hole 48 mm | 1 | |
| `1108-0001-0001` | Flat bracket 1-1 | 4 | |
| `1111-0004-0001` | Angle bracket 4-1 | 2 | |
| `1102-0005-0040` | Flat beam 5-hole 40 mm | 2 | |
| `1201-0043-0002` | Quad block mount | 2 | Monta los motores |
| `1200-0032-0001` | Pattern mount 1s-2post 32 mm | 1 | |
| `1205-0001-0003` | Dual block mount | 1 | |
| `1401-0043-0032` | Clamping mount 2-side, **bore 32 mm** | 3 | Se cierra sobre el tubo de 32 mm → necesita ranura + tornillo |
| `1400-0032-0032` | Clamping mount 1-side, **bore 32 mm** | 2 | **Opcional** (BOM: solo si se usa encoder en joint 2) |
| `1400-0032-0014` | Clamping mount 1-side, **bore 14 mm** | 2 | Para el tubo de la pata (14 mm) |
| `1402-0043-0014` | Clamping mount 2s-1post, bore 14 mm | 1 | |
| `1603-0032-0032` | Pillow block (face) | 2 | **Lleva balero comprado** (1601) insertado |
| `1604-0043-0032` | Pillow block 2s-2post | 3 | **Lleva balero comprado** (1601) insertado |
| `1505-0032-0080` | Espaciador 32 mm OD, 80 mm | 1 | |
| `1506-0032-0004` | Espaciador 32 mm, 4 mm | 5 | Opcional: sustituir por washer metálico |
| `1506-0032-0006` | Espaciador 32 mm, 6 mm | 2 | Opcional |
| `1502-0005-0100` | Espaciador 4 mm-5OD-10 | 4 | Opcional |
| `1502-0006-0020` | Espaciador 4 mm-6OD-2 | 4 | Opcional |
| `1502-0006-0030` | Espaciador 4 mm-6OD-3 | 4 | Opcional |
| `1502-0006-0100` | Espaciador 4 mm-6OD-10 | 4 | Opcional |
| `1500-0010-0032` | Espaciador plástico | 1 | Opcional |
| `1501-0006-0430` | Standoff M4 (eje tensor de banda) | 1 | |

> **Espaciadores (1500/1502/1505/1506):** son imprimibles, pero es **más confiable y rápido
> usar washers/standoffs metálicos** de ferretería del largo equivalente.

## NO imprimir — comprar (ver issue #17 e [BOM](../../HOPPY_BoM.xlsx))

- **Baleros:** 1601 / 1600 (insertos de las pillow blocks), 1607 (idler), 1611 (bridado REX 8 mm)
- **Banda dentada** HTD-5M 59T 295 mm (3412-0009-0295)
- **Poleas** 16T (3414) y 24T (3411) — la precisión del dentado no se imprime bien
- **Sonic/clamp hubs** 1309 (impresos patinan en el eje)
- **Eje REX 8 mm** (2102) — usar barra de acero
- **Resortes de extensión** de rodilla (2915) ×2 — críticos
- **Tubos:** goTube 32 mm OD del gantry (4103, incluido el de **912 mm**) y tubo de la pata
  14 mm OD (4100) → **usar tubo de aluminio real cortado a medida** (no imprimir tubos largos)
- **Tornillería M4** y placas roscadas (2800 / 2802 / 2803) + tornillos McMaster (90128A, 91390A, 98035A, 90591A)
- **Contrapeso ~2.3 kg** (disco de gimnasio)
- **Tapón de pata** 14 mm (hule; opcional imprimir en TPU)
- **Motores** goBilda 5202, **drivers** VNH5019, **micro** TI F28379D → ya se tienen / se compran

---
Fuente original: [RoboDesignLab/HOPPY-Project](https://github.com/RoboDesignLab/HOPPY-Project).
STLs exportados del ensamble; clasificación y specs adaptadas para fabricación.
