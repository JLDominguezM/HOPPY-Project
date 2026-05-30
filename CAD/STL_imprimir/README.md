# Piezas a imprimir — rediseño de clase

⚠️ **Importante:** este proyecto usa el **rediseño del equipo** (BOM de clase
`BOM - HOPPY.xlsx`), **no** las piezas goBilda originales del repo. Las piezas a
imprimir son **IMP-1 … IMP-30 + CASE 1/2**, no las piezas goBilda.

> (Antes esta carpeta tenía STLs de las piezas goBilda originales; se quitaron porque
> el rediseño parte de geometría propia, fabricada por impresión y tubos de PVC.)

## Dónde está cada cosa
- **Qué imprimir + cantidades + material (PLA/ABS) + gramos + tiempo:** checklist **#18**.
- **Inventario completo de materiales (comprar/tener):** issue **#5**.
- **CAD fuente del rediseño:** paquete `DISEÑO` (SolidWorks): `IMP-*.SLDPRT`, ensambles
  `HOPPY-E0/E1/E2`, y `HOPPY-E0.STEP`. De ahí se **exportan los STL** (Archivo →
  Guardar como → STL) para imprimir.

## Resumen de impresión (del BOM)
- **Total:** 33 piezas → **49 copias** · **~4.27 kg** de filamento · **~240 h** de impresión.
- Mayoría **PLA**; en **ABS**: IMP-16, IMP-17, IMP-30, IMP-20.
- Piezas largas (repartir entre impresoras): IMP-3A/3B y IMP-6A/6B (~22 h c/u),
  IMP-27 (~18 h), IMP-28 (~15.5 h).

## Recomendaciones de impresión en PLA
Como el robot recibe impactos al aterrizar, para el PLA:
- Boquilla **210–220 °C**, perímetros lentos (**30–40 mm/s**), ventilador **40–70 %**.
- **5 paredes**, relleno **50–70 %**, capas top/bottom 6.
- **Orientar acostado** para que el impacto quede a lo largo de las capas (no delamine).
- Insertos térmicos M2/M3/M4 → calentar con cautín e insertar; no imprimir roscas.
- Empezar pruebas con fuerza de salto baja; tener repuestos de las piezas de la pata.
