"""Extrae el GANTRY real del STEP y lo deja FIJO, alineado al pivote del modelo.

El gantry es la estructura fija (base octagonal -> torre -> balero del pivote
arriba) en el extremo high-X del ensamble. El boom pasa por su balero. Aqui se
separa del boom, se alinea con el pivote en (0,0,HB) y la base en el piso (z=0),
y se exporta a meshes/vis_gantry.obj (en coords del mundo, geometria FIJA).
"""
import trimesh, numpy as np, os
from tune_eval import HB

S = trimesh.load("/tmp/hoppy.glb")
parts = []
for n in S.graph.nodes_geometry:
    T, gn = S.graph[n]; g = S.geometry[gn].copy().apply_transform(T)
    parts.append((g, g.centroid.copy(), g.extents.copy()))
C = np.array([p[1] for p in parts])

# boom = piezas largas en X (el tubo). Su linea central da Y,Z del pivote.
boom = [p for p in parts if p[2][0] > 0.30]
boomYZ = np.mean([p[1][1:] for p in boom], axis=0)
print(f"boom: {len(boom)} piezas, linea central Y,Z = {np.round(boomYZ,3)}")

# gantry = cluster high-X (la estructura fija), excluyendo el tubo del boom
gi = [i for i, p in enumerate(parts) if p[1][0] > 1.40 and p[2][0] < 0.30]
gmesh = trimesh.util.concatenate([parts[i][0] for i in gi])
print(f"gantry: {len(gi)} piezas, bbox(m)={np.round(gmesh.extents,3)}")

# pivote = (X centro del gantry, Y,Z del boom) = donde el boom cruza el gantry
pivotX = float(np.median([parts[i][1][0] for i in gi]))
pivot = np.array([pivotX, boomYZ[0], boomYZ[1]])
print(f"pivote (CAD) = {np.round(pivot,3)}")

# alinear: pivote -> origen ; escala UNIFORME para que la base llegue al piso
# (proporcional, sin estirar) ; luego pivote a (0,0,HB)
V = gmesh.vertices - pivot
zmin = V[:, 2].min()                       # base relativa al pivote (negativa)
s = HB / (-zmin) if zmin < -1e-3 else 1.0   # escala uniforme: base->piso, pivote->HB
V = V * s
V = V + np.array([0, 0, HB])               # pivote a z=HB, base a z=0
gmesh.vertices = V
print(f"gantry alineado (escala uniforme {s:.2f}): bbox={np.round(gmesh.extents,3)} z=[{gmesh.bounds[0][2]:.3f},{gmesh.bounds[1][2]:.3f}]")
os.makedirs("meshes", exist_ok=True)
gmesh.export("meshes/vis_gantry.obj")
print("meshes/vis_gantry.obj guardado (FIJO)")
