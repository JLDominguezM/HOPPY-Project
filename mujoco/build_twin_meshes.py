"""Extrae las mallas CAD al frame REAL del modelo gemelo (medido del STEP).

A diferencia del overlay cosmetico viejo, aqui la transformacion CAD->modelo es la
medida (pivote, escala 1:1), asi que las mallas caen en su lugar SOLAS, sin offsets
de truco: el gantry queda fijo con la base en el piso y el pivote a HB; el housing
queda en link2 con la cadera en su sitio real.

IMPORTANTE: el GLB de glTF es Y-UP (Y = vertical), NO Z-up. Con Y como vertical la base
del gantry cae en el piso y la altura del pivote da 0.250 m (coincide con la foto real);
con Z-up todo salia rotado 90 grados. Transformacion CAD(m, Y-up) -> modelo(Z-up):
  model_x = -(x_cad - px)        # boom hacia el hoppy = +X (hoppy en low-X_cad)
  model_y =  (z_cad - pz)        # tangencial = Z_cad (lateral)
  model_z =  (y_cad - py) + HB   # ARRIBA = Y_cad ; pivote a z=HB, base del gantry a z=0
Salida: meshes/twin_gantry.obj (mundo, fijo)  y  meshes/twin_housing.obj (link2-local).
"""
import trimesh, numpy as np, os

# pivote en CAD (Y-up): X=centro del gantry, Y=altura del boom, Z=profundidad del boom
PIVOT_CAD = np.array([1.559, 1.312, 1.349])
HB = 0.250                                     # altura del pivote sobre el piso (up=Y, medido)

S = trimesh.load("/tmp/hoppy.glb")
parts = []
for n in S.graph.nodes_geometry:
    T, gn = S.graph[n]; g = S.geometry[gn].copy().apply_transform(T)
    parts.append((g, g.centroid.copy(), g.extents.copy()))
C = np.array([p[1] for p in parts]); E = np.array([p[2] for p in parts])


def to_model(V, add_hb=True):
    """CAD (Y-up) -> modelo (Z-up): boom->+X, vertical=Y_cad->+Z, lateral=Z_cad->+Y."""
    out = np.empty_like(V)
    out[:, 0] = -(V[:, 0] - PIVOT_CAD[0])               # boom (+X hacia el hoppy)
    out[:, 1] = (V[:, 2] - PIVOT_CAD[2])                # tangencial = Z_cad
    out[:, 2] = (V[:, 1] - PIVOT_CAD[1]) + (HB if add_hb else 0.0)  # ARRIBA = Y_cad
    return out


LB, DB, HIP_DZ = 0.687, 0.187, -0.036                   # cadera real (link2-local) = twin.py
KNEE_OFF = (-0.0018, 0.0586, -0.0583)                   # rodilla (union tubo-placas) = twin.py
SPRING_IDX = (181, 182)                                 # las 2 bobinas (van como tendon, no malla)


def export(idxs, fname, add_hb=False, offset=(0, 0, 0)):
    full = trimesh.util.concatenate([parts[i][0] for i in idxs])
    full.vertices = to_model(full.vertices, add_hb=add_hb) - np.array(offset)
    os.makedirs("meshes", exist_ok=True)
    full.export(f"meshes/{fname}")
    b = full.bounds
    print(f"  {fname}: {len(idxs)} piezas  x[{b[0][0]:.3f},{b[1][0]:.3f}] "
          f"y[{b[0][1]:.3f},{b[1][1]:.3f}] z[{b[0][2]:.3f},{b[1][2]:.3f}]")


# GANTRY (mundo, fijo): cluster high-X estructura, sin el tubo del boom (extents_X<0.30)
gi = [i for i in range(len(parts)) if C[i][0] > 1.40 and E[i][0] < 0.30]
export(gi, "twin_gantry.obj", add_hb=True)

# El jumper (hoppy) se reparte en 3 cuerpos por su altura (Y_cad = vertical), con la
# cadera a Y=1.312 y la rodilla LH abajo (Y=1.312-0.096=1.216). El boom pasa a Y=1.312.
#   HOUSING (link2): la caja de motores, arriba de la rodilla -> Y_cad > 1.25
#   THIGH  (link3): seccion superior de la pierna (cadera->rodilla)  1.20 < Y <= 1.25
#   SHANK  (link4): pantorrilla + pie (rodilla->pie)                 Y <= 1.20
# La pierna del CAD cuelga RECTA (pose theta3=theta4=0), asi que cada malla va en su
# frame de eslabon restando la posicion de la junta; articulan con los joints del modelo.
jx = [i for i in range(len(parts)) if C[i][0] < 1.2 and E[i][0] < 0.30]
hi = [i for i in jx if C[i][1] > 1.25]
ti = [i for i in jx if 1.20 < C[i][1] <= 1.25 and i not in SPRING_IDX]   # sin las bobinas
si = [i for i in jx if C[i][1] <= 1.20]
_knee = (LB + KNEE_OFF[0], DB + KNEE_OFF[1], HIP_DZ + KNEE_OFF[2])   # rodilla en model frame
export(hi, "twin_housing.obj")
export(ti, "twin_thigh.obj", offset=(LB, DB, HIP_DZ))   # link3-local (origen en la cadera real)
export(si, "twin_shank.obj", offset=_knee)              # link4-local (origen en la rodilla real)
print("listo: twin_gantry/housing/thigh/shank.obj")
