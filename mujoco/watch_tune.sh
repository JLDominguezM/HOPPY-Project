#!/bin/bash
# Monitoreo del tuner del HOPPY URDF (urdf_tune.py).  Uso: bash watch_tune.sh
cd ~/ImplementacionRobotica/HOPPY-Project/mujoco || exit 1
echo "=== HOPPY URDF Tune — progreso ==="
tail -30 urdf_tune.log 2>/dev/null || echo "(sin log)"
echo
echo "=== Mejor resultado hasta ahora ==="
python3 -c "
import json
try:
    b = json.load(open('urdf_best.json'))
    print('z=%.1f cm  saltos=%d  score=%.1f  (apex_std=%.1f mm, grf_max=%.0f N, %%apoyo=%.0f)'
          % (b.get('z_cm',0), b.get('n_saltos',0), b.get('score',0),
             b.get('apex_std_mm',0), b.get('grf_max',0), b.get('frac_stance',0)*100))
    print('params:', {k: round(v,3) for k,v in b.get('params',{}).items()})
except Exception:
    print('Sin resultados aun')
" 2>/dev/null
echo
echo "=== tuner vivo? ==="
P=$(cat urdf_tune.pid 2>/dev/null)
if [ -n "$P" ] && ps -p "$P" >/dev/null 2>&1; then echo "PID $P CORRIENDO"; else echo "NO corriendo (termino o aun no arranca)"; fi
