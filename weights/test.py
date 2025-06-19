#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arregla_pesos.py
────────────────
Escanea un fichero *.weights.h5 de Keras/TensorFlow, detecta NaN/Inf en
cualquier tensor y los reemplaza por 0.0 (o por un valor pequeño de
ruido gaussiano si prefieres).  Deja una copia intacta del original y
crea un nuevo archivo con sufijo *_fixed.weights.h5.

✏️  USO RÁPIDO
--------------------------------------------------
1. Edita PESO_ORIGEN = "ruta/al/archivo.weights.h5"
2. Ejecuta:
       python3 arregla_pesos.py
   (o ./arregla_pesos.py si le das permisos)

   Se generará  <nombre>_fixed.weights.h5  en el mismo directorio.
"""

import h5py
import numpy as np
import pathlib
import shutil
import sys
import datetime as dt

# ─────────────────────────────
#  EDITA AQUÍ TU RUTA DE PESOS
# ─────────────────────────────
PESO_ORIGEN = "/home/rhobtor/PHD/ARGOS_J8/ARGOSJ8_IA_/weights/policy_meta_ep.weights.h5"
# ─────────────────────────────

# ——————————————————————————————————————————————
#  CONFIGURACIÓN
# ——————————————————————————————————————————————
REEMPLAZO   = "zeros"       # "zeros" | "gauss"
SIGMA_RUIDO = 1e-6          # sólo si REEMPLAZO == "gauss"

# ——————————————————————————————————————————————
#  COMIENZA EL SCRIPT
# ——————————————————————————————————————————————
src = pathlib.Path(PESO_ORIGEN).expanduser()

if not src.exists():
    sys.exit(f"❌  El archivo no existe:\n    {src}")

dst = src.with_name(src.stem + "_fixed" + src.suffix)
backup = src.with_name(src.stem + "_backup_" +
                       dt.datetime.now().strftime("%Y%m%d_%H%M%S") +
                       src.suffix)

# -- safety first: copia de respaldo por si acaso
shutil.copy(src, backup)
print(f"🗄️  Copia de respaldo creada:\n    {backup}")

# copia para editar (evitamos tocar backup/original)
shutil.copy(src, dst)

n_tot, n_bad = 0, 0

def reparar_dataset(dset):
    """Reemplaza cualquier valor no-finito del dataset in-place."""
    global n_tot, n_bad
    arr = dset[...]
    mask = ~np.isfinite(arr)
    n_bad += int(mask.sum())
    n_tot += arr.size
    if mask.any():
        if REEMPLAZO == "zeros":
            arr[mask] = 0.0
        else:  # ruido gaussiano
            arr[mask] = np.random.normal(0.0, SIGMA_RUIDO, size=mask.sum())
        dset[...] = arr        # sobrescribir in-place

with h5py.File(dst, "r+") as f:
    f.visititems(lambda n, obj: reparar_dataset(obj)
                 if isinstance(obj, h5py.Dataset) else None)

print("\n──────── RESULTADO ────────")
print(f"Tensors totales analizados : {n_tot:,}")
print(f"Valores NaN/Inf reparados  : {n_bad:,}")
print(f"✅  Archivo corregido guardado en:\n    {dst}")
print("   (el original sigue intacto)")

# Nota final
if n_bad == 0:
    print("ℹ️  No se encontraron valores no finitos – el archivo ya estaba limpio.")
else:
    print("👍  Ahora puedes usar el fichero *_fixed.weights.h5 en tu nodo de inferencia.")
