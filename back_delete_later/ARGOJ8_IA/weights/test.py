#!/usr/bin/env python3
# check_weights.py
# ------------------------------------------------------------
# Escanea ficheros Keras/TensorFlow *.weights.h5 en busca de
# valores NaN o ±Inf.  Uso:
#
#   python check_weights.py /ruta/a/pesos               # solo informe
#   python check_weights.py /ruta/a/pesos --fix         # repara y copia
#   python check_weights.py /ruta/a/pesos -r            # recursivo
#
# Requiere: h5py, numpy
# ------------------------------------------------------------
import argparse, pathlib, sys, time, shutil, h5py, numpy as np

def scan_file(path: pathlib.Path):
    """Devuelve (total_tensors, corrupt_tensors, corrupt_vals)."""
    tot_t, bad_t, bad_vals = 0, 0, 0
    with h5py.File(path, "r") as f:
        def _visit(name, obj):
            nonlocal tot_t, bad_t, bad_vals
            if isinstance(obj, h5py.Dataset):
                tot_t += 1
                arr = obj[()]          # carga en memoria
                n_bad = np.isnan(arr).sum() + np.isinf(arr).sum()
                if n_bad:
                    bad_t    += 1
                    bad_vals += int(n_bad)
        f.visititems(_visit)
    return tot_t, bad_t, bad_vals

def fix_file(path: pathlib.Path):
    """Genera *_fixed.h5 con NaN/Inf → 0. Devuelve conteo corregido."""
    out = path.with_name(path.stem + "_fixed" + path.suffix)
    shutil.copy2(path, out)
    corrected = 0
    with h5py.File(out, "r+") as f:
        def _visit(name, obj):
            nonlocal corrected
            if isinstance(obj, h5py.Dataset):
                arr = obj[()]
                mask = np.isnan(arr) | np.isinf(arr)
                if mask.any():
                    corrected += int(mask.sum())
                    arr[mask] = 0.0
                    obj[...] = arr
        f.visititems(_visit)
    return out, corrected

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", help="Directorio con *.weights.h5")
    ap.add_argument("-r", "--recursive", action="store_true",
                    help="Búsqueda recursiva en subcarpetas")
    ap.add_argument("--fix", action="store_true",
                    help="Crear copia *_fixed.h5 reemplazando NaN/Inf por 0")
    args = ap.parse_args()

    root = pathlib.Path(args.folder).expanduser()
    pattern = "**/*.weights.h5" if args.recursive else "*.weights.h5"
    files = sorted(root.glob(pattern))
    if not files:
        sys.exit("❌  No se encontraron archivos *.weights.h5")

    print(f"Analizando {len(files)} fichero(s) en '{root}' …\n")
    total_bad = 0
    for p in files:
        tot, bad_t, bad_vals = scan_file(p)
        flag = "✅" if bad_vals == 0 else "⚠️"
        print(f"{flag} {p.name:40}  tensores:{bad_t:4}/{tot:<4}  NaN/Inf:{bad_vals}")
        total_bad += bad_vals

        if bad_vals and args.fix:
            out, corr = fix_file(p)
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            print(f"   ↳ {corr} valores reparados → {out.name}  ({ts})")

    print("\n──────── resumen ────────")
    if total_bad == 0:
        print("🎉  Todos los ficheros están limpios.")
    else:
        print(f"💥  Total valores defectuosos encontrados: {total_bad}")
        if args.fix:
            print("✅  Versiones *fixed* creadas para los archivos afectados.")

if __name__ == "__main__":
    main()
