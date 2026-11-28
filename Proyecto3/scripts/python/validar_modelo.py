"""Compara un modelo identificado con capturas OA reservadas para validación."""

import argparse
import json
from pathlib import Path

import numpy as np

from identificar_modelo import AQUI, cargar_ensayos, escalon_promedio


def validar(datos, modelo):
    """Calcula predicciones y errores sin estimar de nuevo la dinámica."""
    # Verifica que el modelo corresponde a la forma exportada por el ajuste.
    k, tau, retardo, deriva = (float(modelo[clave]) for clave in ("K", "tau", "L", "deriva_rad_s2"))
    if not np.isfinite([k, tau, retardo, deriva]).all() or (
        k < 0 or tau <= 0 or retardo < 0 or deriva < 0 or modelo["tau_a"] != 0
    ):
        raise ValueError("Se requiere un modelo válido de primer orden con retardo")
    condiciones = modelo["condiciones"]
    velocidad = condiciones["w_objetivo_rpm"]
    ensayos, archivos = cargar_ensayos(datos, "validacion", velocidad)
    if not ensayos:
        raise ValueError(f"No hay ensayos de validación para {velocidad:g} rpm")

    # Detecta copias exactas cuando las capturas de ajuste están disponibles.
    origen = (datos / "identificacion").resolve()
    capturas_ajuste = []
    for nombre in condiciones["archivos"]:
        ruta = (datos / nombre).resolve()
        if not ruta.is_relative_to(origen):
            raise ValueError(f"Captura de ajuste fuera de identificación: {nombre}")
        if ruta.is_file():
            capturas_ajuste.append(ruta.read_bytes())

    resultados = []
    errores = []
    for archivo, (inicio, fin, velocidad_medida, u) in zip(archivos, ensayos):
        if not condiciones["u_min"] <= u <= condiciones["u_max"]:
            raise ValueError(f"{archivo}: nivel de frenado fuera del rango ajustado")
        if (datos / archivo).read_bytes() in capturas_ajuste:
            raise ValueError(f"{archivo}: la captura coincide con una utilizada en el ajuste")

        # Ancla la predicción con el tramo previo al freno y usa únicamente
        # intervalos posteriores completos para evaluar el error.
        centro = (inicio + fin) / 2
        previo = fin <= 0
        base = float(np.mean(velocidad_medida[previo] + deriva * centro[previo]))
        posterior = inicio >= 0
        medida = velocidad_medida[posterior]
        predicha = base - deriva * centro[posterior] - k * u * escalon_promedio(
            inicio[posterior], fin[posterior], tau, retardo
        )
        error = predicha - medida
        errores.append(error)
        resultados.append({
            "archivo": archivo, "u": u, "observaciones": int(error.size),
            "rmse_rad_s": float(np.sqrt(np.mean(error**2))),
            "mae_rad_s": float(np.mean(np.abs(error))),
            "t_s": centro[posterior].tolist(),
            "w_medida_rad_s": medida.tolist(),
            "w_predicha_rad_s": predicha.tolist(),
        })

    # El RMSE conjunto pondera cada observación del sensor por igual.
    agrupados = np.concatenate(errores)
    return {
        "w_objetivo_rpm": velocidad,
        "rmse_conjunto_rad_s": float(np.sqrt(np.mean(agrupados**2))),
        "ensayos": resultados,
    }


def main():
    # Guarda las series y métricas en un informe aparte del modelo ajustado.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--modelo", type=Path, default=AQUI / "modelo_planta.json")
    parser.add_argument("--datos", type=Path, default=AQUI.parents[1] / "datos")
    parser.add_argument("--salida", type=Path, required=True)
    args = parser.parse_args()
    try:
        modelo = json.loads(args.modelo.read_text(encoding="utf-8"))
        informe = validar(args.datos, modelo)
        informe["modelo"] = str(args.modelo)
        contenido = json.dumps(informe, indent=2, allow_nan=False)
        with args.salida.open("x", encoding="utf-8") as salida:
            salida.write(contenido + "\n")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(1, f"Error: {exc}\n")
    print(f"RMSE conjunto: {informe['rmse_conjunto_rad_s']:.4f} rad/s")
    print(f"Informe guardado en {args.salida}")


if __name__ == "__main__":
    main()
