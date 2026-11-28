"""Ajusta un modelo local del frenado con ensayos OA de identificación."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from leer_ensayo import leer_ensayo

AQUI = Path(__file__).resolve().parent


def preparar_ensayo(archivo, nivel):
    """Extrae el escalón y los intervalos de observación del sensor."""
    muestras, _ = leer_ensayo(archivo)
    if muestras[0]["modo"] != "OA":
        raise ValueError(f"{archivo}: se esperaba un ensayo OA")

    # Delimita el escalón con la orden registrada y el motor apagado.
    escalon = next(
        (i for i, fila in enumerate(muestras) if fila["pwm"] == 0 and fila["u"] > 0),
        None,
    )
    if escalon is None or escalon == 0 or muestras[escalon - 1]["u"] != 0:
        raise ValueError(f"{archivo}: falta la transición de freno libre a aplicado")
    origen = muestras[escalon]["t"]
    u = muestras[escalon]["u"]
    if not np.isclose(u, nivel, rtol=0, atol=1e-6):
        raise ValueError(f"{archivo}: el nivel registrado difiere de la plantilla")

    # El flanco nuevo cierra un intervalo. Excluye intervalos que cruzan
    # el corte del motor y el tramo en que el disco ya se detuvo.
    datos = []
    anterior = None
    for fila in muestras:
        if fila["pwm"] != 0:
            if fila["t"] >= origen:
                break
            anterior = None
            continue
        if fila["t"] >= origen and (fila["u"] != u or fila["w"] <= 0):
            break
        if not fila["nueva"]:
            continue
        instante = fila["t"] - origen
        if anterior is not None and fila["w"] > 0:
            datos.append((anterior, instante, fila["w"]))
        anterior = instante

    if not datos:
        raise ValueError(f"{archivo}: no hay observaciones útiles del sensor")
    inicio, fin, velocidad = np.asarray(datos).T
    if np.count_nonzero(fin <= 0) < 2 or np.count_nonzero(inicio >= 0) < 4:
        raise ValueError(f"{archivo}: se requieren 2 periodos antes y 4 después del escalón")
    return inicio, fin, velocidad, u


def escalon_promedio(inicio, fin, tau, retardo):
    """Promedia la respuesta unitaria en cada intervalo entre flancos."""
    a = np.maximum(inicio - retardo, 0)
    b = np.maximum(fin - retardo, 0)

    # Integra 1 - exp(-t/tau) desde el fin del retardo.
    integral = (b - a) + tau * np.exp(-a / tau) * np.expm1(-(b - a) / tau)
    return integral / (fin - inicio)


def ajustar(ensayos):
    """Estima K, tau y L comunes y una velocidad inicial por ejecución."""
    # Compensa la desaceleración libre con una pendiente local compartida.
    duracion = min(fin[-1] for _, fin, _, _ in ensayos)
    resolucion = float(np.median(np.concatenate([b - a for a, b, _, _ in ensayos])))
    medias = [(a + b) / 2 for a, b, _, _ in ensayos]

    def residuos(parametros):
        k, tau, retardo, pendiente, *bases = parametros
        return np.concatenate([
            base - pendiente * tiempo - k * u * escalon_promedio(a, b, tau, retardo) - w
            for (a, b, w, u), tiempo, base in zip(ensayos, medias, bases)
        ])

    # Acota el retardo a la ventana observada y prueba varias inicializaciones.
    retardos = (0, duracion / 5, duracion / 2)
    resultados = [
        least_squares(
            residuos,
            [1, duracion / 3, l, 0.1, *(w[0] for _, _, w, _ in ensayos)],
            bounds=([0, 1e-6, 0, 0, *([0] * len(ensayos))],
                    [np.inf, np.inf, duracion, np.inf, *([np.inf] * len(ensayos))]),
            x_scale="jac",
        )
        for l in retardos
    ]
    convergentes = [resultado for resultado in resultados if resultado.success]
    if not convergentes:
        raise ValueError("El ajuste no convergió con los datos disponibles")
    mejor = min(convergentes, key=lambda resultado: resultado.cost)
    k, tau, retardo, pendiente, *bases = map(float, mejor.x)
    if np.linalg.matrix_rank(mejor.jac) < len(mejor.x):
        raise ValueError("Los ensayos no permiten separar ganancia, dinámica, retardo y deriva")

    # Las pérdidas por nivel sirven para revisar la zona muerta y la saturación.
    perdidas = {}
    for (a, b, w, u), tiempo, base in zip(ensayos, medias, bases):
        forma = escalon_promedio(a, b, tau, retardo)
        energia = np.dot(forma, forma)
        if energia == 0:
            raise ValueError("El retardo estimado deja un ensayo sin respuesta observable")
        perdida = float(np.dot(forma, base - pendiente * tiempo - w) / energia)
        perdidas.setdefault(u, []).append(perdida)

    return {
        "K": k, "tau": tau, "tau_a": 0.0, "L": retardo,
        "polos_rad_s": [-1 / tau], "ceros_rad_s": [],
        "deriva_rad_s2": pendiente,
        "rmse_ajuste_rad_s": float(np.sqrt(np.mean(mejor.fun**2))),
        "intervalo_mediano_sensor_s": resolucion,
        "niveles": [
            {"u": u, "perdida_estimada_rad_s": float(np.mean(perdidas[u]))}
            for u in sorted(perdidas)
        ],
    }


def identificar(datos, velocidad):
    # Selecciona un punto de operación y limita las fuentes a identificación.
    ensayos = []
    archivos = []
    raiz = (datos / "identificacion").resolve()
    with (datos / "condiciones_ensayos.csv").open(encoding="utf-8-sig", newline="") as entrada:
        for fila in csv.DictReader(entrada):
            relativa = Path(fila["archivo"])
            if not relativa.parts or relativa.parts[0] != "identificacion":
                continue
            if float(fila["w_objetivo_rpm"]) != velocidad:
                continue
            nivel = float(fila["u"])
            if nivel == 0:
                continue
            archivo = (datos / relativa).resolve()
            if not archivo.is_relative_to(raiz) or archivo in archivos:
                raise ValueError(f"Ruta fuera de identificación o duplicada: {relativa}")
            ensayos.append(preparar_ensayo(archivo, nivel))
            archivos.append(archivo)
    if len({ensayo[3] for ensayo in ensayos}) < 3:
        raise ValueError(f"Se requieren tres niveles de identificación para {velocidad:g} rpm")

    modelo = ajustar(ensayos)
    modelo["condiciones"] = {
        "w_objetivo_rpm": velocidad,
        "u_min": min(ensayo[3] for ensayo in ensayos),
        "u_max": max(ensayo[3] for ensayo in ensayos),
        "archivos": [archivo.relative_to(datos.resolve()).as_posix() for archivo in archivos],
        "motor_durante_frenado": "apagado",
        "validado": False,
    }
    return modelo


def main():
    # La salida común permite cargar el resultado desde disenar_controlador.py.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--velocidad", type=float, required=True, help="consigna inicial en rpm")
    parser.add_argument("--datos", type=Path, default=AQUI.parents[1] / "datos")
    parser.add_argument("--salida", type=Path, default=AQUI / "modelo_planta.json")
    args = parser.parse_args()
    try:
        modelo = identificar(args.datos, args.velocidad)
        contenido = json.dumps(modelo, indent=2, allow_nan=False)
        with args.salida.open("x", encoding="utf-8") as salida:
            salida.write(contenido + "\n")
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Error: {exc}\n")
    print(f"Modelo provisional guardado en {args.salida}")


if __name__ == "__main__":
    main()
