"""Lectura y validación de ensayos en el formato de Registro.h."""

import csv
import math
from pathlib import Path

ENCABEZADO = "t_ms,modo,ref,w,e,u,pwm,nueva".split(",")


def leer_ensayo(archivo):
    """Devuelve una lista de muestras tipadas y los comentarios del CSV."""
    muestras = []
    eventos = []
    encabezado_recibido = False

    # El número de línea permite localizar errores en la captura original.
    with Path(archivo).open(encoding="utf-8-sig", newline="") as entrada:
        for numero, linea in enumerate(entrada, start=1):
            linea = linea.strip()
            if not linea:
                continue
            if linea.startswith("#"):
                eventos.append(linea)
                continue

            try:
                fila = next(csv.reader([linea], strict=True))
                if not encabezado_recibido:
                    if fila != ENCABEZADO:
                        raise ValueError("encabezado incompatible con Registro.h")
                    encabezado_recibido = True
                    continue

                t_ms, modo, ref, w, error, u, pwm, nueva = fila
                tiempo, motor, actualizada = int(t_ms), int(pwm), int(nueva)
                referencia, velocidad, diferencia, freno = map(float, (ref, w, error, u))

                # Un archivo representa un ensayo con un solo modo y un reloj creciente.
                if modo not in ("OA", "PID") or tiempo < 0 or (
                    muestras and (modo != muestras[-1]["modo"] or tiempo <= muestras[-1]["t_ms"])
                ):
                    raise ValueError("modo inválido o tiempo de ensayo fuera de orden")
                if (
                    not all(map(math.isfinite, (referencia, velocidad, diferencia, freno)))
                    or not 0 <= freno <= 1
                    or not 0 <= motor <= 255
                    or actualizada not in (0, 1)
                ):
                    raise ValueError("muestra fuera del formato")

                muestras.append(dict(
                    t_ms=tiempo, modo=modo, ref=referencia, w=velocidad,
                    e=diferencia, u=freno, pwm=motor, nueva=actualizada, t=tiempo / 1000,
                ))
            except (csv.Error, ValueError) as exc:
                raise ValueError(f"{archivo}, línea {numero}: {exc}") from exc

    # Una captura sin muestras no permite ningún análisis.
    if not muestras:
        raise ValueError(f"{archivo}: no contiene muestras de un ensayo")
    return muestras, eventos
