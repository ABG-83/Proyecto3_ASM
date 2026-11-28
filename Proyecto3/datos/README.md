# Ensayos a lazo abierto para identificar la planta

Usar dos velocidades iniciales, baja y alta, que el banco pueda alcanzar
de forma repetible. Elegir tres niveles de frenado dentro del recorrido
calibrado y conservar los mismos niveles para ambas velocidades. Anotar
los valores reales antes de comenzar.

| Velocidad inicial | Freno bajo | Freno medio | Freno alto |
| --- | --- | --- | --- |
| Baja | Ajuste y repetición de validación | Ajuste y repetición de validación | Ajuste y repetición de validación |
| Alta | Ajuste y repetición de validación | Ajuste y repetición de validación | Ajuste y repetición de validación |

Guardar cada ejecución de ajuste en `identificacion/` y las repeticiones
independientes en `validacion/`. Si el banco lo permite, registrar también
una desaceleración libre (`u=0`) por velocidad para observar la caída de
velocidad atribuible al rozamiento del banco.

## Captura

Antes de ensayar, comprobar que el firmware emite el encabezado CSV y
registra cuándo se corta el motor, cuándo se aplica el freno y cuándo
termina el ensayo. Verificar que se pueda registrar la velocidad inicial
medida y que haya marcas nuevas durante el frenado. Mantener documentado
el intervalo entre el corte del motor y la aplicación del freno.

Desde `Proyecto3`, abrir una captura para cada ejecución. Este comando
solo ejemplifica la ruta y el nombre de archivo:

```text
python scripts/python/capturar.py --port COM4 --salida datos/identificacion/oa_u050_w120_r1.csv
```

En el nombre, `u050` representa `u=0.50`, `w120` la consigna inicial de
120 rpm y `r1` la primera repetición. Guardar la repetición independiente
con otro número y en `validacion/`.

Completar una fila por ensayo en `condiciones_ensayos.csv`. En `archivo`
usar la ruta relativa a `datos/`, incluida la carpeta que indica si el
ensayo se usa para ajuste o validación. Registrar la consigna y la
velocidad medida en rpm justo antes de aplicar el freno, `u` entre 0 y
1, el número de marcas por vuelta y el periodo configurado en ms.
Anotar en microsegundos los pulsos calibrados para `u=0` y `u=1`.
Registrar en `observaciones` cualquier incidencia durante la ejecución.

## Revisión antes del ajuste

Comprobar con `leer_ensayo.py` que el CSV contiene un solo ensayo `OA`,
con tiempo creciente y muestras legibles. Revisar que las transiciones
de `pwm` y `u` coincidan con sus eventos y que el motor esté apagado
durante el tramo de frenado. Usar `nueva` para distinguir las mediciones
del sensor de las filas que solo repiten el último valor. Mantener los
ensayos de validación fuera de la estimación de parámetros.
