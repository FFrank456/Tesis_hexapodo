# Pruebas simples en placa — 29/09/2026

Autorizadas por el usuario con motor y ESP32 disponibles. Puerto COM9, 460800 baudios, sin flashear ni cambiar ganancias. Se confirmó DATA v6, SPEED v1 y POSITION v1; corriente 8.832/80090, velocidad 0.0284/0.01392 y posición 6.166/3192.

Se ejecutaron tres ensayos cortos mediante `tests/run_cascade_smoke.py --run --stage current|speed|position`. Techo PWM 40 %, supervisión host de datos, pérdidas DMA, recorrido, velocidad y CS máximo 150 mA equivalentes. Cada ejecución solicitó STOP en finally y confirmó eje detenido. La supervisión serial no constituye protección independiente. No hubo errores de protocolo ni pérdidas DMA.

## Resultados

Los promedios de corriente y velocidad corresponden a los últimos 0.4 s de cada etapa. Los picos incluyen el transitorio; todos los valores de corriente son CS escalado, sin calibración física completa.

| Ensayo | Referencia | Resultado | Observación |
|---|---|---|---|
| Corriente CW, ~0.83 s | 10 mA eq. | 11.20 mA eq. | Pico 14.30 mA; PWM medio 14.87 %. Sin giro apreciable. |
| Corriente CW, ~0.83 s | 20 mA eq. | 19.98 mA eq. | Pico 21.53 mA; PWM medio 21.04 %. Sin giro apreciable. |
| Velocidad CW, ~2 s | +1 rad/s | +0.890 rad/s | Pico 1.077 rad/s; PWM máximo 27.65 %. |
| Velocidad CCW, ~2 s | −1 rad/s | −0.877 rad/s | Pico absoluto 1.232 rad/s; PWM máximo 30.15 %. |
| Posición primer objetivo, ~3 s activos | 0.208344 rad | 0.226643 rad | Error −0.018300 rad, dentro de tolerancia 0.02 rad; alcanzó HOLD/coast vigilado. |
| Posición segundo objetivo, ~3 s activos | −0.091656 rad | 0.224399 rad antes de STOP | Error −0.316056 rad; casi no avanzó en CCW. No alcanzó objetivo. |

Para posición se usaron máximo de referencia 0.5 rad/s, rampa 1 rad/s², tolerancia 0.02 rad y límite de referencia CS de 30 mA eq. Ambos objetivos se definieron respecto a la posición inicial del ensayo 0.058343864 rad: +0.15 y −0.15 rad desde ese punto. El segundo desplazamiento solicitado desde el final del primero fue aproximadamente −0.3183 rad. No se cambió el cero del encoder.

En el primer movimiento la velocidad medida alcanzó 1.165 rad/s, por encima de la referencia máxima de 0.5 rad/s. Esto confirma que limitar la referencia no limita estrictamente la velocidad física. El segundo movimiento quedó con referencia de velocidad −0.5 rad/s y lazo de velocidad saturado en 30 mA eq.; CS medio 29.99 mA, PWM medio −29.17 %, velocidad casi nula. La inferencia es falta de esfuerzo para vencer fricción/carga en ese punto; estos registros no permiten distinguir sus causas. El movimiento CCW previo a −1 rad/s confirma que el sentido contrario sí funcionó en otra condición.

Las pruebas verifican comunicación, ejecución y algunas respuestas básicas. `failure=null` en los resúmenes significa que no se dispararon los límites de supervisión; **no significa que todos los objetivos de control se cumplieran**. No se valida estabilidad general ni mantenimiento de posición bajo carga. No se aumentó el esfuerzo tras el objetivo CCW fallido.

## Estado final

Confirmado OPEN/STOP, PWM=0 %, CS=0 y velocidad=0. Posición final 0.226643470 rad. Se restauraron límites de posición 2 rad/s, 5 rad/s² y 0.02 rad, ILIMIT=0.1 A eq. y consignas IREF/WREF/PREF=0. Se dejó LIMIT=0..40 %. Ganancias y origen del encoder sin cambios. Puerto cerrado, disponible para Processing. Las configuraciones seriales son volátiles.

## Evidencia

- [Corriente: resumen](../build-codex/cascade-current-20260929-144709.summary.json), [registro serial](../build-codex/cascade-current-20260929-144709.jsonl).
- [Velocidad: resumen](../build-codex/cascade-speed-20260929-144728.summary.json), [registro serial](../build-codex/cascade-speed-20260929-144728.jsonl).
- [Posición: resumen](../build-codex/cascade-position-20260929-144801.summary.json), [registro serial](../build-codex/cascade-position-20260929-144801.jsonl).
- [Restauración y STOP final](../build-codex/cascade-restored-20260929.json).

Cada ensayo incluye además un CSV con los índices DATA originales; los JSONL conservan las extensiones y comandos exactos. Estos archivos son registros de prueba, no exportaciones CSV v3 de Processing.

Siguiente paso propuesto: caracterizar arranque y seguimiento de velocidad baja en ambos sentidos, con una pequeña secuencia repetible, antes de retocar el PI de posición. El lazo exterior depende de que el de velocidad siga referencias pequeñas sin quedarse inmóvil ni acelerar muy por encima de ellas.
