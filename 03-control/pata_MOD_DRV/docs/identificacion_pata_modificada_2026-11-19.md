# Identificación de la pata modificada · 2026-11-19

Ensayo automático del dashboard sobre la pata modificada (`ROBOT_ESAMBLE_alfred_mejorado`). Análisis con `matlab/identificar_articulacion.m`, `fc_vel_hz` = 3. Los CSV están en `matlab/datos/` con fecha del 19-nov.

| Articulación | Archivos |
|---|---|
| Coxa (M1) | `coxa_rampa.csv` + `coxa_vaiven (2).csv` (sin gravedad) |
| Tibia (M3) | `tibia_colgando_rampa.csv` + `tibia_colgando_vaiven.csv` (fémur fijo en 0.283 rad) |
| Fémur, tibia en 0 | `femur_rampa.csv` + `femur_tibia_estirada.csv` |
| Fémur, tibia en −1.55 rad | `femur_rampa (2).csv` + `femur_tibia_90.csv` |
| Fémur, tibia en +1.55 rad | `femur_rampa (3).csv` + `femur_tibia_recogida.csv` |

`femur_rampa (1).csv` (5 s, abortado) no se usó. Ningún archivo tiene nFAULT.

## Resultados

Modelo `I = Jeq·α + Ic·sign(ω) + b·ω + G·cos(θ + θ0)`, corrientes en A equivalentes del CS.

| | Jeq (A·s²/rad) | Ic (A) | b (A·s/rad) | G (A) | θ0 (rad) | R² | PWM de arranque +/− |
|---|---|---|---|---|---|---|---|
| Coxa | 0.00131 | 0.0604 | −0.0055 | — | — | 0.83 | 58 / 44 % |
| Tibia | 0.00105 | 0.0130 | −0.0002 | 0.0179 | −1.569 | 0.95 | 38 / 27 % |
| Fémur, tibia en 0 | 0.00164 | 0.0123 | 0.0141 | 0.1087 | −3.034 | 0.93 | 34 / 30 % |
| Fémur, tibia en −1.55 | 0.00141 | 0.0229 | 0.0059 | 0.0853 | 3.007 | 0.90 | 22 / 30 % |
| Fémur, tibia en +1.55 | 0.00161 | 0.0187 | 0.0085 | 0.0847 | −2.834 | 0.95 | 62 / 28 % |

Comandos de fricción sugeridos por el script: `M1,FRICI,0.0483,0.1207`, `M3,FRICI,0.0104,0.0334` y, en el fémur, entre `0.0098,0.0363` y `0.0149,0.0650` según la postura.

## Lectura

- **Fricción.** La tibia y el fémur tienen una fricción de Coulomb de 0.012–0.023 A, entre 3 y 8 veces menor que en la pata anterior (0.06–0.11 A). La gravedad del fémur (~0.09–0.11 A) ya es mucho mayor que la fricción. La coxa sigue alta (0.060 A, con arranque a 44–58 % de PWM).
- **b negativa** en la coxa y la tibia: la fricción baja al aumentar la velocidad (efecto Stribeck). En la sintonía conviene usar b = 0.
- **Gravedad del fémur.** Varía 26 % con la postura de la tibia y θ0 se mueve unos ±0.2 rad. El promedio es G ≈ 0.093 A y θ0 ≈ −3.05 rad. El fémur cuelga cerca de +1.47 rad y la horizontal queda en −0.11 rad: **el fémur baja con el encoder positivo** (el usuario lo comprobó a mano el 19-nov).
- **Gravedad de la tibia.** Se anula con la tibia en θ ≈ 0 y es restauradora hacia 0. El cero de la tibia se puso alineada con el fémur, con los dos colgando (confirmado por el usuario), y en este ensayo el fémur seguía colgando: por eso la tibia en reposo queda en 0. Este GRAV solo vale con el fémur colgando. Para cualquier postura del fémur, con el cero en la pose del CAD, el mismo resultado se escribe con el término acoplado: `M3,GRAV,0,0` y `M3,GRAV2,0.0179,3.1416,2,1`. La gravedad depende de q_fémur + q_tibia, que es el ángulo de la tibia bajo la horizontal (los dos encoders son positivos hacia abajo). Esta conversión supone que el fémur estaba vertical durante el ensayo; la calibración de gravedad la afinará.
- **El cero no se guarda.** El firmware no guarda el cero del encoder: vale 0 en la pose del arranque o del último `ENCODER,ZERO`. En los ensayos del fémur, la horizontal quedó cerca de 0; en los de la tibia, el fémur estaba colgando con 0.28 rad. Los θ0 de GRAV solo valen si el cero se pone siempre en la misma pose.
- **Inercias.** Jeq va de 0.00105 a 0.00164 A·s²/rad. Para el lazo de velocidad del fémur, usar la mayor (tibia en 0).

## Primera prueba de posición (19-nov, 22:14 y 22:17)

CSV: `matlab/datos/femur_prueba_ganancias.csv` y `tibia_prueba_ganancias.csv`.

- **Fémur 4/7, sin sobrepaso.** Los tres escalones que fallan son de bajada hacia 0–0.5 rad, donde la gravedad es máxima: se queda pegado a 0.015–0.175 rad del objetivo. Causa: el empujón de arranque sube como máximo a 2·Is = 0.072 A y el speed PI queda congelado mientras empuja; al bajar, la corriente neta es empujón − GRAV = 0.072 − 0.093 < 0, es decir, sigue levantando, y el fémur no se suelta. Con la pata vieja no pasaba porque Is valía 0.11. Arreglo provisional: Is = 0.07 en el fémur (tope 0.14 A). Arreglo de fondo, pendiente: que el tope del empujón sume la parte del GRAV que se opone al movimiento.
- **Fémur con Is = 0.07 (22:40 aprox., `femur_prueba_ganancias (1).csv`): 7/7.** Llega en 1.3–3.3 s (media ≈ 1.9 s), sin sobrepaso, con error final ≤ 0.018 rad; las tres bajadas que antes fallaban ahora llegan en 1.3–2.4 s. Sostener el fémur cerca de la horizontal cuesta ~0.093 A (57 % de PWM) de forma continua.
- **Tibia 6/8, sobrepaso ≤ 1 %, llegadas de 0.65 a 2.1 s.** Los dos escalones que fallan van a ±1.80 rad y se quedan en +1.76 y −1.74 empujando con 52–56 % de PWM: son topes mecánicos. El rango de prueba tiene que quedar dentro de ±1.6 rad.

## Calibración de gravedad v6 (19-nov, 22:33, `matlab/datos/calibracion_gravedad.csv`)

- Se cortó por seguridad cuando el fémur llegó a 2.078 rad. El rango de prueba era ±1.9 (grilla ±1.75), que lleva el fémur más allá de la vertical (+1.47) y la tibia contra sus topes. Además, varias posturas quedaron sin datos: el tiempo de espera para cambiar de postura era fijo, de 6 s, y a 0.15 rad/s un salto de 0.9–1.4 rad no alcanza. Corregido: ahora la espera crece con la distancia.
- Con la tibia en −1.4 y el fémur en 0.88, la tibia no pasó de −1.41 aunque empujaba con 57 % de PWM: hay un tope o un choque en esa postura.
- Aun así se midieron 13 posturas del fémur y 16 de la tibia. Ajuste (posturas nominales, s = +1):
  - Fémur: `GRAV,0.0785,-3.123` + `GRAV2,0.0133,-3.019,3,1` → R² 0.993, error rms 0.003 A (con la gravedad de la identificación: R² 0.867, rms 0.014 A).
  - Tibia: `GRAV,0,0` + `GRAV2,0.0133,-3.019,2,1` → R² 0.82, rms 0.004 A (identificación: R² 0.70).
- Fue la gravedad por defecto del dashboard hasta la calibración completa del 20-nov.

## Calibración de gravedad v6 completa (20-nov, 17:03, `matlab/datos/calibracion_gravedad.csv`)

- Grilla fémur ±1.04 × tibia ±1.25: 25 de 25 posturas válidas en cada articulación, sin cortes. Fricción media en movimiento: fémur 0.017 A, tibia 0.008 A.
- Ajuste (recalculado en MATLAB con el mismo método del dashboard; s = +1, el signo esperado):
  - Fémur: `GRAV,0.0677,-3.100` + `GRAV2,0.0102,-2.968,3,1` → R² 0.886, rms 0.006 A.
  - Tibia: `GRAV,0,0` + `GRAV2,0.0102,-2.968,2,1` → R² 0.805, rms 0.003 A.
  - Con estos datos, el ajuste parcial del 19-nov da fémur R² 0.55: el nuevo es claramente mejor.
- Es la gravedad por defecto del dashboard desde el 20-nov.
- **Prueba del fémur con la gravedad calibrada (20-nov, 17:21, `femur_prueba_ganancias (2).csv`, dither 10 %; el ESP32 tenía la calibración guardada en el dashboard, GRAV 0.0613):** 7/7, media 1.86 s, error final ≤ 0.009 rad (ayer ≤ 0.018), sobrepaso ≤ 1 %. Pegado el 27 % del tiempo en TRACK (ayer 29 %). Sostener el fémur cerca de la horizontal cuesta ahora 0.068 A (50 % de PWM), antes 0.093 A (57 %). La prueba de ayer fue sin dither, así que la comparación cambia dos cosas a la vez.

- **Prueba de la tibia con la gravedad calibrada (20-nov, 17:29, `tibia_prueba_ganancias (1).csv`, dither 10 %):** los 6 escalones normales llegaron en 0.9–2.1 s, con error ≤ 0.013 rad y sobrepaso ≤ 1 %. El rango quedó escrito como −14 en vez de −1.4, así que un escalón pidió −13.9 rad: la tibia bajó a 2 rad/s, sin resistencia y sin saltos de encoder, hasta **−2.44 rad**, y ahí se trabó empujando con unos 0.09 A durante 8 s. Ayer el tope estaba en −1.74 rad. Causa: el acople de la tibia estaba flojo. Al llegar al tope, el eje patinó y el encoder siguió contando sin que la tibia se moviera. El usuario lo apretó; la tibia no tiene daños. El dashboard ahora rechaza rangos fuera de ±2.2 rad.
- **Tibia con el acople apretado y el cero repuesto (20-nov, 17:37, rango −1.4…1.4, dither 10 %): 7/7.** Media 1.69 s (0.85–2.95 s), error final ≤ 0.014 rad, sobrepaso ≤ 1 %, barrido completo −1.3 ↔ 1.3 sin tocar topes.

**Estado al 20-nov:** fémur 7/7 y tibia 7/7 con la planta del 19-nov, la gravedad calibrada el 20-nov y dither 10 %. Coxa con sus ganancias anteriores (7/7 el 19-nov).

## Pendiente

- Sentidos comprobados a mano el 19-nov: en el fémur y la tibia, el encoder negativo sube la articulación. Cero de la tibia: alineada con el fémur.
- Fijar una pose de cero única para todas las sesiones (propuesta: fémur horizontal y tibia alineada, la pose cero del CAD).
- Recalcular SPI y PPI en el panel de sintonía por asignación de polos con estas plantas, y probar con "Probar en la pata".
- Calibrar la gravedad (GRAV y GRAV2), porque ahora la fricción es bastante menor que la gravedad.
