---
fecha: 2026-10-14
tags: [diseno-electronico, alimentacion, bateria]
---
# Batería LiPo 3S: corriente máxima y dimensionamiento

> [!important] Actualizado 2026-11-09 ([[2026-11-08-cambios-acordados]])
> La LiPo 3S queda **solo para la Unidad 1** (la roja prestada por Marco). La **Unidad 2** usa una batería de **~14,4 V (4S)**, química y modelo por confirmar, porque el MP1584 no regula 12 V con 3S. **No se usan baterías 2S.** Ver [[arquitectura-electronica-hexapodos]].

Batería de la **Unidad 1**: **LiPo 3S (11,1 V nominal; 12,6 V cargada; ~9,9 V descargada), 3300 mAh (opción 5000 mAh), ≥ 25C, conector XT60.** Solicitadas 2 en préstamo a Marco (cc. prof. Dante Inga), 2026-10-12.

## Corriente máxima

$$I_{max} = \text{Capacidad (Ah)} \times C$$

| Batería 3S | Continua (25C) | Ráfaga (50C, ~10 s)* |
|---|---|---|
| 3300 mAh | 3,3 × 25 = **82,5 A** | ~165 A |
| 5000 mAh | 5,0 × 25 = **125 A** | ~250 A |

\* Solo si la etiqueta indica la ráfaga (p. ej. "25C/50C").

## Límites prácticos

- **Conector XT60:** ~60 A continuos. Es el límite real antes que la batería. Para más, XT90.
- **Cable:** 12 AWG ≈ 40–60 A; 14 AWG ≈ 30 A.
- **Tasa C real:** las etiquetas suelen ser optimistas. Para diseño, tomar ~50 %: unos **40 A** utilizables en la 3300 mAh 25C sin caída fuerte de voltaje ni calentamiento.

## Demanda de cada unidad

| Caso | Corriente desde la batería |
|---|---|
| Unidad 1, pico de servos 10 A a 6 V + RPi 5 (~75 W) | ≈ 7,5 A |
| Unidad 1, atasco simultáneo de servos (20–30 A a 6 V) | ≈ 12–18 A |
| Unidad 2, 150 W de diseño, con 3S (ya no aplica) | ≈ 15 A |
| Unidad 2, 150 W de diseño, con ~14,4 V (4S) | ≈ 11,6 A |

Cálculo: $I_{bat} = P / (V_{bat} \cdot \eta)$, con $V_{bat}$ = 11,1 V y $\eta$ ≈ 0,9 (buck).

**Conclusión:** la 3S 3300 mAh 25C con XT60 cubre la Unidad 1 (y cubriría la U2 en corriente, pero no en tensión para regular 12 V) con margen amplio, incluso en el peor caso (< 40 A).

## Autonomía estimada (Unidad 1)

Con 3–4 A promedio y 80 % de capacidad útil: ~40–50 min (3300 mAh); ~60–70 min (5000 mAh, +~130 g).

## Protección

- Fusible ~20 A en la salida de la batería (Unidad 1).
- Buck 6 V ≥ 15 A para servos y buck 5,1 V / 5 A para la Raspberry Pi 5, ambos con entrada ≥ 13 V.
