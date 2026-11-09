---
tipo: resultado-prueba
area: control
unidad: Unidad 1
fecha: 2026-10-01
hardware: Pololu Mini Maestro 18 + 18× MG996R
tags:
  - control
  - servos
  - calibracion
  - maestro
---

# Calibración de las tibias del hexápodo (Mini Maestro 18, MG996R)

> [!note] Reemplaza a la calibración anterior con ESP32 + 2× PCA9685
> Esa calibración se eliminó el 2026-11-09 porque la Unidad 1 ya no usa ESP32 ni PCA9685 (ver [[arquitectura-electronica-hexapodos]]).

## Mapa de canales (tibia, fémur, coxa)
LF 0, 1, 2 | LM 3, 4, 5 | LR 6, 7, 8 | RR 9, 10, 11 | RM 12, 13, 14 | RF 15, 16, 17

## Barrido de las tibias
| Posición física | LF (ch0) | RF (ch15) |
|---|---|---|
| Arriba, sobre el cuerpo | — | ~2000 µs (choca) |
| Pose "estar" (0°) | 512 µs | ~1990 µs |
| De frente (horizontal) | 1500 µs | 1000 µs |
| Extremo inferior | 2496 µs (abajo, casi toca el pecho) | 500 µs (mínimo) |

- El lado derecho gira **en espejo** y con un desfase de ~45° (~500 µs): RF ≈ 2500 − LF.
- La pose HOME (tibias a 170° = 2389 µs en todas) choca porque las tibias derechas giraron hacia arriba, sobre el cuerpo.

## Pendiente
- [ ] Corregir mecánicamente el horn de las tibias derechas (~3 dientes, 14,4° por diente) para que quede RF = 3000 − LF.
- [ ] Volver a verificar RM, RR, LM y LR.

## Configuración del Maestro
- Tibias (canales 0, 3, 6, 9, 12, 15): mínimo ~496–512 µs, máximo 2496 µs.
- Run script on startup: **desactivado**.
- Script guardado: locomoción de la pata LF con la pose "estar" (no arranca solo).

Relacionado: [[urdf-pose-cero-hexapodo]]
