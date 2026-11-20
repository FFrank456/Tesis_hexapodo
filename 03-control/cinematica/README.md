# Cinemática directa geométrica — pata modificada (Unidad 2)

Este trabajo es independiente de `01-mecanica/cinematica-directa-pata`, cuyas dimensiones no corresponden a la pata modificada.

| Archivo | Qué hace |
|---|---|
| `cd_pata.m` | Función de cinemática directa: ángulos del modelo (rad) → pie (mm) en el marco de J0 |
| `simular_cinematica_directa.m` | Planteamiento matemático, geometría medida en Fusion, comprobación con matrices homogéneas, validación contra Fusion, espacio de trabajo y simulación con deslizadores |

## Geometría (Fusion, `ROBOT_ESAMBLE_alfred_mejorado`, 19-nov-2026)

Medida a través del MCP de Fusion en la pose cero (J0 = J1 = J2 = 0).

| Parámetro | Valor | Significado |
|---|---|---|
| L1 | 50.000 mm | J0 → J1, radial |
| h1 | 6.050 mm | J1 más arriba que J0 |
| L2 | 60.000 mm | J1 → J2 (el fémur está horizontal en la pose cero) |
| u, w | 97.886 / 36.356 mm | J2 → pie: avance y caída |
| L3, gamma | 104.420 mm / 20.376° | Recta equivalente de la tibia |
| d | −0.071 mm | Desfase lateral del pie respecto al eje J0 |

El pie es el punto más bajo de `soporte_tibia_A` en la pose cero: (244.504, −112.635, −40.009) mm en el marco raíz de Fusion. El plano de la pata está a −45° en planta.

## Juntas verificadas

- Los ejes de `join_coxa_1_J0`, `join_femur_1_j1` y `join_femur_1_j2` coinciden (0.000 mm, paralelos) con el eje D de 3 mm, la carcasa del N20, la brida y el buje de su motor.
- J0 es vertical; J1 y J2 son horizontales y paralelos.
- Sentido de giro en Fusion, medido con +10°: J0+ gira en sentido horario visto desde arriba, J1+ sube el fémur y J2+ sube la tibia. Por eso `th_modelo = [-1 1 -1] .* q_Fusion`.
- El modelo reproduce en Fusion la pose cero, los tres giros aislados y dos poses combinadas, (20, −15, 25)° y (−35, 40, −60)°, con un error ≤ 0.001 mm.

## Pendiente

- **Encoder → modelo:** `th = [1 -1 1] .* q + offset`.
  - Fémur: el encoder negativo lo sube (comprobado a mano el 19-nov y coherente con la identificación, donde cuelga en +1.47 rad).
  - Tibia: el encoder negativo la sube (comprobado a mano el 19-nov), así que el positivo es flexión hacia abajo, como en el modelo.
  - Coxa: sin verificar.
  - Cero de la tibia: se pone con la tibia alineada con el fémur (el 19-nov, con los dos colgando), igual que en el CAD, así que `offset` de la tibia = 0.
  - El firmware **no guarda el cero**: al arrancar, el encoder vale 0 en la pose en que esté la pata. Para que la cinemática y los GRAV valgan, hay que poner el cero siempre en la misma pose. Propuesta: la pose cero del CAD (fémur horizontal, tibia alineada con él).
- Límites reales y colisiones (se usa ±90°, según la nota de rediseño).
- El pie es redondeado, así que el punto de contacto real se desplaza con la orientación de la tibia; el modelo sigue un punto fijo de la tibia.

## Ejecutar

En MATLAB, abrir esta carpeta y ejecutar `simular_cinematica_directa`. Desde PowerShell, en la raíz de `TESIS_2`, solo la parte numérica:

```powershell
matlab -wait -batch "cd('03-control/cinematica'); simular_cinematica_directa"
```
