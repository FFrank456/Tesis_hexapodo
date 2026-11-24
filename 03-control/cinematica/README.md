# Cinemática directa geométrica — pata modificada (Unidad 2)

Este trabajo es independiente de `01-mecanica/cinematica-directa-pata`, cuyas dimensiones no corresponden a la pata modificada.

| Archivo | Qué hace |
|---|---|
| `cd_pata.m` | Función de cinemática directa: ángulos del modelo (rad) → pie (mm) en el marco de J0 |
| `simular_cinematica_directa.m` | Planteamiento matemático, geometría medida en Fusion, comprobación con matrices homogéneas, validación contra Fusion, espacio de trabajo y simulación con deslizadores |
| `ci_pata.m` | Cinemática inversa analítica: pie (mm, marco de J0) → ángulos del modelo. Vectorizada (3×N) |
| `geometria_pata.m` | Geometría, conversión encoder ↔ modelo (`th = signo·q + offset`) y límites de los encoders |
| `verificar_ci.m` | Comprueba la inversa contra la directa, las poses de Fusion y los puntos inalcanzables; dibuja `espacio_trabajo_ci.png` |

## Cinemática inversa (20-nov)

Inversión cerrada de las ecuaciones de `cd_pata`:
- **Planta:** `(x, y) = Rot(th0)·(r, d)`, de donde `r = ±√(x² + y² − d²)` y `th0 = atan2(y, x) − atan2(d, r)`. Se elige el signo de r que deja |th0| ≤ 90°: la coxa no gira más. Con r < 0, el pie queda detrás del eje de la coxa (fémur muy abajo y tibia muy doblada).
- **Plano de la pata:** con `a = r − L1` y `b = z − h1`, el ángulo de la rodilla `β = −(th2 + γ)` cumple `cos β = (a² + b² − L2² − L3²) / (2·L2·L3)`. Luego `th1 = atan2(b, a) − atan2(L3·sin β, L2 + L3·cos β)` y `th2 = −β − γ`.
- **Dos ramas de la rodilla:** `rama = +1` (por defecto) es la tibia doblada hacia abajo respecto de la recta del fémur (`th2 ≥ −γ = −20.4°`), y `rama = −1` es la otra.
- **Verificación** (`verificar_ci`): 20 000 poses al azar dentro de los límites de los encoders vuelven con error ≤ 4·10⁻¹² rad. También reproduce las 6 poses validadas en Fusion y marca como inalcanzables los puntos fuera del anillo.
- **Espacio de trabajo** con la coxa en 0, la rodilla normal y los límites provisionales: el pie va de −154 a +150 mm de altura respecto de J0 y de 2 a 214 mm de alcance radial. Ejemplo: el pie en (150, 0, −110) mm corresponde al encoder (0, +0.363, +0.418) rad.
- **Límites de los encoders provisionales** en `geometria_pata.m`: coxa ±0.6, fémur ±1.2 y tibia ±1.6 rad, hasta medir los rangos a mano.

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
  - Coxa: el encoder negativo gira en sentido horario visto desde arriba (comprobado a mano el 20-nov), así que el positivo es antihorario, como en el modelo. Falta definir su cero.
  - Cero de la tibia: se pone con la tibia alineada con el fémur (el 19-nov, con los dos colgando), igual que en el CAD, así que `offset` de la tibia = 0.
  - El firmware **no guarda el cero**: al arrancar, el encoder vale 0 en la pose en que esté la pata. Para que la cinemática y los GRAV valgan, hay que poner el cero siempre en la misma pose. Propuesta: la pose cero del CAD (fémur horizontal, tibia alineada con él).
- Límites reales y colisiones (se usa ±90°, según la nota de rediseño).
- El pie es redondeado, así que el punto de contacto real se desplaza con la orientación de la tibia; el modelo sigue un punto fijo de la tibia.

## Validación en la pata real (pendiente)

Primer intento (20-nov), midiendo avance, altura y lateral con signo respecto de la pose cero: el avance tuvo el signo del modelo, pero las alturas eran físicamente imposibles. Por ejemplo, −138 mm solo con la tibia a 0.8 rad, cuando el máximo posible es 81 mm, y +200 mm al girar solo la coxa, que tiene el eje vertical. El método con signos se presta a confusión, así que se mide de nuevo con dos distancias sin signo.

En cada pose se miden **H**, la altura de la punta del pie sobre la mesa, y **R**, la distancia horizontal desde el eje de la coxa a la punta del pie, ambas en mm. Como H depende de dónde está la mesa, se compara ΔH = H − H(pose cero). Valores predichos por `cd_pata` con `th = [1 -1 1]·q`:

| q coxa | q fémur | q tibia | R (mm) | ΔH (mm) |
|---|---|---|---|---|
| 0 | 0 | 0 | 207.9 | 0 |
| 0 | 0 | 0.8 | 152.1 | −59.2 |
| 0 | −0.5 | 0 | 206.0 | +80.1 |
| 0 | −0.5 | 1.2 | 154.1 | −25.7 |
| 0 | 0.5 | 0.5 | 124.9 | −94.4 |
| 0 | −1.0 | 1.5 | 150.9 | +8.0 |
| ±0.5 | 0 | 0 | 207.9 | 0 (el pie gira; la distancia entre las dos marcas, +0.5 y −0.5, es 199.3 mm: 99.7 mm a cada lado) |

## Ejecutar

En MATLAB, abrir esta carpeta y ejecutar `simular_cinematica_directa`. Desde PowerShell, en la raíz de `TESIS_2`, solo la parte numérica:

```powershell
matlab -wait -batch "cd('03-control/cinematica'); simular_cinematica_directa"
```
