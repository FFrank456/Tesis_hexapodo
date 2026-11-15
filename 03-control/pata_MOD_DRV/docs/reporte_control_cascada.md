# Reporte del control en cascada de la pata MOD_DRV

Estado al 14-nov-2026. Este archivo es una copia del reporte del 13-nov, con lo agregado el 14-nov (FRICB y DITHER).

La cascada corriente → velocidad → posición ya controla la pata ensamblada. En la última prueba (12-nov), las tres articulaciones llegaron al objetivo en 1.1–2.8 s, con error final ≤ 0.016 rad y sobrepaso ≤ 14 %.

- **Ganancias:** se calculan por asignación de polos en el dashboard y se envían por comando. Viven solo en la RAM del ESP32; los valores por defecto del firmware no se cambiaron y no sirven para la pata.
- **Lo que más mejoró el control** no fueron las ganancias sino las compensaciones: fricción en la referencia de corriente con empujón de arranque, mapa de fricción por ángulo y gravedad.
- **Pendiente principal:** calibrar en la pata la gravedad acoplada fémur + tibia, rehacer el mapa de fricción, probar FRICB y DITHER, y guardar todo en la NVS.

## Arquitectura de la cascada

```
θ objetivo → [Posición 50 Hz, P + vmax/acel.] → ω ref → [Velocidad 100 Hz, PI] → (+) → I ref → [Corriente 1 kHz, PI] → PWM 10 kHz → DRV8876 + N20
                     ↑ θ                                  ↑ ω                      ↑ compensaciones                ↑ I medida (IPROPI, ADC)
                     └────────────── encoder (PCNT) ───────┘                 fricción FRICI/FRICB/FMAP/DITHER, gravedad GRAV/GRAV2
```

| Lazo | Frecuencia | Controlador | Medida | Retardo |
|---|---|---|---|---|
| Corriente | 1 kHz | PI, salida en % de PWM (10 kHz) | IPROPI del DRV8876, ADC DMA 64 kS/s por canal, bloque de 1 ms | 1 muestra |
| Velocidad | 100 Hz | PI, salida en A | Encoder por PCNT, diferencia en ventana de 20 ms | ~19 ms en total |
| Posición | 50 Hz | P (Ki = 0), con límites de velocidad y aceleración (PLIMIT) | Encoder | 10 ms |

Al entrar en la tolerancia, la posición pasa a SOSTENER, con freno (IN1 = IN2 = 1) o con la corriente de gravedad. Vuelve a corregir si el error supera 2 × tolerancia.

## Planta identificada

El fémur es la articulación mejor identificada (R² = 0.92) y la tibia la peor (R² = 0.74). El modelo se ajustó con `matlab/identificar_articulacion.m` sobre los CSV del ensayo automático del 11-nov:

I = Jeq·α + Ic·sign(ω) + b·ω + G·cos(θ + θ0)

| Articulación | a (A/%) | Jeq (A·s²/rad) | b (A·s/rad) | Ic (A) | Is arranque (A) | G (A) | θ0 (rad) | PWM de arranque + / − | R² |
|---|---|---|---|---|---|---|---|---|---|
| M1 Coxa | 0.00292 | 0.00061 | 0.00527 | 0.046 | 0.10 | — | — | 56 / 42 % | 0.83 |
| M2 Fémur (tibia estirada) | 0.00268 | 0.00154 | 0.00582 | 0.041 | 0.11 | 0.093 | −2.753 | 40 / 30 % | 0.92 |
| M3 Tibia | 0.00318 | 0.00126 | 0.01369 | 0.059 | 0.20 | 0.050 | −1.681 | 48 / 58 % | 0.74 |

- **Unidades de corriente:** A equivalentes del sensado, es decir, la corriente real × el ciclo de trabajo, con una zona muerta de ~20 % de PWM. `a` es la ganancia PWM → corriente medida.
- **Is** es la corriente de arranque vista en las pruebas de posición del 12-nov, descontando la gravedad.
- **La gravedad de la tibia** se identificó con el fémur en 0; para otras posturas hace falta GRAV2.
- **Escala del encoder:** el firmware supone 2800 cuentas por vuelta, sin verificar.

## Diseño por asignación de polos

El retardo de ~19 ms del lazo de velocidad fija el límite: ese lazo no puede establecerse en menos de 0.31–0.35 s (≈ 2 Hz) sin perder margen.

- **Corriente** (polo discreto exacto, una muestra de retardo): Ki = (1 − e^(−4T/ts)) / (a·T), Kp = Ki·τe, con τe = 0.2 ms.
- **Velocidad:** el retardo τ se modela como un polo. Se iguala J·τ·s³ + (J + b·τ)·s² + (b + Kp)·s + Ki = J·τ·(s² + 2ζωn·s + ωn²)(s + p3). El tercer polo p3 lo fija la planta y debe quedar ≥ 3 veces más rápido.
- **Posición** (solo P, sobre el lazo de velocidad ya cerrado, con retardo equivalente τp): τp·s² + s + Kp = 0, de donde Kp = σ − τp·σ², con σ = 4/ts.

| Lazo | Desempeño elegido | Mínimo alcanzable |
|---|---|---|
| Corriente | ts = 15 ms (≈ 37 Hz de cruce) | ts ≈ 6 ms |
| Velocidad | ts = 0.4 s, sin sobrepaso | 0.31–0.35 s |
| Posición | ts = 1.0–1.3 s, solo P | 0.5–1.1 s según la articulación |

Requisitos de la cascada: cada lazo ≥ 4× más rápido que el exterior, cruce ≤ 1/10 del muestreo (100 / 5 / 2.5 Hz) y margen de fase ≥ 45°.

## Ganancias

| Lazo | Por defecto del firmware (Kp / Ki) | M1 Coxa | M2 Fémur | M3 Tibia |
|---|---|---|---|---|
| Corriente `PID` | 8.832 / 80090 | 16.03 / 80162 | 17.47 / 87340 | 14.72 / 73607 |
| Velocidad `SPI` | 0.0284 / 0.01392 | 0.00547 / 0.0480 | 0.0185 / 0.1072 | 0.00956 / 0.1044 |
| Posición `PPI` | 6.166 / 3192 | 2.084 / 0 | 2.971 / 0 | 1.772 / 0 |

| Lazo | Por defecto (fémur) | Diseño coxa | Diseño fémur | Diseño tibia |
|---|---|---|---|---|
| Corriente | 34 Hz, MF 73° | 37 Hz | 37 Hz | 37 Hz |
| Velocidad | 2.9 Hz, MF 80°, con un polo lento en 0.4 rad/s | 1.4 Hz | 2.0 Hz | 1.2 Hz |
| Posición | 6.7 Hz, MF −129° (inestable) | 0.32 Hz | 0.43 Hz | 0.28 Hz |
| Separación corriente/velocidad · velocidad/posición | 12× · 0.4× | 26× · 4.6× | 18× · 4.7× | 31× · 4.4× |

El margen de fase del diseño queda entre 68° y 81°. Las frecuencias del diseño se calcularon con un ts de posición de 1.2 s.

## Compensaciones

Se suman todas a la referencia de corriente, no al PWM. Hay una excepción: DITHER, que va directo al PWM porque su frecuencia está por encima del ancho de banda del lazo de corriente.

| Compensación | Qué hace | Comando | Estado |
|---|---|---|---|
| Coulomb + empujón de arranque | Suma Ic mientras la articulación se mueve. Si está pegada, da un empujón de Is a 2×Is. El integrador de velocidad no acumula mientras está pegada y se borra si se pasa de velocidad | `FRICI,Ic,Is` | Probado en la pata |
| Forma del empujón | Arranca en Is (antes arrancaba en Ic) y sube hasta 2×Is en 200 ms. Cerca del objetivo arranca en Ic | `FRICB,1,200` | **Nuevo, sin probar en la pata.** En la simulación, la zona dura se cruza en 4.8 s en vez de 10.5 s |
| Vibración (*dither*) | Onda cuadrada ±pct % en el PWM mientras se pide movimiento | `DITHER,10,80` | **Nuevo, sin probar en la pata.** En la simulación, el eje pegajoso llega en 4.7 s en vez de 14 s |
| Mapa de fricción | Ic por ángulo y por sentido, en tramos de 0.05 rad, sacado del recorrido del espacio de trabajo | `FMAP,…` | Probado en la simulación; hay que rehacerlo en la pata después de calibrar la gravedad |
| Gravedad propia | G·cos(θ + θ0) | `GRAV` | Probado en la pata |
| Gravedad acoplada | Segundo término según el ángulo de otra articulación (tibia: fémur + tibia) | `GRAV2` | En la simulación recupera G con < 1 % de error; falta en la pata |

## Protecciones

| Protección | Comportamiento |
|---|---|
| Picos del sensado | Un bloque de 64 muestras sigue siendo válido con hasta 32 en el tope del ADC |
| Bloques inválidos | Hasta 5 seguidos mantienen el último PWM; desde el sexto, el motor se detiene |
| Parada con gravedad | La articulación queda frenada, no suelta; nFAULT siempre la suelta |
| Sobrevelocidad | Freno si va 0.5 rad/s por encima de lo pedido sin corriente |
| Pruebas del dashboard | Detienen todo si una articulación sale de su rango más de 0.08 rad, si la corriente pasa de 0.65 A durante 0.3 s o si salta nFAULT |

## Resultados en la pata (12-nov)

| Articulación | Llegaron | Tiempo medio | Peor error | Peor sobrepaso | Antes de FRICI |
|---|---|---|---|---|---|
| M2 Fémur | 6 / 6 | 1.44 s | 0.013 rad | 0 % | tirones y caídas |
| M3 Tibia | 6 / 6 | 1.76 s | 0.016 rad | 14 % | 5–7 s por movimiento |
| M1 Coxa | 5 / 6 | 2.51 s | 0.015 rad | 14 % | se quedaba pegada |

- **Fémur en −1…1:** 6 de 7 llegaron; en −0.89 → 0 se pasó un 31 %.
- **Coxa en −0.5…1.5:** se disparó a 9.6 rad/s después de despegarse. Ya está corregido; falta repetir la prueba.
- **Recorrido del espacio de trabajo:**
  - punto duro del fémur en −0.60…−0.65 rad;
  - hacia + la fricción es cerca del doble que hacia −;
  - *stick-slip* del 40–70 % a 0.3 rad/s;
  - los rebotes de la tibia los causaba la gravedad sin acoplar.

## Pendiente

1. Probar FRICB y DITHER en la pata.
2. Calibrar la gravedad, rehacer el recorrido y el mapa de fricción, y validar.
3. Repetir la coxa en su rango amplio y revisar el sobrepaso de 31 % del fémur.
4. Guardar todo en la NVS.
5. Verificar la escala del encoder.
6. Cambiar los bujes de PLA por rodamientos.
7. Preparar las tablas para la tesis.

Fuentes: firmware y dashboard en `03-control/pata_MOD_DRV` (repositorio FFrank456/Tesis_hexapodo); datos en `matlab/datos`.
