# pata_MOD_DRV — contexto para Claude

Firmware y herramientas de control de **una pata** (coxa, fémur, tibia) del hexápodo de la Unidad 2, sobre la placa propia **MOD_DRV**: un ESP32 con 3 drivers DRV8876 y 3 motores N20. Tesis de Alfredo Erazo (UTEC). Responder en español, directo.

Reporte completo del estado del control: `docs/reporte_control_cascada.md`.

## Cómo quiere trabajar el usuario

- Cuando le pidas ejecutar algo (flashear, una prueba, una calibración), explica cada paso en detalle: **qué hacer, cuánto debería tardar, qué debería pasar y qué hacer si no funciona**.
- Las pruebas en la pata las hace él con el dashboard. Tú analizas los CSV que guarda en `matlab/datos/`.
- Antes de decir "listo", verifica: tests en PC, compilación con ESP-IDF y, en el dashboard, una prueba con el ESP32 simulado si tocaste JavaScript.

## Hardware

| Motor | Articulación | Conector | IN1 / IN2 | IPROPI (ADC1) | Encoder A / B | nFAULT |
|---|---|---|---|---|---|---|
| M1 | Coxa | J3A (DRVA) | 26 / 27 | 32 (CH4) | 35 / 25 | 5 |
| M2 | Fémur | J3B (DRVB) | 18 / 19 | 33 (CH5) | 23 / 17 | 14 |
| M3 | Tibia | J3C (DRVC) | 16 / 2 | 34 (CH6) | 4 / 13 | 15 |

- Los pines están en `main/pin_map.h`. R_IPROPI = 3 kΩ. El encoder de la coxa en IO35 es solo entrada (no tiene pull-up interno).
- Motores N20 12 V 100:1 de AliExpress. Necesitan 40–60 % de PWM para arrancar, por la fricción de los bujes de PLA.
- **Escala del encoder sin verificar.** El firmware supone 2800 cuentas por vuelta (7 PPR × 4 × 100). Si al girar 90° a mano no da 1.57 rad, corregir `encoder.h`.
- **La corriente medida es la real × ciclo de trabajo**, porque el DRV8876 solo reporta con el PWM encendido. Todas las corrientes (A) del firmware y del dashboard están en esas unidades, ILIMIT incluido.

## Compilar, flashear y probar

- **ESP-IDF v5.4.4** en `C:\esp\v5.4.4\esp-idf`, con la extensión de VS Code. Para compilar desde PowerShell, primero `. C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1` (el `export.ps1` del IDF falla en esta instalación) y después `idf.py build`. El puerto está en `.vscode/settings.json` (hoy COM16). "Acceso denegado" significa que el dashboard o el monitor tienen el puerto abierto.
- Tests en PC, desde esta carpeta:
  ```
  gcc -std=gnu11 -Itests/host_stubs -Imain tests/test_main_host.c main/pwm_sequence.c -lm -o t && ./t
  gcc -std=c11 -Imain tests/test_dma_sense.c -lm -o t && ./t
  python3 tests/test_serial_schema.py
  ```
  `test_main_host.c` simula la planta (fricción estática, zona dura de PLA, gravedad) e imprime los tiempos de llegada.
- **Dashboard:** `dashboard/pata_dashboard.html`. Usa Web Serial, así que se abre en Chrome o Edge (no en un perfil administrado por el centro educativo). Recargar con **Ctrl+F5** después de cada cambio.

## Arquitectura del control (cada motor, en `main/main.c`)

- **Corriente**, 1 kHz: PI en `current_pi.h`. ADC DMA 3 canales × 64 kS/s, bloques de 1 ms (`current_sense.c`). PWM MCPWM a 10 kHz.
- **Velocidad**, 100 Hz: PI en `speed_pi.h`. Velocidad del encoder (PCNT) en ventana de 20 ms. El retardo total del lazo es ~19 ms.
- **Posición**, 50 Hz: PI en `position_pi.h`, usado solo como P. Trayectoria con vmax y aceleración (PLIMIT). Estados TRACK / REVERSING / ARRIVING / HOLD. Al entrar en la tolerancia sostiene con freno o con la gravedad; vuelve a corregir si el error pasa de 2 × tolerancia.
- **Compensaciones**, sumadas con signo a la referencia de corriente en `cascade_current()`:
  - `friction_feedforward()`: FRICI, FRICB y FMAP;
  - `gravity_hold_a()`: GRAV y GRAV2.
- **Protecciones:**
  - un bloque de corriente sigue siendo válido con hasta 32 de 64 muestras saturadas;
  - hasta 5 bloques inválidos seguidos mantienen el último PWM (contador `cs_ride_through` en DIAG);
  - una articulación con gravedad queda frenada si el sensado falla;
  - nFAULT detiene ese motor.

## Comandos serie (`Mx,` = motor 1–3; las respuestas son `Mx,ACK,...` o `Mx,ERR,...`)

| Comando | Qué hace |
|---|---|
| `PID,kp,ki,0` | PI de corriente (%/A, %/(A·s)) |
| `SPI,kp,ki` | PI de velocidad (A/(rad/s), A/rad) |
| `PPI,kp,ki` | Lazo de posición (Ki = 0 recomendado) |
| `PLIMIT,vmax,acc,tol` · `ILIMIT,A` · `LIMIT,min,max` | Límites de trayectoria, de corriente y de PWM |
| `MODE,POSITION` · `PREF,rad` · `WREF,rad/s` · `STOP` · `HOLD,BRAKE\|COAST` | Modos y consignas |
| `FRICI,Ic,Is` | Fricción en la corriente: Ic mientras se mueve; empujón de arranque mientras está pegada |
| `FRICB,inicio,rampa_ms` | Empujón: inicio 1 = arranca en Is, 0 = en Ic; sube hasta 2×Is en rampa_ms. Con velocidad pedida < 0.2 rad/s arranca siempre en Ic. Por defecto 1, 200 |
| `DITHER,pct,hz` | Onda cuadrada ±pct % sumada al PWM mientras se pide movimiento (0 = apagado). Por defecto 0, 80 |
| `FMAP,+\|-,inicio,paso,v0,v1,…` · `FMAP,CLEAR` | Mapa de fricción por ángulo y sentido (hasta 48 tramos); reemplaza a Ic |
| `GRAV,G,θ0` | Gravedad G·cos(θ + θ0) |
| `GRAV2,G,θ0,motor,c` | Segundo término G·cos(θ + c·θ[motor] + θ0); la tibia depende del fémur |
| `FRIC,pct` | Antiguo, en % de PWM. **Dejar en 0**: el lazo de corriente lo cancela |
| `INFO`, `DIAG`, `TEL,MULTI` | Información, diagnóstico (ADC, encoder, nFAULT, ride-through) y telemetría a 100 Hz |
| `NVS,SAVE` | Guarda en la flash las ganancias, límites y compensaciones de los 3 motores (todo lo de CFG, más FMAP, FRICB, DITHER, ESIGN y HOLD). Exige los 3 motores en STOP |
| `NVS,POSE` · `NVS,POSE,CLEAR` | Guarda o borra la **pose de arranque**: los ángulos actuales, que el encoder toma al encender |
| `CAN,ID,n` · `CAN,ID` · `CAN,STATUS` | Asigna el ID de pata (1–6; se guarda en NVS y `NVS,ERASE` no lo borra) / lo muestra / estado del bus. Sin ID la placa no atiende órdenes CAN, solo PARO |
| `NVS,STATUS` · `NVS,ERASE` | Qué hay guardado / borrar todo (al reiniciar vuelven los valores de fábrica) |

La línea `CFG,Mx,...`, unas 3 veces por segundo, trae todos los parámetros. El dashboard la interpreta en `handleCfg`.

## Dashboard: paneles

- **Tarjetas M1, M2, M3:** pestañas Posición, Velocidad, Corriente y PWM, más las compensaciones.
- **Ensayo automático de identificación:** sentido del encoder, rampa y vaivén. Genera los CSV que usa `matlab/identificar_articulacion.m`.
- **Sintonía de la cascada por asignación de polos:** calcula PID, SPI y PPI a partir de ts y Mp, verifica márgenes y separación entre lazos, y simula.
  - **Enviar al ESP32** manda las ganancias, FRICI, FRICB, DITHER, GRAV (o la calibrada), FMAP e ILIMIT.
  - **Probar en la pata** hace 0 → A → 0 → −A → 0 más un barrido de todo el rango, y resume. Rangos por defecto: coxa −0.5…1.5, fémur −1…1, tibia −1…0.7.
- **Recorrido del espacio de trabajo:** barridos lentos con las otras articulaciones en varias posturas. Detecta puntos duros, enganches, topes, rebotes y barridos que no se completan.
  - **Aplicar mapa de fricción** calcula FMAP con el último recorrido.
  - **Calibrar gravedad** recorre 25 posturas del fémur y la tibia y ajusta GRAV y GRAV2. En cada postura, cada articulación sube y baja ±0.12 rad a 0.15 rad/s (`GC_A`, `GC_V`). La gravedad es el promedio de la corriente real de las dos pasadas, medida solo en movimiento y sin empujón activo; la mitad de la diferencia es la fricción. Se mide en movimiento porque, parada (HOLD), el firmware entrega solo la corriente del modelo de gravedad. Durante la calibración el fémur conserva su gravedad anterior para no caer. La tibia va sin gravedad y la sostiene el freno: su modelo viejo sin acoplar (GRAV 0.0503, θ0 −1.68) tiene el signo cambiado en las esquinas y la tiró hasta −1.09 rad en el segundo intento del 14-nov. El tercer intento (15:07) falló igual porque el navegador corría la versión anterior del dashboard. El cuarto intento (16:18, v3) no juntó datos del fémur: quedó 7 min trabado entre −0.75 y −0.82 rad.
  - Supuse que el GRAV del fémur (0.0932, θ0 −2.753) tenía el signo invertido y la v4 lo giraba medio giro. **Era al revés: el signo original es el correcto.** Con el fémur sin corriente (primer intento, gravedad apagada), por encima de 0 rad cae casi siempre hacia θ positivo (2452 muestras contra 60). Con el signo girado (`GRAV,0.0932,0.3883`), la gravedad lo levantaba a saltos hacia +1.1 rad. La v5 quita ese giro. No volver a "corregir" el signo sin medir antes hacia dónde cae.
  - Por debajo de 0 rad cae hacia los dos lados (momentos de inercia y *stick-slip*): ahí la gravedad es débil o el modelo no la representa bien.
  - El GRAV vieja de la tibia (θ0 −1.68) da casi 0 con la tibia horizontal: tampoco sirve.
  - **v6 (19-nov, pata modificada):** la tibia conserva durante la calibración el GRAV2 acoplado de la identificación del 19-nov (el GRAV simple sigue en 0), y la calibración guardada pasó a la clave `pata_gravcal_v2`, así que las de la pata vieja se ignoran.
  - La calibración escribe la versión en la primera línea y, antes de mover nada, confirma en la línea CFG que la tibia quedó con GRAV 0; si no, se detiene. No aplica el ajuste si R² < 0.5. Dura unos 6 min.
  - El mapa y la gravedad calibrada se guardan en el localStorage del navegador.
- Los CSV se descargan en *Descargas*. El usuario los mueve a `matlab/datos/`.

## Estado al 14-nov-2026

- **Ganancias:** solo se cambian desde el dashboard. Viven en la RAM y, desde el 21-nov, se guardan en la flash con *Guardar en la placa* (`NVS,SAVE`). Las de fábrica de los `.h` (8.832/80090, 0.0284/0.01392, 6.166/3192) no sirven para la pata.
- **Diseño vigente:** corriente ts = 15 ms, velocidad ts = 0.4 s sin sobrepaso, posición ts = 1.0–1.3 s solo P.
  - Coxa: PID 16.03/80162 · SPI 0.00547/0.0480 · PPI 2.084
  - Fémur: PID 17.47/87340 · SPI 0.0185/0.1072 · PPI 2.971
  - Tibia: PID 14.72/73607 · SPI 0.00956/0.1044 · PPI 1.772
- **Último resultado** (12-nov, escalones de 0.5 rad): llegada en 1.4–2.5 s de media, error ≤ 0.016 rad, sobrepaso ≤ 14 %.
- **Problema principal:** *stick-slip* por los bujes de PLA. La fricción para arrancar es 2–4 veces la de movimiento y cambia a lo largo del recorrido. A 0.3 rad/s las articulaciones pasan detenidas el 40–70 % del tiempo.
- **FRICB y DITHER probados en la pata (14-nov,** CSV en `matlab/datos/2026-11-14_fricb_dither/`, `_d10` = dither 10 %, `_d0` = sin dither; la d10 se hizo primero):
  - Coxa: dither 10 % la mejora (6/7 contra 4/7 llegadas; pegada 25 % contra 45 % del tiempo). Sin dither se queda pegada a 0.020 rad del objetivo.
  - Fémur: dither 10 % lo empeora al subir desde ángulos negativos (vaivén −0.93 ↔ −0.65 rad). Mientras está pegado, la referencia de corriente queda en ~0.03 A con Is = 0.11: probablemente le resta la gravedad sin calibrar.
  - Tibia: 7/7 en las dos, sin diferencia clara.
  - El escalón de la coxa a 1.40 rad falla siempre en ~1.30: tope o zona dura mecánica.
  - Con FRICB el sobrepaso bajó a ≤ 1 % (antes ≤ 14 %).
  - Por ahora: dither 10 % en la coxa, 0 en el fémur y la tibia.
- **`FRIC_LOOSE_RAD` (corregido y probado el 14-nov,** CSV en `matlab/datos/2026-11-14_loose/`): el empujón se reiniciaba con la vibración del encoder (1 cuenta en la ventana de 20 ms = 0.11 rad/s; 2 cuentas pasaban el umbral de 0.15 rad/s). Ahora, una vez pegada, solo cuenta como suelta (o como yendo hacia atrás) si además recorrió 0.01 rad desde donde se pegó.
  - Cortes del empujón sin que la articulación se moviera: de 420–490 a 11–13 por prueba (coxa y fémur) y de 231 a 3 (tibia).
  - Coxa (con dither 10 %): 7/8 escalones, tiempo medio 2.7 → 1.4 s. Fémur: 8/8 por primera vez, pegado 23 → 16 % del tiempo. Tibia: igual que antes.
  - Sin sobrepasos nuevos por el empujón. El único (tibia 0 → 0.5, 16 %) fue un salto por *stick-slip* sin empujón activo, seguido de 11 s de vaivén 0.49 ↔ 0.58 rad: en HOLD, la corriente de gravedad (~0.02 A) la levanta. Es la gravedad sin calibrar.
  - Lo que queda lento es todo subida contra la gravedad: fémur −0.5 → 0 y −0.9 → 0 (~7 s).
- **Primera calibración de gravedad (14-nov) fallida:** apagaba la gravedad y medía parada. Sin gravedad, el firmware suelta el fémur en REVERSING y cae, así que nunca se quedó quieto. Además, en HOLD la corriente es la del modelo (0 con la gravedad apagada). Se rehízo midiendo en movimiento (ver el panel). Con un ESP32 simulado en el navegador, el método nuevo recupera la gravedad exacta; todavía falta correrlo en la pata. Ajustar la gravedad con los recorridos del 12-nov no sirvió: R² entre 0.3 y 0.7, porque la fricción en movimiento (0.06–0.08 A) es del tamaño de la gravedad.
- **Los rebotes de la tibia** cerca de 0.8 y −0.9 rad los causaba la gravedad sin acoplar, no la mecánica. La calibración de gravedad quedó descartada mientras haya bujes de PLA (ver Pendiente, punto 2).
- **Configuración vigente (14-nov):** gravedad vieja; FRICB 1, 200 ms; dither 10 % a 80 Hz en la coxa y 0 en el fémur y la tibia; firmware con `FRIC_LOOSE_RAD`.

## Pendiente (en orden)

1. Correr los tests en PC (en este PC no hay gcc; `test_main_host.c` tiene el test nuevo de vibración del encoder sin correr).
2. ~~Calibrar gravedad~~ **Descartado el 14-nov** tras 5 intentos: con los bujes de PLA la fricción (0.06–0.11 A) es del tamaño de la gravedad y tapa la medida. Se queda la gravedad vieja (fémur `GRAV,0.0932,-2.7533`, tibia la de la identificación), que es la que dio 8/8 en el fémur. Retomarlo después de cambiar a rodamientos (punto 6), igual que el mapa de fricción.
3. Repetir la coxa en su rango amplio (antes salió de rango por el disparo al soltarse; ya corregido).
4. ~~Guardar ganancias y compensaciones en NVS~~ **Hecho el 21-nov** (`NVS,SAVE`, botón *Guardar en la placa*); se cargan solas al arrancar (línea `NVS,CONFIG,LOADED,mask=0x7`). El cero no se puede guardar tal cual, porque el encoder es incremental: se guarda la **pose de arranque** (`NVS,POSE`, botón *Guardar pose de arranque*). Al encender, el encoder empieza en esos ángulos, así que la pata tiene que estar en esa misma pose de reposo. Su precisión depende de que la pose sea repetible; la solución de fondo es un *homing* contra un tope mecánico. Guardado el 21-nov en la placa de COM6: las 3 configuraciones y la pose de arranque coxa 0, fémur +1.0098 (colgando) y tibia 0. Comprobado con un reinicio: el fémur arranca en +1.0098 y las ganancias vuelven.
5. Verificar la escala del encoder.
6. Mecánica: cambiar los bujes de PLA por rodamientos. Es la solución de fondo del *stick-slip*.
7. Tablas y gráficas para la tesis.

## Cuidado con

- **Git en este PC:** si un commit deja `.git/*.lock` u otros bloqueos, revisar que no haya git corriendo antes de borrarlos. El repositorio es FFrank456/Tesis_hexapodo, rama `main`.
- `pata_banco1/` es la versión vieja para el banco de pruebas en protoboard: no tiene los cambios de la MOD_DRV.
