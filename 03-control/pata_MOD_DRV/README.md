# pata_MOD_DRV

Control en cascada (corriente → velocidad → posición) de los **3 motores de una pata** — Coxa, Fémur y Tibia — **funcionando a la vez** en la placa propia **MOD_DRV** (proyecto Altium `Moduloa_DRV`, en `02-electronica/unidad2-MOD_DRV`): ESP32-WROOM-32E + 3 DRV8876 en chip.

Comparte el control de motores con `pata_banco1` (el banco de pruebas en galleta con ESP32 DevKit y módulos DRV8876). `main/pin_map.h` define para la MOD DRV:
- los pines,
- la resistencia IPROPI de 3 kΩ (3.0 V/A),
- el nFAULT de cada driver, que aquí sí está conectado.

Pines tomados de `02-electronica/notas/Modulo de control de extremidad - Registro del proyecto.md` §4.4, extraídos de la netlist compilada.

## Conexiones (fijas en la placa)

| Señal | DRVA = M1 Coxa | DRVB = M2 Fémur | DRVC = M3 Tibia |
|---|---|---|---|
| IN1 (PWM, sentido CW) | IO26 | IO18 | IO16 |
| IN2 (PWM, sentido CCW) | IO27 | IO19 | IO2 (pin de arranque) |
| IPROPI (R1 = 3 kΩ) | IO32 · ADC1_CH4 | IO33 · ADC1_CH5 | IO34 · ADC1_CH6 |
| nFAULT (R4 = 10 kΩ a 3V3) | IO5 | IO14 | IO15 (pin de arranque) |
| Encoder (J3 pin 4) | IO35 (solo entrada, sin pull-up) | IO23 | IO4 |
| Encoder (J3 pin 6) | IO25 | IO17 | IO13 |
| Conector | J3A | J3B | J3C |

**Asunción:** DRVA (J3A) = Coxa, DRVB (J3B) = Fémur, DRVC (J3C) = Tibia. Si en la pata se conecta distinto, reordena las columnas en `pin_map.h`.

- **Configuración fija en la placa:** PMODE, nSLEEP y VREF van a 3V3. El driver queda siempre activo, en modo PWM, con corte en $I_{TRIP} = 3.3 / (3\,\text{kΩ} \cdot 1000\,\mu\text{A/A}) = 1.1$ A.
- **Rango del ADC:** con atenuación de 12 dB llega a ~3.1 V, es decir ~1.03 A medibles.
- **Encoder A de la Coxa en IO35:** IO35 no tiene pull-up interno, así que el firmware no intenta activarlo (`GPIO_IS_VALID_OUTPUT_GPIO`). El encoder de J3A debe tener salida push-pull o un pull-up externo.
- **Bus CAN (J4):** IO21/IO22 quedan para el módulo CAN; este firmware todavía no los usa.
- **Antes de enchufar un motor:** comprueba que el cable siga el orden del conector PicoBlade.

  | Pin | 1 | 2 | 3 | 4 | 5 | 6 |
  |---|---|---|---|---|---|---|
  | Señal | OUT1 | OUT2 | GND | Encoder | 3V3 | Encoder |

Motores Pololu HPCB 12 V 100:1:
- **Encoder:** 7 PPR por canal → 2800 cuentas x4 por vuelta de salida (`encoder.h`).
- **Corriente:** ~80 mA en vacío y ~0.75 A con el eje bloqueado. Usa `ILIMIT` de 0.3–0.5 A; el driver no corta hasta 1.1 A.

## nFAULT

Cada DRV8876 tiene su propio nFAULT:
- **Detección:** si el pin está en bajo durante `DRV_NFAULT_FILTER_MS` (3 ms seguidos) mientras el motor está activo, se detiene **solo ese motor**.
- **Aviso:** se imprime `Mx,ERR,DRV_NFAULT,STOP`.
- **Telemetría:** en la línea `T` se activa el bit 8 de `flags`. El dashboard marca el motor en rojo con "nFAULT".
- **Causas posibles:** sobrecorriente, sobretemperatura o subtensión de V_MOT.
- **Sin V_MOT:** si solo conectas el USB, los drivers quedan apagados y el pull-up R4 mantiene nFAULT en alto, así que no salta.

## Lo demás es igual que pata_banco1

- **Arquitectura:** MCPWM de 10 kHz con un operador por motor; ADC DMA de 3 canales a 64 kS/s por canal con media cada 1 ms; 3 PCNT x4; PI corriente (1 kHz) ← PI velocidad (100 Hz) ← PI posición (50 Hz) por motor.
- **Fallos:** cada motor se detiene solo ante su propio fallo.
- **Protocolo serial (460800 baud):**
  - prefijos `M1,` / `M2,` / `M3,` / `ALL,`;
  - `STOP` sin prefijo detiene los 3;
  - `SEL,1|2|3`;
  - `TEL,LEGACY|MULTI`, `INFO`, `DIAG`;
  - `Mx,ESIGN,±1`;
  - los mismos comandos por motor que `ControlCurrentN20` (`SET`, `CTRL`, `PREF`, `MODE`, `WREF`, `ILIMIT`, `LIMIT`, `PID`, `SPI`, `PPI`, `PLIMIT`, `ENCODER,ZERO`, `SEQ`, `DIR`).
- **`INFO`** muestra la placa (`INFO,BOARD,pata_MOD_DRV`) y, por motor, los pines y el nFAULT. `DIAG` añade `nfault_low`.

## Mejoras del control de posición

| Cambio | Antes | Ahora | Comando |
|---|---|---|---|
| **Retención con freno** | Al llegar a la banda de tolerancia o al invertir sentido, el motor quedaba suelto (IN1 = IN2 = 0) | IN1 = IN2 = 1: el DRV8876 frena por los transistores del lado bajo, la fuerza contraelectromotriz detiene el eje y resiste que lo muevan. El dashboard muestra "FRENO" (bit 16 de `flags`) | `Mx,HOLD,BRAKE` (por defecto) o `Mx,HOLD,COAST` |
| **Velocidad más rápida** | Ventana de 50 ms (~25 ms de retraso) | Ventana de 20 ms (~10 ms de retraso). Resolución: 0.11 rad/s por cuenta | `ENCODER_SPEED_MIN_WINDOW_US` en `encoder.h` |
| **Inversión de sentido** | Esperaba 100 ms detenido; "detenido" = menos de 0.1 rad/s | Espera 20 ms; "detenido" = menos de 0.3 rad/s (1 cuenta en la ventana de 20 ms ya da 0.11 rad/s) | `POSITION_REVERSE_QUIET_US`, `POSITION_STOP_RAD_S` en `position_pi.h` |
| **Compensación de fricción** | — | Se suma un % de PWM a la salida del lazo de corriente, solo en Velocidad y Posición | `Mx,FRIC,pct` (0–40 %, por defecto 0) |
| **Compensación de gravedad** | — | Se suma `amps·cos(posición + θ₀)` a la referencia de corriente, proyectado al sentido activo (se resta cuando la gravedad ayuda) | `Mx,GRAV,amps,θ₀` (por defecto 0,0) |

`STOP` sigue dejando el motor suelto. El freno solo actúa dentro del modo Posición.

### Retención activa contra la gravedad

El freno resiste el movimiento, pero no sostiene un peso quieto: con la pata estirada, el fémur se deslizaba hacia atrás, el lazo volvía a empezar y tardaba segundos en juntar corriente. Con `GRAV` distinto de 0, la cascada pasa a usar **corriente con signo**:

$$I = \pm I_{vel} + GRAV \cdot \cos(\text{posición} + \theta_0)$$

- **Al llegar** (HOLD/ARRIVING) **o con la referencia de velocidad en 0:** la corriente es solo la de gravedad, en el sentido que sostiene el eslabón. El dashboard muestra "SOSTIENE" (bit 32 de `flags`). No hay caída ni rearranque, y el integrador de corriente no se reinicia.
- **Al subir:** se suman la corriente de velocidad y la de gravedad.
- **Al bajar:** se pide **menos** corriente de sostén; el lazo de velocidad sigue frenando y la gravedad hace el trabajo. Antes el motor quedaba suelto y la pata caía.
- **Si el sentido de la corriente se invierte:** el firmware frena 1 ms (corta antes de conectar) y reinicia el integrador de corriente.
- **Sin `GRAV`** (0): todo funciona como antes (freno al llegar).

Resultado de la simulación en PC (`test_main_host`, planta con gravedad equivalente a 0.09 A):

| | Sin GRAV | Con GRAV = 0.09 A |
|---|---|---|
| Re-entradas a TRACK en 4 s | 56 (cae y rearranca) | 0 |
| Error tras llegar | no llega a quedarse | 0.014 rad |
| Velocidad máxima al bajar (vmax 2 rad/s) | — | 1.86 rad/s |

**Cómo ajustar cada articulación:**
1. **Fricción:** en modo PWM, sube `SET` poco a poco desde 0 hasta que el eje apenas empiece a girar. Anota ese % (por ejemplo 12 %) y usa ~70 % de ese valor en `FRIC` (por ejemplo 8). Si el motor oscila alrededor del objetivo, bájalo.
2. **Gravedad** (fémur y tibia):
   1. Pon el eslabón **horizontal** y haz **Cero encoder** en esa postura; así cos(0) = 1 donde la gravedad pesa más y θ₀ = 0.
   2. En modo Corriente, sube `CTRL` hasta que el eslabón se sostenga solo: esa corriente es `amps`.
   3. El signo es positivo si la gravedad lo empuja hacia posiciones negativas. Si al activarlo la pata se cae más rápido, cambia el signo.
   4. Si haces cero en otra postura, θ₀ es el ángulo que falta para llegar a horizontal.
3. **PI de posición:** prueba primero solo P, con `Mx,PPI,4,0`, y sube Kp de 4 a 8. El lazo de velocidad ya tiene integrador. El dashboard tiene campos Kp/Ki y **Aplicar PI pos** (en STOP).

El dashboard tiene, en la pestaña Posición de cada motor: Kp/Ki de posición, "Al llegar" (Freno/Suelto), Fricción, Gravedad y θ₀, con el botón **Aplicar compensación**.

## Uso

1. **Compilar y flashear:** **ESP-IDF: Build, Flash** (v5.4.4). En la MOD_DRV el autoprograma por DTR/RTS del CP2102N hace el reset solo; si se queda en `Connecting...`, mantén **BOOT** (SW2).
2. **Dashboard:** cierra el monitor y abre `dashboard/pata_dashboard.html` en Chrome o Edge (perfil personal, no el administrado) → **Conectar**. El dashboard lee los pines y el nombre de la placa del ESP32 (`INFO`), así que sirve igual para el banco.
3. **Primera prueba:**
   1. Alimenta V_MOT (12 V) en J1.
   2. Prueba cada motor en PWM ±30 % y comprueba que la posición crece con PWM positivo. Si no crece, usa **Invertir encoder**.
   3. Pulsa **Cero los 3**.
   4. Prueba posición con objetivos de 0.3–0.5 rad, `ILIMIT` de 0.3 A y techo de PWM de 70 %.

## Pruebas en PC (sin hardware)

```
gcc -std=gnu11 -Itests/host_stubs -Imain tests/test_main_host.c main/pwm_sequence.c -lm -o t && ./t
gcc -std=c11 -Imain tests/test_dma_sense.c -lm -o t && ./t
```

- **`test_main_host`:** pines de la MOD_DRV, `INFO`, 3.0 V/A, filtro y aislamiento del nFAULT (solo se detiene el motor afectado), cascadas independientes, ruteo de comandos, freno al llegar (ambas salidas en alto), HOLD/FRIC/GRAV.
- **`test_position_fast_reverse`:** inversión con 20 ms de espera y umbral de 0.3 rad/s. `test_position_pi` y `test_encoder` mantienen sus casos originales fijando los valores antiguos (100 ms y 50 ms).
- **`test_dma_sense`:** patrón CH4/CH5/CH6.

## Pendiente de validar en hardware

- Que DRVA/DRVB/DRVC correspondan a coxa, fémur y tibia en la pata real.
- El tipo de salida del encoder en IO35: sin pull-up interno, un encoder open-drain no contaría.
- El signo de cada encoder.
- Las ganancias en esta placa, con los motores de la pata.

## Prueba de control por CAN

El ESP32-S3 puede mandar consignas de posición a la MOD DRV. El cableado, los comandos de prueba y el formato de las tramas están en la [guía de la prueba CAN](../prueba_can/README.md).
