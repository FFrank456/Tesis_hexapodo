# pata_banco1

Control en cascada (corriente → velocidad → posición) de los **3 motores N20 de una pata** — Coxa, Fémur y Tibia — **funcionando a la vez** en un solo ESP32 DevKit V1 con 3 módulos DRV8876 (MJKDZ). Es una extensión de `ControlCurrentN20` / `ControlCurrentN20_M2Femur`: cada motor tiene exactamente los mismos lazos, ganancias iniciales y protecciones que el proyecto de un motor.

Pines tomados del repositorio del compañero (`flavioe-alarcon/hexapodo-secundario-firmware`, `AGENTS.md` y `CONTEXTO_TESIS.md` §8.1). Todo el mapeo está en [`main/pin_map.h`](main/pin_map.h).

## Conexiones

| Señal | M1 Coxa | M2 Fémur | M3 Tibia |
|---|---|---|---|
| EN/IN1 (PWM, sentido CW) | GPIO21 | GPIO18 | GPIO16 |
| PH/IN2 (PWM, sentido CCW) | GPIO5 | GPIO17 | GPIO4 |
| CS / IPROPI | GPIO36 (VP) | GPIO39 (VN) | GPIO34 |
| Canal ADC1 | CH0 | CH3 | CH6 |
| Encoder A | GPIO32 | GPIO25 | GPIO27 |
| Encoder B | GPIO33 | GPIO13 | GPIO14 |

En **cada** módulo DRV8876:
- **SLEEP y PMODE** van a 3.3 V del ESP32, con cables firmes. Si SLEEP queda flotando, el driver duerme y la corriente marca 0 A.
- **VIN/VM** van a +12 V, y el **GND es común** con el ESP32.
- **FAULT, VREF e IMODE** quedan sin conectar.
- **Placa roja:** OUT1 y OUT2 están invertidos en la serigrafía y no tiene protección de polaridad.
- **Resistencia IPROPI:** mide CS–GND en cada módulo desenergizado. El firmware asume 2.49 kΩ (2.49 V/A); si mide otra cosa, cambia `CS_R_IPROPI_OHMS`.

Motores N20 1:100 con encoder de 7 PPR → 2800 cuentas x4 por vuelta de salida (`encoder.h`).

GPIO5 es pin de arranque: durante el boot puede quedar en alto unos milisegundos antes de que el firmware fuerce las salidas a 0, lo que puede dar un tirón breve en la Coxa.

## Arquitectura

| Bloque | Implementación |
|---|---|
| PWM | MCPWM grupo 0: 1 timer a 10 kHz (20 MHz / 2000 ticks), 1 operador por motor con 2 generadores (IN1/IN2). |
| Reloj de control | Evento *timer empty* del MCPWM → tarea de control cada 10 periodos (1 kHz). |
| Corriente | ADC1 DMA continuo, patrón CH0/CH3/CH6 a 192 kS/s en total (64 kS/s por canal), porque el SAR del ESP32 en modo DMA no convierte más rápido que ~200 kS/s. Media de 64 muestras por canal cada 1 ms. Las muestras se asignan por el campo de canal, no por posición; se toleran bloques desde 58 muestras. Saturar un canal solo invalida ese motor. |
| Encoders | 3 unidades PCNT x4 con pull-up interno y filtro de glitches de 1 µs. |
| Control | Por motor: PI corriente (1 kHz) ← PI velocidad (100 Hz) ← PI posición (50 Hz). Misma lógica que el proyecto de un motor. |
| Fallos | Cada motor se detiene solo ante su propio fallo de encoder o lazo. Una pérdida de DMA (compartido) detiene a los tres. |

CPU a 240 MHz y pila de `app_main` de 6144 B (ya ajustados en `sdkconfig`).

## Protocolo serial (460800 baud)

- **Motor objetivo:** se antepone `M1,`, `M2,` o `M3,` (también `COXA,`, `FEMUR,`, `TIBIA,`). La respuesta lleva el mismo prefijo, por ejemplo `M2,SET,30` → `M2,ACK,SET,30.000`.
- **`ALL,<comando>`** lo aplica a los tres. Ejemplos: `ALL,ENCODER,ZERO`, `ALL,PID,8.832,80090,0`.
- **Sin prefijo**, el comando va al motor seleccionado con **`SEL,1|2|3`** (al arrancar, `DEFAULT_SELECTED_MOTOR` = Fémur). Así el dashboard Processing de un motor sigue funcionando.
- **`STOP` sin prefijo detiene los 3.** `M2,STOP` detiene solo uno.
- **`TEL,LEGACY`** (modo al arrancar): `DATA` v6, `SPEED` y `POSITION` del motor seleccionado, idénticos al proyecto de un motor.
- **`TEL,MULTI`**: una línea `T` a 100 Hz con los 3 motores, más una línea `CFG` por motor (~3 Hz) con sus ganancias y límites.
- **`INFO`** imprime el mapa de pines y el signo de encoder de cada motor.
- **`DIAG`** imprime el estado del ADC (tramas, muestras por canal, raw min/máx/medio, estado) y de cada encoder.
- **`Mx,ESIGN,1|-1`** invierte el sentido de conteo del encoder de ese motor (solo en STOP; invalida el `PREF`).

Comandos por motor, iguales que en el proyecto de un motor:

| Comando | Qué hace |
|---|---|
| `SET,%` | Lazo abierto |
| `CTRL,CW\|CCW,A` | Lazo de corriente |
| `IREF` | Referencia de corriente |
| `PID,kp,ki,0` | Ganancias del PI de corriente |
| `LIMIT,min,max` | Techo de PWM |
| `MODE,SPEED,CW\|CCW` | Lazo de velocidad |
| `WREF` | Referencia de velocidad |
| `SPI` | Ganancias del PI de velocidad |
| `ILIMIT` | Límite de corriente |
| `PREF,rad` | Referencia de posición |
| `MODE,POSITION` | Lazo de posición |
| `PPI` | Ganancias del PI de posición |
| `PLIMIT,vmax,acel,tol` | Límites de posición |
| `ENCODER,ZERO` | Cero del encoder |
| `SEQ,%:ms;...` | Secuencia de PWM |
| `DIR` | Sentido |

Línea `T`: `T,time_ms,selected` seguido de 11 campos por motor:

```
mode, duty_percent, current_a, current_ref_a, position_rad, speed_rad_s,
speed_ref_rad_s, position_ref_rad, position_state, current_status, flags
```

- **mode:** 0 OPEN, 1 PID (corriente), 2 SPEED, 3 POSITION.
- **flags:** bit 1 = PREF válido, bit 2 = velocidad válida, bit 4 = secuencia activa.

## Uso

1. **Compilar y flashear:** `idf.py -p COM15 build flash`, o "Build, Flash" en VS Code (ESP-IDF v5.4.4).
2. **Dashboard de 3 motores:** abre [`dashboard/pata_banco1_dashboard.html`](dashboard/pata_banco1_dashboard.html) en **Chrome o Edge** → **Conectar** → elige COM15. El dashboard envía `TEL,MULTI` por su cuenta.
   - Para cada articulación tiene modos Posición, Velocidad, Corriente y PWM, más STOP, Cero e Invertir encoder.
   - Los gráficos muestran medida contra referencia y el PWM.
   - **STOP TODO** también se activa con la barra espaciadora.
   - **Grabar** → **Exportar CSV** guarda los 3 motores a 100 Hz.
   - El Monitor Serie debe estar cerrado: el puerto solo admite un programa a la vez.
3. **Dashboard Processing de un motor:** conéctalo normalmente; el ESP32 arranca en LEGACY con el motor `DEFAULT_SELECTED_MOTOR` de `pin_map.h` (Fémur). Para usar Processing con otro motor, cambia ese valor y reflashea: al abrir el puerto, el ESP32 suele reiniciarse y vuelve al valor por defecto, así que un `SEL` enviado antes se pierde.

**Primera prueba recomendada:**
1. Cada motor en PWM ±30 %, para verificar el sentido y que la posición crece con PWM positivo. Si no crece, usa **Invertir encoder**.
2. **Cero los 3**.
3. Posición con objetivos pequeños (0.3–0.5 rad).

## Pruebas en PC (sin hardware)

```
gcc -std=gnu11 -Itests/host_stubs -Imain tests/test_main_host.c main/pwm_sequence.c -lm -o t && ./t
gcc -std=c11 -Imain tests/test_dma_sense.c -lm -o t && ./t
gcc -std=c11 -Imain tests/test_encoder.c -lm -o t && ./t        (y test_current_pi / test_speed_pi / test_position_pi)
python tests/test_serial_schema.py
```

- **`test_main_host`:** ruteo M1/M2/M3/ALL/SEL, STOP global, ESIGN, pines MCPWM, cascadas independientes con una planta simulada y aislamiento de fallos.
- **`test_dma_sense`:** ADC de 3 canales (ruteo por canal, saturación por canal, pérdidas y datos viejos).

## Pendiente de validar en hardware

- Lectura de CS con 3 canales, por posible diafonía entre canales del multiplexor. El comando `DIAG` (o el botón **Diagnóstico** del dashboard) muestra, por canal, cuántas muestras llegaron, el raw mínimo/máximo/medio y el estado.
- Signo de cada encoder.
- Ganancias: se identificaron con otro motor y otro módulo, con R_IPROPI de 3 kΩ.
