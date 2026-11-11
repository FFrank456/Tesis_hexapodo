# ControlCurrentN20

**Prueba en placa — 29/09/2026:** corriente 10/20 mA eq., velocidad ±1 rad/s y dos objetivos de posición ensayados por COM9. El primer objetivo de posición alcanzó la banda; el segundo quedó inmóvil con referencia CS limitada a 30 mA eq. También se observó exceso de velocidad respecto a la referencia pequeña. Sin pérdidas DMA, final STOP confirmado. Es una comprobación parcial, no validación general de la sintonía. [Resultados y estado final](docs/prueba_cascada_20260929.md).

## Estado actual — 28/09/2026

**Posición implementada:** PI exterior **Kp=6.166, Ki=3192, Kd=0**, en cascada con velocidad y corriente. CurrentPID añade pestaña Posición, gráficas en rad/rad/s y CSV común v3. Límites iniciales: 2 rad/s, rampa de referencia 5 rad/s² y tolerancia 0.02 rad; inversión automática con espera de detención y llegada en coast vigilado. Compilado y probado en software, pendiente de validar estabilidad y desempeño en planta. [Activación, límites, protocolo y verificación](docs/control_posicion_20260928.md).

**Revisión 6: corriente representada únicamente por el promedio aritmético de cada bloque DMA de 200 muestras.** Se retiraron el filtro EMA, sus parámetros, estado, cálculos, campos seriales, curvas y columnas de los nuevos CSV. Se mantienen PWM de 10 kHz y ADC continuo de 200 kS/s, sin selección ON/OFF.

El ESP32 también procesa el encoder y transmite posición en rad y velocidad en rad/s. Los tres Processing visualizan las medidas recibidas; no convierten conteos ni calculan velocidades.

**El usuario se encarga del flasheo.** Esta revisión requiere cargar el firmware nuevo y usar los Processing actualizados. No se ha accionado el motor ni flasheado la placa durante la activación del PI. Los ensayos anteriores en lazo abierto no validan el nuevo controlador.

Se actualizaron las ganancias del PI de corriente a las de MATLAB para eje libre: **Kp=8.832, Ki=80090 y Kd=0**. Se implementó el PI exterior de velocidad: **Kp=0.0284, Ki=0.01392**, también del PDF de MATLAB. El arranque conserva modo OPEN, PWM cero y referencias cero. El PI interior regula CS media equivalente, no una medida calibrada de torque. La nueva cascada está compilada y probada en host; **no está validada en la planta**. La prueba histórica siguiente corresponde a las ganancias anteriores (504.9 / 7462).

**Prueba preliminar por serial del 23/09/2026:** en CW, con PWM máximo 40 %, siguió referencias de 10 y 20 mA equivalentes. A 40 mA quedó limitado por el techo PWM y sostuvo 29.85 mA; al bajar a 20 mA reguló 19.83 mA con el eje girando. Sin pérdidas DMA ni salida inesperada de PI. Terminó en STOP, referencia y PWM cero. Ganancias confirmadas por telemetría; no se flasheó. La placa tenía inicialmente otras ganancias, por lo que el protocolo v6 por sí solo no certifica el binario cargado. Véase el [informe y registros del ensayo PI](build-codex/pi-trial-20260923-205551.md). Sigue pendiente la validación con osciloscopio y en otros puntos de operación.

## Control PI habilitado

El cálculo está en [current_pi.h](main/current_pi.h) y su integración en [main.c](main/main.c). Kp está en puntos porcentuales PWM/A equivalente y Ki en puntos porcentuales PWM/(A equivalente·s). Se conserva el nombre `PID` en los comandos y la telemetría v6 de 59 campos, por compatibilidad.

- Realimentación: `current_mean_a`, media de las 200 muestras DMA, sin filtro adicional.
- Integración con el tiempo real entre bloques nuevos; una lectura repetida no vuelve a integrar. El control consulta el último bloque disponible: no garantiza procesar todos los bloques ni una frecuencia fija de 1 kHz. Las consultas observadas anteriormente rondaban 500–600 Hz; deben medirse de nuevo con el PI.
- Salida limitada, antiwindup condicional y transferencia desde PWM manual del mismo sentido conservando inicialmente la salida. Para invertir el sentido se requiere STOP.
- Una medida inválida, pérdida DMA o intervalo sin actualización de al menos 20 ms provoca STOP. La adquisición también invalida datos antiguos. Esto no sustituye una protección independiente contra sobrecorriente.
- Referencia admitida: 0–1 A equivalente CS. Este rango limita la consigna, no garantiza limitar la corriente física a 1 A. Referencia cero manda coast y borra el integrador.

En CurrentPID, **Enviar PI** envía las ganancias y el límite PWM mostrado. **Enviar ref** envía la referencia y habilita PI si estaba en OPEN; cambiarla durante el control conserva el integrador. El botón **PI** también aplica antes el límite mostrado (40 % inicial). El firmware por sí solo inicia con límite máximo 100 %; las órdenes seriales directas deben configurar el límite deseado mediante `LIMIT,0,maximo`.

Comandos disponibles: `PID,8.832,80090,0`, `LIMIT,0,40`, `IREF,valor_en_A_equivalentes`, `MODE,PID,CW` (o CCW), y `STOP`. Las ganancias están incorporadas como valores iniciales. Solo se admite Kd cero y parámetros numéricos finitos. STOP vuelve a OPEN y PWM cero; conserva las consignas almacenadas y borra ambos integradores.

## Cascada de velocidad implementada

`referencia rad/s -> PI velocidad -> referencia A eq. CS -> PI corriente -> PWM -> motor`

El firmware ejecuta ambos PI. Processing no calcula control ni velocidad. **ControlCurrentN20_CurrentPID** presenta pestañas **Corriente** y **Velocidad** que solo cambian la vista. **Aplicar referencia / activar** envía la referencia y activa el modo elegido; **Ajustes y diagnóstico** contiene ganancias y PWM manual. Las gráficas ocupan la mayor parte de la ventana. Al pasar de SPEED a PID se borra la referencia interior antes de aceptar una referencia manual.

- Velocidad de salida en rad/s, con ventana del encoder de 50 ms ya existente. El PI exterior se actualiza al disponer de al menos 10 ms nuevos del reloj del encoder; integra con el intervalo real. No se promete periodicidad exacta. Un hueco de 50 ms o feedback marcado inválido provoca STOP. Un encoder desconectado que siga reportando cero válido no se distingue de un eje quieto solo con esta prueba de validez.
- La referencia de velocidad es una **magnitud 0–32 rad/s**, con sentido separado CW/CCW. Se compara con velocidad medida proyectada en ese sentido. La gráfica conserva la velocidad firmada y presenta la referencia con el signo correspondiente.
- El PI exterior entrega entre cero y `ILIMIT` A equivalentes CS, inicialmente **0.100 A eq.**. El comando admite un máximo de 1 A eq.; limita la referencia, no constituye protección de corriente física. Processing mantiene el techo inicial **40 % PWM**, editable; el arranque del firmware mantiene techo interno 100 % hasta recibir LIMIT.
- Ambos PI tienen antiwindup condicional. Además, el exterior detiene la acumulación positiva si el PI interior está en el techo PWM con error de corriente positivo. Bajar ILIMIT recorta la referencia y ajusta el integrador exterior solo si estaba por encima del nuevo límite.
- No se manda corriente negativa ni frenado activo. Si el PI pide esfuerzo negativo se aplica cero/coast; la desaceleración depende de la planta. `WREF,0` manda coast inmediatamente y borra ambos integradores. Para invertir: STOP, esperar a que se detenga el eje y seleccionar el otro sentido. La activación SPEED rechaza giro opuesto medido mayor de 0.5 rad/s.
- Se mantiene la parada por problemas de la media DMA del PI interior. La estabilidad de las ganancias continuas al discretizarlas debe comprobarse en planta; las pruebas host verifican implementación, no márgenes de estabilidad físicos.

Nuevos comandos, además de los anteriores:

| Comando | Acción |
|---|---|
| `SPI,0.0284,0.01392` | Ganancias del PI exterior |
| `ILIMIT,0.1` | Máxima referencia de corriente generada por velocidad, A eq. |
| `WREF,5` | Referencia de velocidad de magnitud 5 rad/s |
| `MODE,SPEED,CW` | Activar cascada, con ambas medidas válidas |
| `MODE,SPEED,CCW` | Activar cascada en sentido contrario, desde eje detenido |
| `STOP` | OPEN, PWM cero y reinicio de ambos PI |

Durante SPEED, `IREF` se rechaza: la referencia interior pertenece al PI exterior. Los valores de referencia/ganancias no se guardan en flash; al reiniciar vuelven los valores iniciales. El envío de referencia durante el mismo modo no reinicia el PI. Todos los comandos numéricos de CurrentPID usan punto decimal independientemente del idioma de Windows.

**Telemetría:** DATA v6 conserva los 59 campos a 100 Hz nominales; los modos añaden `SPEED` (código 2) y `POSITION` (código 3). SPEED y POSITION se envían como extensiones independientes a 10 Hz nominales cada una. La primera amplía el diagnóstico de velocidad:

```text
SPEED,1,encoder_time_us,direction_sign,reference_magnitude_rad_s,error_rad_s,current_reference_a_eq,kp,ki,integral_a_eq,current_limit_a_eq,limited,control_time_us
```

Los otros Processing ignoran esta extensión. CurrentPID exige una trama SPEED v1 reciente para activar velocidad, grafica corriente y velocidad, y exporta estas variables junto con DATA. Cada fila conserva los timestamps de origen de SPEED: la extensión se mantiene entre recepciones, **no es una adquisición independiente a 100 Hz**. Las ganancias de corriente se conservan por muestra; `pwm_limit_max_percent` registra el último techo confirmado mediante `ACK,LIMIT`, y queda desconocido tras reconexión/reinicio hasta recibir otra confirmación.

La interfaz muestra los últimos 20 s y conserva hasta 60000 muestras para exportar (aproximadamente 10 min a 100 Hz). Al llenarse avisa, conserva las filas antiguas y sigue mostrando la planta en vivo. **Nuevo registro** borra la captura: exportar antes si se desea conservarla. El CSV v3 es común a corriente, velocidad y posición; conserva la telemetría, modo, unidades, referencia/medida/error genéricos, segmento temporal y métricas, y añade campos de posición. Se escribe en segundo plano por lotes, sin construir una tabla completa de diez minutos en memoria.

Indicadores: RMSE, error absoluto medio, error absoluto máximo y porcentaje de tiempo de PWM en el techo confirmado. Usan hasta 5 s de muestras válidas del modo/segmento actual, ponderadas por tiempo del ESP32; se excluyen huecos mayores de 100 ms y medidas inválidas. La cobertura válida se muestra junto a los indicadores. Son estadísticas de telemetría, no una medida exacta de todos los ciclos del control ni una certificación de estabilidad. Ver [interfaz y contrato CSV v2](docs/processing_dashboard_csv_v2.md).

Origen y límites del diseño: [revisión de MATLAB/Simulink](matlab/revision_cascada_20260927.md). Se implementan corriente y velocidad; posición sigue pendiente de corregir y validar la cascada del modelo. [Verificación de esta implementación](docs/control_cascada_20260928.md).

## Configuración

| Elemento | Configuración |
|---|---|
| Driver | DRV8876, PMODE=1: avance/coast y reversa/coast |
| PWM horario / antihorario | GPIO27 / GPIO26; consigna firmada de -100 a +100 % |
| PWM | 10 kHz; reloj de 20 MHz y período de 2000 ticks |
| CS al ADC | GPIO32, ADC1_CH4; 12 bits, atenuación de 12 dB |
| Adquisición | DMA continuo, 200000 muestras/s; 200 muestras por bloque |
| Encoder A/B | GPIO18 / GPIO19, PCNT x4 |
| Encoder mecánico | 7 pulsos/vuelta/canal, reductora 100:1; 2800 conteos/vuelta de salida |
| Serial | 460800 baudios; telemetría nominal de 100 Hz |

En cada sentido se aplica PWM a una entrada del puente y se mantiene la otra en cero. STOP coloca ambas entradas en cero: coast.

## Medida de corriente

La adquisición recibe un bloque cada 1 ms nominal: 5 us entre conversiones, aproximadamente 20 muestras por período PWM y diez períodos por bloque. Todas las muestras del bloque intervienen, incluidas ON, OFF y STOP.

Cada código se convierte mediante la tabla de voltajes calibrados del SDK:

```text
V_procesado[k] = 0                       si raw[k] == 0
V_procesado[k] = V_calibrado_SDK(raw[k])  si raw[k] > 0

adc_raw_mean     = suma(raw[k]) / 200
adc_voltage_mean = suma(V_procesado[k]) / 200
current_mean_a   = adc_voltage_mean / 3.0   (SENSE_CS_VOLTS_PER_AMP)
```

La conversión se aplica a cada muestra antes de promediar. No hay filtro recursivo ni memoria entre bloques. Cada promedio utiliza exclusivamente las 200 muestras de su bloque; un cambio de PWM puede quedar dentro de ese intervalo.

El voltaje SDK de la última muestra se conserva como diagnóstico en `adc_voltage`; puede ser 0.142 V con raw cero aunque su contribución al promedio sea cero. No se resta 0.142 V a los códigos positivos. La asignación de cero no recupera información bajo el rango útil del ADC ni demuestra corriente física nula.

Solo un bloque completo válido publica sus medias. Saturación, error de calibración, datos mal formados, antigüedad excesiva o pérdida DMA producen medias NaN y `dma_block_samples=0`. Processing muestra huecos y no reutiliza una medida antigua como válida. Las pérdidas DMA reinician las colas y provocan STOP desde la tarea de control.

`current_mean_a` representa **CS promedio escalado con 3.0 V/A** (MOD_DRV: R_IPROPI = 3 kΩ × 1000 µA/A; la placa anterior usaba 2.49 V/A). La calibración ADC y la ganancia física CS/corriente siguen pendientes. En avance/coast, IPROPI no observa toda la corriente de recirculación: esta magnitud no equivale automáticamente a la corriente media del devanado ni al torque. Referencia: [DRV8876, sección 7.3.3.1](https://www.ti.com/lit/ds/symlink/drv8876.pdf).

El cálculo de bloques es interno, nominalmente a 1 kHz. La telemetría a 100 Hz envía el último resumen: **no transmite las 200 muestras ni registra todos los bloques de 1 ms**. No debe confundirse la tasa ADC, la consulta de medidas y la frecuencia del CSV.

## Encoder calculado en ESP32

Los parámetros están en [encoder.h](main/encoder.h), el cálculo en [encoder_processing.h](main/encoder_processing.h) y la lectura PCNT en [encoder.c](main/encoder.c).

```text
Conteos/vuelta del motor = 7 * 4 = 28
Conteos/vuelta de salida = 28 * 100 = 2800
Posición [rad] = (conteo - referencia) * 2*pi / 2800
Velocidad [rad/s] = delta_conteos * 2*pi / (2800 * delta_t_s)
```

El factor x4 ya incluye ambos canales. Se conserva el signo, la posición puede superar una vuelta y el contador se extiende a 64 bits. Las diferencias se calculan como enteros antes de convertirlas a radianes.

La velocidad usa una ventana móvil de al menos 50 ms, con su duración real en `encoder_speed_window_us`. La lectura PCNT lleva su propio `encoder_time_us`, independiente del tiempo del bloque ADC y del puerto serial. Sin ventana suficiente se transmite NaN y `encoder_speed_valid=0`; un hueco superior a 100 ms reinicia el historial. Esta ventana introduce retardo que debe considerarse al identificar la planta.

Cada `SEQ` establece la referencia de posición antes del primer escalón. El botón **Cero encoder ESP32** envía `ENCODER,ZERO`, confirmado por `ACK,ENCODER,ZERO`. Cambiar la referencia no borra el conteo absoluto ni el historial de velocidad. El comando se rechaza durante una secuencia activa. StepExperiment interrumpe la captura si cambia `encoder_reference_id` durante el ensayo.

Los CSV guardan rad y rad/s exactamente como llegan del ESP32. Para identificar posición/velocidad debe usarse `encoder_time_us`; `sample_time_us` corresponde al bloque de corriente.

## Processing y protocolo serial v6

Los tres sketches requieren **DATA de 59 campos**, contando `DATA` como índice cero, y `sensing_version=6`. Las tramas antiguas o incompletas se rechazan con un aviso. Se eliminaron tres campos respecto a v5, por lo que cambiaron los índices; debe actualizarse firmware y visualizador conjuntamente.

| Índice | Campo | Significado |
|---|---|---|
| 1 | `time_ms` | Tiempo del bloque de corriente en ms |
| 2–4 | `adc_raw`, `adc_voltage`, `current_raw_a` | Última muestra del bloque, solo diagnóstico |
| 10–11 | `pwm_frequency_hz`, `adc_sample_rate_hz` | Frecuencias nominales PWM y ADC |
| 18 | `duty_percent` | PWM firmado aplicado |
| 24 | `sample_time_us` | Recepción ISR del bloque ADC; no apertura exacta |
| 25 | `encoder_count` | Conteo absoluto x4 |
| 26–29 | Campos de secuencia | Identificador, etapa, inicio y comienzo de etapa |
| 30–31 | `current_valid`, `current_status` | Validez y estado del bloque |
| 37–39 | Tasas observadas | PWM, consulta y conversiones ADC |
| 40 | `dma_overflows` | Pérdidas DMA acumuladas |
| 41 | `sensing_version` | 6 |
| 42–44 | Descartes y fase | Descartes por ventana/calidad y validación de fase |
| 45 | `adc_raw_mean` | Media de cuentas ADC |
| 46 | `adc_voltage_mean` | Media de voltajes procesados, V |
| 47 | `current_mean_a` | Medida principal: media CS / 3.0, A equivalentes |
| 48 | `dma_block_samples` | 200 válido; 0 inválido |
| 49 | `encoder_version` | 1 |
| 50 | `encoder_time_us` | Tiempo de lectura PCNT |
| 51 | `encoder_relative_count_x4` | Conteo relativo firmado |
| 52 | `output_position_rad` | Posición relativa de salida, rad |
| 53 | `output_speed_rad_s` | Velocidad firmada de salida, rad/s |
| 54–55 | Validez y ventana de velocidad | 0/1 y duración real en us |
| 56 | `counts_x4_per_output_rev` | 2800 |
| 57–58 | Referencia del encoder | Conteo absoluto de origen e identificador |

La línea `HEADER` describe todos los campos en su orden exacto. Estado 9 o 15 y `current_valid=1` indican bloque procesado válido; no certifican calibración física. Estado 15 identifica cero asignado en la última muestra y no implica que todo el bloque sea cero. No se utiliza fase PWM: `sample_phase_us=-1`, `dma_phase_validated=0` y `dma_rejected_window=0`.

- [Processing manual](processing/ControlCurrentN20_Processing4/ControlCurrentN20_Processing4.pde): control PWM, media DMA de corriente y gráfica del PWM enviado de 0 a 1.
- [CurrentPID](processing/ControlCurrentN20_CurrentPID/ControlCurrentN20_CurrentPID.pde): corriente, velocidad y posición en cascada; gráficas, métricas y CSV v3. Ganancias iniciales de corriente 8.832/80090, velocidad 0.0284/0.01392 y posición 6.166/3192; Kd=0.
- [StepExperiment](processing/ControlCurrentN20_StepExperiment/ControlCurrentN20_StepExperiment.pde): secuencia 0→100→0 %, pasos de 5 % cada segundo; 41 etapas, máximo 48 y 120 s. Gráficas de corriente media, PWM y velocidad recibida; CSV con medias DMA y encoder SI.

## Verificación y uso

Binario generado en `build-codex/ControlCurrentN20.bin`. La activación PI se documenta en [verificación PI](build-codex/pi-activation-20260923.md); la [verificación v6](build-codex/dma-mean-v6-verification.md) corresponde a la etapa anterior de sensado sin controlador habilitado.

Después del flasheo a cargo del usuario:

1. Abrir uno de los Processing actualizados y conectar a 460800 baudios.
2. Confirmar recepción v6, ADC cercano a 200 kS/s, PWM cercano a 10 kHz y ausencia de pérdidas.
3. Comparar `adc_voltage_mean` con Mean de CS en el osciloscopio, con escala de sonda y acoplamiento DC correctos y ventanas equivalentes.
4. Comprobar 2800 conteos y 2*pi rad por vuelta de salida, signo y velocidad.
5. Validar calibración y temporización antes de identificar la planta y habilitar el lazo.

## Registros anteriores

El [ensayo del 21/09/2026](processing/ControlCurrentN20_StepExperiment/exports/multistep_20260921_093322.csv) corresponde a v5: 4100 registros, bloques válidos de 200 muestras y ningún desbordamiento registrado. La [comparación al 40 %](build-codex/dma-mean-active40-20260918-215537.md) obtuvo 63.51 mV del ESP32 frente a 60.56 mV del osciloscopio en ventanas no simultáneas; es una comprobación preliminar.

Los CSV e informes históricos conservan sus datos originales. Las estrategias y resultados anteriores están en el [historial](README_HISTORIAL.md) y en el [estado v5 archivado](docs/history/README_v5_20260921.md); no describen la revisión activa.
