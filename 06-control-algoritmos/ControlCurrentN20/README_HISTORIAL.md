# Historial del proyecto

Este documento conserva decisiones anteriores. El estado vigente está en [README.md](README.md).

- 29/09/2026: pruebas simples autorizadas en COM9 con ganancias actuales. Corriente y giro en ambos sentidos observados; posición CW dentro de tolerancia, segundo objetivo CCW no alcanzado con límite CS de 30 mA eq. Sin pérdidas DMA, final STOP confirmado. Sin flasheo. Ver docs/prueba_cascada_20260929.md.

- 28/09/2026: lazo exterior de posición 6.166/3192 implementado en ESP32; referencias rad, rampa y límites, espera de inversión y llegada coast. CurrentPID incorpora Posición y CSV v3; extensiones SPEED/POSITION a 10 Hz nominales. Compilación, pruebas host/Java y revisión visual correctas. Sin flasheo ni ensayo físico. Ver docs/control_posicion_20260928.md.

- 28/09/2026: ganancias MATLAB de corriente libre 8.832/80090; PI exterior de velocidad 0.0284/0.01392 y adaptación de CurrentPID. Compilado y probado en host, sin flasheo ni validación de esta cascada en planta. Ver docs/control_cascada_20260928.md.

- [Estado v5 y resultados previos al retiro del filtro, 21/09/2026](docs/history/README_v5_20260921.md).
- [Copia antigua de pruebas FuncionalTEST1, no compilada](docs/history/FuncionalTEST1_legacy.txt).
- La revisión 6 usa únicamente la media DMA y requiere el protocolo de 59 campos.

# Revision DMA con clasificador temporal (reemplazada el 15/09/2026)

# ControlCurrentN20

## Estado del proyecto — 15 de septiembre de 2026

**Revision DMA confirmada en placa y ensayada por serial el 15/09/2026 a las 10:28; fase ON y calibracion fisica pendientes.** Se recibio `sensing_version=2` y se ejecuto una secuencia PWM de nueve etapas, terminada en STOP. La compilacion se verifico antes; esta prueba no incluyo flasheo. El usuario confirmo que el motor giro, pero el encoder solo registro 0..1 conteos: la lectura de movimiento es incorrecta y requiere revisar los canales A/B y su configuracion.

[Resultados y limitaciones de la prueba serial](build-codex/dma-serial-probe-20260915-102838-analysis.md): ADC aproximadamente 200 kS/s sin desbordamientos; corriente disponible a 100%, estado 12 en PWM parcial. La tarea de consulta bajo a unos 800 Hz y acumulo 2418 notificaciones omitidas; pendiente investigar.

| Area | Estado vigente |
| --- | --- |
| PWM manual | Consigna -100..100%, positivo horario, negativo antihorario, cero STOP; portadora nominal 10 kHz. |
| ADC | DMA continuo a 200 kS/s nominales, GPIO32/ADC1_CH4, 12 bits y atenuacion 12 dB. Tasa observada por contadores: aproximadamente 200 kS/s en la prueba serial; temporizacion fisica pendiente. |
| Seleccion ON | Implementada como clasificador temporal, **deshabilitada para PWM parcial** hasta medir desfase e incertidumbre. `SENSE_DMA_PHASE_VALIDATED=0`. |
| Corriente y EMA | Cada muestra aceptada entra al EMA; se conserva su memoria entre escalones, OFF y errores. No se selecciona el maximo por bloque. |
| Cableado | Se conserva: GPIO27/GPIO26 al driver, CS a GPIO32 y encoder A/B en GPIO18/GPIO19. No se agregan puentes. |
| Processing | Manual, CurrentPID y StepExperiment actualizados y compilados; muestran diagnosticos y admiten DATA de 48 campos. |
| Ensayos PWM | Secuencias ejecutadas por ESP32, hasta 48 etapas y 120 s. Ejemplo 0..100..0% en pasos de 5% cada segundo (41 etapas). |
| Encoder | Conteos PCNT x4; encoder antes de reductora 100:1. Pulsos por vuelta desconocidos: no asumir RPM. |
| Control de corriente | PID bloqueado. La presencia de su interfaz no significa que el lazo este habilitado ni ajustado. |
| Modelo y torque | Identificacion y calibracion corriente/torque pendientes; sin lazos de velocidad o posicion implementados. |

### Que debe mostrar la nueva revision

Para PWM estable, usando la magnitud de la consigna en ambos sentidos:

| PWM | Comportamiento esperado, sin fallos adicionales |
| --- | --- |
| 0% | Estado 2, corriente ON y EMA publicados ausentes; memoria interna del EMA conservada. |
| Mayor que 0% y menor que 100% | ADC raw/voltaje de diagnostico disponibles; estado 12, «fase PWM sin validar»; corriente ON y EMA ausentes. No equivale a 0 A. |
| 100% | Muestras continuas aceptables sin clasificar ON/OFF: estado 13, o 14 si se asigna cero por bajo rango. Calibracion fisica pendiente. |

Durante cambios pueden aparecer estados de establecimiento; errores, saturacion y perdidas mantienen su diagnostico propio. Esta tabla describe el software, no resultados ya observados en la placa.

### Proximos pasos

1. Abrir uno de los Processing actualizados a 460800 baudios. En la prueba del 15/09 se confirmo COM9 y `sensing_version=2`; comprobar el puerto si cambia la conexion.
2. Verificar recepcion, tasas observadas, contadores, ausencia de perdidas y comportamiento en STOP. El envio a Processing sigue siendo aproximadamente 100 Hz, no 200000 tramas/s.
3. Medir las latencias ADC/MCPWM y su variacion bajo carga antes de habilitar la seleccion ON. Mantener el indicador de fase sin validar mientras no exista evidencia temporal suficiente.
4. Contrastar voltaje ADC y corriente con referencias externas; verificar el rango bajo y la ganancia supuesta de 2.49 V/A. El filtro no corrige una calibracion incorrecta.
5. Repetir los ensayos multinivel una vez validado el sensado; calibrar el encoder e identificar el modelo antes de disenar y habilitar el lazo.

### Processing manual: PWM normalizado y EMA opcional

El sketch manual es `processing/ControlCurrentN20_Processing4/ControlCurrentN20_Processing4.pde` (no existe un archivo llamado exactamente ControlCurrentN20.pde en este proyecto). Incluye una grafica de la consigna PWM enviada por esta interfaz: `abs(porcentaje)/100`, de 0 a 1, con el sentido indicado aparte. Es la envolvente de duty, no los pulsos electricos de 10 kHz ni una confirmacion de aplicacion. Solo se actualiza tras escribir SET al puerto; editar el deslizador sin enviar no cambia esta senal.

Cambiar `final boolean SHOW_EMA = true;` a `false` al inicio del sketch para mostrar solo corriente cruda, sin curva, barra ni valor EMA. Esta opcion actua en Processing; no desactiva ni modifica el EMA del firmware. Se conservan las tramas originales y las muestras invalidas siguen ausentes. PWM y corriente comparten historial, pausa y limpieza; el historial representa muestras de telemetria, no tiempos exactos de envio. Las funciones del sketch incluyen comentarios sobre su cometido.

Control y adquisicion de datos de un motor DC N20 con reductora, driver DRV8876 y ESP32 mediante ESP-IDF. El objetivo actual es medir corriente y encoder durante ensayos de PWM para identificar el conjunto motor-reductora y ajustar el lazo de corriente.

## Estado actual: DMA 200 kS/s + EMA por muestra; fase pendiente de validar

Se mantiene el cableado: CS en GPIO32, PWM en GPIO27/GPIO26 y encoder en GPIO18/GPIO19. ADC1 adquiere un solo canal a **200000 muestras/s nominales**, en bloques de 200 muestras (1 ms). MCPWM conserva 10 kHz, la tarea de secuencias/encoder consulta a 1 kHz y Processing recibe unas 100 tramas/s. El PID permanece bloqueado.

**Limitacion activa:** con el cableado actual no existe una marca PWM dentro de cada muestra DMA. Ya no se usa el tiempo de lectura de la tarea ni `tiempo % 100` como si fuera una fase valida. Se registran el timestamp de la interrupcion de fin de bloque ADC y la referencia del comparador MCPWM (1 us del periodo). Esta reconstruccion sigue dependiendo de las latencias de ambas interrupciones y de la frecuencia ADC real; no es sincronizacion hardware del ADC.

Por ello `SENSE_DMA_PHASE_VALIDATED=0` es el valor de fabrica. Para 0 < |PWM| < 100% el ADC sigue adquiriendo raw/voltaje, pero corriente ON y EMA publicados son NaN con estado 12: **fase PWM sin validar**. No se afirma haber excluido OFF. A 100% estable no hace falta clasificar fases y se procesan las muestras continuas; a 0% no se actualiza el EMA ni se presenta corriente ON. No se requiere agregar puentes.

### Procesamiento y protecciones

- Se elimino el maximo por bloque: cada muestra aceptada entra al EMA, `y += 0.00052666384*(x-y)`. La primera aceptada inicializa el filtro. El estado se conserva al cambiar PWM, sentido, OFF, errores y perdidas DMA; durante intervalos invalidos la salida publicada es ausente. A 200 kS/s aceptadas, la constante de tiempo aproximada es 9.49 ms; al aceptar menos muestras, se hace mas lenta. No se cambia alpha con el duty.
- Se precalculan las 4096 conversiones de codigo ADC a mV con line-fitting SDK, 12 bits/12 dB. No se invoca la calibracion 200000 veces por segundo. Esto no calibra fisicamente la cadena CS/corriente.
- Solo en muestras temporalmente aceptadas, codigo cero o voltaje menor de 150 mV se asignan a 0 A (estado 14). Exactamente 150 mV se convierte con 2.49 V/A. No demuestra corriente fisica nula. Saturacion 4095, errores y fase no verificada nunca se convierten en 0 A.
- Una generacion identifica cada cambio de salida, antes de tocar MCPWM. Bloques que cruzan cambios, los dos periodos de establecimiento y datos antiguos se descartan. El EMA se calcula en una copia y se confirma solo si la generacion sigue vigente.
- Las marcas de fin de bloque se emparejan con el FIFO DMA. Si se pierde el emparejamiento, se detiene/limpia/reinicia la adquisicion; el contador de perdidas provoca STOP desde la tarea de control. El EMA conserva memoria, pero no aparece como una lectura nueva. Los contadores no se reinician durante la recuperacion.
- La ventana implementada para **despues de validar la fase** excluye 5 us tras el flanco y 2 us antes de OFF, ampliados por `SENSE_PHASE_UNCERTAINTY_US=5` a ambos lados. No se usa la amplitud de CS para decidir ON/OFF. A 10% no queda ventana con estos margenes; no se promete medir todos los duties.

### Validacion temporal pendiente en placa

Antes de habilitar `SENSE_DMA_PHASE_VALIDATED`, medir la diferencia entre los instantes fisicos y las marcas ISR, bajo carga de serial/encoder, y ajustar `SENSE_DMA_EOF_LATENCY_US`, `SENSE_PWM_ISR_LATENCY_US` y la incertidumbre en `main/current_sense.h`. Verificar tambien 200 kS/s reales, fase, jitter y ausencia de muestras OFF con una referencia de laboratorio. Las capturas previas de CS/PWM no miden por si solas la latencia DMA. Si no puede acotarse esa incertidumbre, no habilitar el clasificador; para una garantia temporal se necesitara una referencia capturada junto con ADC u otro hardware de adquisicion.

### Processing y CSV

Los tres sketches se actualizaron: manual, CurrentPID y StepExperiment. Distinguen tasa ADC nominal/observada de consultas, cuentan muestras ON aceptadas, descartes por ventana y descartes por calidad/fase/cambio. Muestran explicitamente fase pendiente y no unen curvas a traves de datos ausentes. El manual conserva PWM normalizado 0..1 y `SHOW_EMA`; esta opcion solo controla la visualizacion.

DATA conserva los primeros 44 campos y agrega cuatro (48 en total): `sensing_version=2`, `dma_rejected_window`, `dma_rejected_quality`, `dma_phase_validated`. `current_valid_count` cuenta muestras ADC aceptadas, no consultas. `adc_sample_count` cuenta todas las muestras procesadas (no incluye muestras perdidas); la suma de aceptadas + ambos descartes coincide con ese total. `sample_attempt_count` conserva consultas; `sample_missed_count`, notificaciones de consulta perdidas.

En version 2, `adc_read_us` es la duracion nominal del bloque (1000 us), no una llamada oneshot; `sample_time_us` es el timestamp ISR de recepcion del bloque, no el instante fisico del ADC. `sample_phase_us` es una estimacion, -1 cuando no aplica. `current_raw_a` es la ultima muestra aceptada del bloque, no su pico ni promedio. `current_ema_alpha` indica el coeficiente por muestra aceptada.

Estados nuevos: 12 fase no verificada (invalido), 13 DMA ON/100% aceptado, 14 bajo rango asignado a cero. Estado 9 conserva su significado historico de CS continuo sin seleccion, para no reinterpretar CSV antiguos. Los CSV nuevos identifican `DMA_ON_samples`, `DMA_DC_100_percent` o `DMA_below_range_zero`. Esta corriente ON no es automaticamente corriente media del devanado ni torque medio.

Referencia SDK: [ADC continuo ESP-IDF 5.4.4](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-reference/peripherals/adc_continuous.html).

### Verificacion de esta revision

- Compilacion ESP-IDF 5.4.4: `build-codex/ControlCurrentN20.bin`, 227872 bytes (0x37A20), aproximadamente 78% libre en la particion de aplicacion. Registro: `build-codex/dma-correction-build.log`.
- `tests/test_dma_sense.c` ejecuta la tarea de produccion con SDK simulado: EMA de las 200 muestras (no maximo), rechazo OFF/ventana, 149/150 mV, saturacion, generacion de PWM, perdida de FIFO y conservacion del filtro. Se prueba con fase deshabilitada y habilitada simulada; esto no valida tiempos en placa.
- Los tres Processing compilan; `TestSensingProcessing` verifica versiones antiguas y nuevas, CSV y huecos de datos invalidos. `TestStepExperiment` verifica secuencias y exportacion. `RenderDMA` genera vistas simuladas para inspeccion visual.
- `tests/test_current_sense.c` corresponde al oneshot historico y ya no es la prueba de la adquisicion activa.
- Durante la compilacion no se flasheo ni acciono el motor. Posteriormente, el 15/09 a las 10:28, se confirmo la revision en placa y se enviaron escalones PWM por serial (ver informe). La fase del DMA permanece sin validar y deshabilitada para PWM parcial.

## Archivos de referencia

- [Adquisicion DMA y EMA](main/current_sense.c) y [parametros de sensado](main/current_sense.h).
- [Clasificador de ventana ON](main/dma_window.h) y [control, PWM y telemetria](main/main.c).
- [Processing manual](processing/ControlCurrentN20_Processing4/ControlCurrentN20_Processing4.pde).
- [Processing de corriente/PID](processing/ControlCurrentN20_CurrentPID/ControlCurrentN20_CurrentPID.pde).
- [Processing de escalones](processing/ControlCurrentN20_StepExperiment/ControlCurrentN20_StepExperiment.pde).
- [Historial de revisiones, conexiones, comandos y experimentos anteriores](README_HISTORIAL.md). Los protocolos y procedimientos antiguos se conservan como antecedentes; no sustituyen el estado vigente descrito aqui.


# Historial de ControlCurrentN20

Estas secciones conservan las revisiones y experimentos anteriores. Sus descripciones de adquisicion, protocolo y habilitacion PID no son instrucciones para la version vigente. Consultar el [estado actual](README.md).

## Archivo: lectura ON sincronizada por software (9 de septiembre de 2026)

**Esta seccion es historica: su inicio tras 5 us, piso como ausente y reinicio de EMA han sido reemplazados por las reglas anteriores.**

Se sustituye el ADC continuo/DMA por la API publica `adc_oneshot_read()` de ESP-IDF 5.4.4. El ESP32 clasico no entrega una fase PWM por muestra DMA: la llegada de un bloque no permite determinar con fiabilidad su ventana ON. Esta revision **no es DMA sincronizado ni disparo hardware del ADC**. Usa sincronizacion por software con el flanco fisico de GPIO27 o GPIO26 y comprobacion conservadora de la conversion completa.

### Adquisicion y ventanas

- PWM: 10 kHz (periodo 100 us), mismo patron marcha/libre y consignas -100..100 %. No se cambio PMODE ni el cableado.
- ADC: GPIO32 / ADC1_CH4, 12 bits, atenuacion 12 dB y calibracion line-fitting del SDK. No se ha corregido el piso de medida ni calibrado fisicamente IPROPI.
- Una tarea despertada por MCPWM intenta una lectura cada 1 ms. Espera observar LOW y luego HIGH en la entrada PWM activa; acota el flanco con timestamps antes/despues de leer GPIO.
- Inicio de lectura >= extremo superior del flanco + **5 us**. Final de lectura <= extremo inferior del flanco + duracion ON - **2 us**. Los timestamps se redondean conservadoramente. Los margenes son valores iniciales a validar con osciloscopio.
- Se valida toda la llamada ADC, incluyendo su sobrecarga; no se afirma conocer el instante exacto de apertura. Conversiones retrasadas o que crucen OFF se descartan, aunque sus extremos caigan HIGH en ciclos diferentes. La espera de flanco esta acotada a 250 us.
- Tras cambiar PWM se esperan al menos dos periodos. A 100 % no hay flancos: se verifica la salida constante y se usa fase -1. A 0 % se conserva ADC/voltaje como diagnostico, pero corriente y EMA quedan ausentes.
- El motor lock impide cambiar el duty solicitado durante la lectura. Una muestra valida representa **corriente durante ON**, no corriente media del ciclo ni torque medio directamente.

| PWM | ON | Ventana candidata desde el flanco |
| --- | --- | --- |
| 5 % | 5 us | Ninguna |
| 10 % | 10 us | 5..8 us |
| 30 % | 30 us | 5..28 us |
| 50 % | 50 us | 5..48 us |

La tabla no garantiza que la API ADC quepa en esas ventanas. En ensayos anteriores se observaron llamadas del orden de 44 us: si persiste esa duracion, **ni 10 % ni 30 % proporcionaran medidas validas**. Esta revision no acelera el ADC ni promete resolver el sensado a duty bajo; muestra explicitamente la limitacion. Para ventanas mas cortas se necesitara una adquisicion con temporizacion hardware adecuada o validar marcha/freno con el DRV8876.

### EMA, controlador y protocolo

El EMA solo integra muestras ON validas. Usa `alpha(dt)=1-pow(0.9,dt/1000 us)`: constante de tiempo aproximada 9.49 ms. Despues de una muestra invalida o una separacion mayor de 20 ms, se reinicia con la siguiente muestra valida. Este filtro sigue siendo de observacion; el lazo rapido requiere diseno y validacion aparte.

El PID permanece bloqueado con `ERR,PID,ON_current_requires_calibration_and_controller_tuning`. La ventana valida no certifica ganancia de corriente, signo durante frenado ni estabilidad del controlador anterior. SET, STOP y los ensayos multistep siguen disponibles.

Se conservan las **44 columnas DATA** y 460800 baudios:

- `current_status=1` y `current_valid=1`: ventana ON validada; 2: PWM apagado; 3: ventana insuficiente; 4: codigo ADC cero; 5: saturacion; 6/7: error ADC/calibracion; 8: cambio de PWM reciente.
- `adc_sample_rate_hz=1000`: objetivo de intentos, no garantia de conversiones ni lecturas validas. Processing recibe unas 100 tramas/s.
- `adc_sample_tick=0`: no hay fase fija de disparo hardware. El comparador que despierta la tarea no es el disparo del ADC.
- `sample_phase_us`: cota superior de la fase de inicio de la llamada ADC, -1 si no existe flanco aplicable.
- `adc_read_us`: duracion conservadora de la llamada ADC. `sample_time_us`: fin acotado de conversion, o tiempo del intento si no se convierte.
- `adc_sample_count` cuenta conversiones completadas, incluidas las rechazadas por ventana/piso. `current_valid_count` cuenta solo lecturas ON aceptadas. La frecuencia ADC observada no es frecuencia de muestras validas.
- `current_ema_alpha=0.1` es el coeficiente nominal para 1 ms; el real se ajusta al tiempo entre medidas.
- `dma_overflows=0` queda reservado por compatibilidad; DMA no esta activo. `adc_voltage_ema` corresponde al EMA ON.

Los tres Processing muestran corriente ON, ausencia de medida, fase de inicio y duracion ADC. Mantienen compatibilidad con firmware continuo (estado 9). Los CSV identifican `validated_ON_current`, `continuous_CS_all_phases` o `unavailable`; no conectan graficas a traves de muestras ausentes.

### Verificacion y siguiente ensayo

Verificado: firmware compilado con ESP-IDF 5.4.4 en `build-codex` (binario 0x34710 bytes; 80 % de particion libre). Los tres sketches pasaron preprocesado/compilacion, pruebas Java de protocolo/CSV/secuencias y revision visual con datos simulados. Pruebas C ejercitan 16250304 combinaciones de limites, todos los desfases de despertar a 20/40/100 %, interrupciones simuladas, piso/saturacion, errores y EMA con tiempo variable. La conversion simulada rapida no prueba que el ADC fisico cumpla esos tiempos.

Esta revision requiere nuevo flasheo; no se ha accionado el motor durante el desarrollo. Comparar en el osciloscopio CS y GPIO PWM a 30 %, registrar `adc_read_us`, fase y estados, y solo bajar a 10 % si existe ventana suficiente. Usar voltajes conocidos para calibrar ADC y una referencia independiente para calibrar CS/corriente. El pico de conmutacion se excluye; la rampa fisica de corriente durante ON no debe eliminarse como si fuera ruido.

## Archivo: ADC continuo libre con EMA (revision anterior)

**Lo siguiente describe la revision DMA anterior y no la configuracion activa.**

Esta revision sustituye el muestreo oneshot por ventanas. ADC1_CH4 (GPIO32) adquiere CS continuamente a **200000 muestras/s nominales**, con 12 bits y atenuacion de 12 dB. No espera flancos ni selecciona muestras segun el PWM. La adquisicion utiliza ADC continuo con DMA (I2S0 en ESP32) y una tarea dedicada en el segundo nucleo. El PWM permanece a 10 kHz.

- El EMA se aplica a **cada muestra**, incluyendo ON, OFF y codigos ADC cero. Se procesa voltaje calibrado mediante una tabla precalculada para evitar llamadas de calibracion repetidas a 200 kHz.
- `alpha = 0.00052666384` por muestra: respuesta equivalente aproximadamente a alpha 0.1 cada 1 ms, constante de tiempo de 9.49 ms a la tasa nominal. Si la tasa real difiere, cambia la constante de tiempo real.
- Se reciben bloques de 200 muestras (1 ms nominal). Hay espacio para ocho bloques; un desbordamiento descarta datos pendientes, reinicia el filtro y queda registrado. La tarea no imprime datos por muestra.
- La tarea de secuencias/encoder consulta instantaneas aproximadamente a 1 kHz. Su notificacion MCPWM se conserva como temporizador del experimento; **no dispara el ADC**. Processing recibe instantaneas a unos 100 Hz.
- Las frecuencias observadas y el contador ADC permiten verificar el ritmo efectivo. El contador incluye todas las muestras procesadas, no solamente las instantaneas enviadas a Processing. Los datos perdidos no se cuentan como procesados.

### Interpretacion de las medidas

`adc_raw` y `adc_voltage` son la ultima muestra del bloque; `adc_voltage_ema` es el EMA de toda la senal CS. Las columnas historicas `current_raw_a` y `current_ema_a` contienen respectivamente estos voltajes divididos entre 2.49 V/A, y se presentan como **CS equivalente**, pendiente de calibracion. No se divide por el duty ni se resta 0.057 A. No representan automaticamente corriente media del devanado: CS puede caer en OFF mientras sigue circulando corriente en el motor.

El codigo ADC cero se conserva y se convierte con la calibracion SDK, que puede entregar aproximadamente 0.142 V para ese codigo. Por tanto, puede aparecer de nuevo una base equivalente cercana a 0.057 A incluso en reposo. Es un limite de interpretacion/calibracion del ADC, no prueba de corriente real; el EMA no lo elimina. No se fuerza ese voltaje a cero ni se inventa un offset.

El modo continuo usa `current_status=9` y `current_valid=1` para indicar datos de CS procesados sin fallos detectados; **no certifica corriente del motor**. Estado 10 indica desbordamiento DMA. Saturacion, fallo de calibracion, error de lectura o datos demasiado antiguos producen valores ausentes. Los errores ADC y los desbordamientos provocan STOP si se esta accionando el motor.

La entrada al PID mediante MODE/CTRL queda bloqueada con `ERR,PID,continuous_CS_requires_current_model_calibration`: usar directamente este promedio cambiaria la magnitud realimentada del lazo. SET, STOP, encoder y secuencias continuan disponibles. Se debe validar la relacion CS/corriente antes de habilitar de nuevo ese controlador.

### Processing y protocolo actuales

Los tres sketches aceptan el modo continuo y conservan compatibilidad con las tramas antiguas. Las graficas identifican CS equivalente. Los CSV de StepExperiment y CurrentPID agregan `measurement_basis=continuous_CS_all_phases`, `dma_overflows` y `adc_voltage_ema`.

Cada DATA tiene ahora **44 campos incluyendo DATA**. Los primeros 42 mantienen su orden. Se agregan indice 42 `dma_overflows` e indice 43 `adc_voltage_ema`:

- `adc_sample_rate_hz`: 200000 nominal; la tasa observada cuenta muestras procesadas.
- `adc_sample_tick`: 0; sin fase de disparo ADC.
- `sample_phase_us`: -1; sin asociacion a fase PWM.
- `adc_read_us`: tiempo de procesamiento del bloque, no duracion de una conversion individual.
- `sample_time_us`: marca de software al recibir el bloque, no timestamp de apertura ADC. Los bloques tienen duracion y pueden acumular retraso; no usar esta marca como fase exacta.
- `current_ema_alpha`: coeficiente por muestra, publicado con siete decimales.
- `current_valid_count`: instantaneas aceptadas por la tarea de 1 kHz; no cuenta las 200000 muestras ADC individuales.

La exportacion a 100 Hz no reconstruye el pulso de 10 kHz. La ultima muestra puede caer en ON u OFF; el EMA incorpora todas las muestras que llegan correctamente por DMA. Para comparar con el osciloscopio, usar primero voltajes DC conocidos y despues la media de CS sobre muchos periodos, considerando el retraso del EMA. La captura de todas las muestras crudas hacia el PC no esta implementada.

### Verificacion y puesta en marcha

Firmware y sketches deben actualizarse juntos, a 460800 baudios. Compilar con ESP-IDF 5.4.4 mediante `idf.py build` y cargar mediante `idf.py -p COM6 flash` (ajustar puerto). Cerrar Processing antes del flasheo.

Se verifica la compilacion ESP32 en `build-codex/`, los tres sketches y las pruebas `tests/test_continuous_ema.c`, `tests/TestSensingProcessing.java` y `tests/TestStepExperiment.java`. Las pruebas del EMA comprueban respuesta temporal, inclusion de ON/OFF y reinicio. La tasa efectiva, el comportamiento electrico y el consumo de CPU requieren repetir el ensayo en la placa; no se ha flasheado en esta revision.

Para la comprobacion inicial: mantener OPEN, revisar ADC observado cercano a 200000 Hz y `dma_overflows=0`; probar PWM cero y varios niveles; comparar `adc_voltage_ema` con CS promediado en el osciloscopio. Conservar el registro crudo y revisar el piso del ADC antes de ajustar offset o ganancia.

## Archivo de la revision anterior: estado al 7 de septiembre de 2026

**Las secciones siguientes documentan el firmware anterior con ventanas ON y sus experimentos. Sus reglas de validez, tiempos y habilitacion PID fueron reemplazadas por la revision continua descrita arriba.**

| Area | Estado actual |
| --- | --- |
| PWM manual | Implementado: -100 a 100 %, positivo horario, negativo antihorario y cero parada. |
| Corriente | Adquisicion revisada: ventana ON comprobada por software, calibracion ADC por muestra y EMA segun tiempo real. Validez y fallos explicitos; pendiente verificacion fisica. |
| Control de corriente | Modos OPEN/PID, limites y anti-windup. Requiere realimentacion valida para entrar en PID; congela el calculo sin ella y detiene tras 20 ms de perdida. Ajuste experimental pendiente. |
| Encoder A/B | Implementado con PCNT x4 y conteo firmado de 64 bits. Se registro movimiento en el ensayo del 07/09; pendiente calibrar resolucion y validar conteo. |
| Resolucion del encoder | Desconocida. Esta antes de la reductora; se guardan conteos sin suponer RPM. |
| Escalones PWM | Secuencias de varios niveles y duraciones individuales, ejecutadas por el ESP32. El ejemplo principal recorre 0..100..0 % en pasos de 5 % cada 1 s. |
| Processing | Tres sketches: manual, PID e identificacion. Muestran estado del sensado y frecuencias observadas; las muestras invalidas dejan huecos en corriente y celdas vacias en CSV. |
| Comunicacion | Firmware y sketches a 460800 baudios. |
| Modelo MATLAB / torque | Todavia no identificado ni implementado. No hay lazos cerrados de velocidad o posicion. |

### Verificaciones de la version anterior al cambio de sensado

- Compilacion completa con ESP-IDF 5.4.4 para ESP32, incluido enlace, generacion de imagen y comprobacion de tamano. La verificacion se ejecuto en `build-codex/`; imagen generada de 209536 bytes, con aproximadamente 80 % libre en la particion de aplicacion.
- Corregido el orden de cabeceras: `freertos/FreeRTOS.h` debe incluirse antes de `freertos/semphr.h`. Ese orden causaba los errores de compilacion en cascada.
- Los tres sketches pasaron preprocesado y compilacion Java con Processing instalado.
- Pruebas de software de validacion de secuencias, teclado, ACK, conteos exactos, tiempos, RPM, CSV y fin/reinicio de ensayo.
- Revisado el renderizado de la nueva interfaz de escalones.
- Corregida la recursion del teclado que causaba `StackOverflowError` en el campo PWM manual.

La compilacion y las pruebas de software no validan el controlador sobre el motor. El ensayo multinivel documentado abajo se realizo con el firmware anterior: confirma respuesta del encoder, pero detecta limitaciones de corriente y una discrepancia de frecuencia. La revision de sensado descrita en este README requiere flashear el firmware nuevo y repetir el ensayo; aun no hay una medicion sobre el motor que demuestre su resultado.

### Verificaciones de la revision de sensado

- Compilacion completa para ESP32 con ESP-IDF 5.4.4 en `build-codex/`: enlace y generacion de imagen correctos; imagen de 214704 bytes, con aproximadamente 80 % libre en la particion de aplicacion.
- Los tres sketches actualizados pasaron preprocesado y compilacion Java.
- `tests/test_current_sense.c` ejercita el codigo de sensado con tiempo, GPIO y ADC simulados: ventanas, conversiones retrasadas, piso/saturacion ADC, calibracion, PWM cero y continuo.
- `tests/TestSensingProcessing.java` y `tests/TestStepExperiment.java` verifican estados de corriente, tratamiento de valores ausentes, CSV y comportamiento del ensayo.
- No se flasheo el ESP32 ni se acciono el motor durante estas verificaciones.

## Motor y conexiones

Segun la tabla proporcionada por el usuario, el N20 de 300 RPM tiene reductora 100:1, alimentacion nominal de 12 V, 300 RPM en vacio y 230 RPM bajo carga nominal. La tabla indica 0.05 A en vacio, 0.20 A nominales y 0.85 A de bloqueo. Son referencias del catalogo, no mediciones del ejemplar ni limites automaticos del firmware. El bloqueo no es un punto de funcionamiento continuo.

| Senal | GPIO ESP32 | Destino |
| --- | ---: | --- |
| PWM horario | 27 | IN1 del driver, segun cableado actual |
| PWM antihorario | 26 | IN2 del driver, segun cableado actual |
| Corriente | 32 | CS/IPROPI, ADC1_CH4 |
| Encoder A | 18 | Canal A del encoder del rotor |
| Encoder B | 19 | Canal B del encoder del rotor |
| GND | GND | Tierra comun de ESP32, driver, encoder y fuente |

Los niveles digitales A/B deben ser compatibles con 3.3 V; no conectar salidas de 5 V directamente al ESP32. Confirmar la alimentacion y el tipo de salida del encoder antes de cablear. Verificar tambien que el modo de entradas del DRV8876 corresponde al control IN1/IN2 utilizado.

## Archivos activos

| Archivo o carpeta | Responsabilidad |
| --- | --- |
| `main/main.c` | PWM MCPWM, planificacion de adquisicion, EMA, PID, comandos seriales y telemetria. |
| `main/current_sense.c`, `main/current_sense.h` | Lectura ADC, comprobacion de ventana ON, calibracion por muestra y estados de validez. |
| `main/encoder.c`, `main/encoder.h` | PCNT, pines y signo del encoder. |
| `main/pwm_sequence.c`, `main/pwm_sequence.h` | Validacion y temporizacion de secuencias PWM. |
| `main/CMakeLists.txt` | Incluye los cuatro archivos C y las dependencias ESP-IDF. |
| `sdkconfig.defaults` | Consola UART personalizada a 460800 baudios para configuraciones nuevas. |
| `processing/ControlCurrentN20_Processing4/` | Control manual con deslizador y cuadro numerico. |
| `processing/ControlCurrentN20_CurrentPID/` | Referencia, ganancias PID, limites, control manual y CSV del lazo. |
| `processing/ControlCurrentN20_StepExperiment/` | Secuencias multinivel, encoder, graficas y CSV de identificacion. |
| `tests/` | Preprocesado de sketches, pruebas de identificacion y renderizado de interfaz. |

`main/LEDC_program.txt` y `main/FuncionalTEST1.txt` son referencias historicas; no forman parte de la compilacion activa. Las exportaciones anteriores conservan el formato y las unidades de su version.

## Compilacion y puesta en marcha

En un entorno ESP-IDF 5.4.4 configurado para ESP32, desde esta carpeta:

```text
idf.py build
idf.py -p COM6 flash
```

`COM6` es el puerto utilizado anteriormente; cambiarlo si corresponde. Cerrar Processing y cualquier monitor serial antes de flashear. Abrir solo un sketch sobre el mismo puerto a la vez.

Actualizar firmware y sketches juntos. La consola debe tener `CONFIG_ESP_CONSOLE_UART_CUSTOM=y` y `CONFIG_ESP_CONSOLE_UART_BAUDRATE=460800`; el modo de consola por defecto puede restablecer 115200 durante la configuracion. La UART sigue usando los pines habituales del ESP32, TX GPIO1 y RX GPIO3.

La carpeta `build-codex/` se uso para una verificacion aislada y no sustituye la compilacion normal en `build/`. La verificacion completa no flasheo el dispositivo. No abrir el monitor de IDF simultaneamente con Processing.

## Frecuencias, muestreo y estado compartido

| Parametro | Valor configurado |
| --- | ---: |
| PWM | 10000 Hz |
| Reloj MCPWM | PLL160M, divisor entero para 20 MHz |
| Periodo MCPWM | 2000 ticks = 100 us |
| Resolucion del timer | 20000000 Hz |
| Paso PWM | 0.05 %; 100 % es salida continua |
| Evento para despertar la tarea | Tick 20 = 1 us, cada 10 periodos PWM |
| Intentos de adquisicion / lectura del encoder | Nominalmente 1000 Hz |
| Telemetria serial | Nominalmente 100 Hz |
| EMA alpha nominal | 0.10 a intervalos de 1 ms; ajustado al intervalo valido real |

El reloj y periodo nuevos sustituyen la solicitud anterior de 40.96 MHz / 4096 ticks. La interrupcion MCPWM notifica a la tarea; `adc_oneshot_read()` sigue ejecutandose en contexto de tarea. El evento de 1 us solo despierta la tarea: no se publica como instante real de conversion. La tarea busca una ventana ON mediante el nivel del GPIO activo, segun el procedimiento de sensado descrito abajo. El encoder se consulta despues; no son mediciones capturadas simultaneamente por hardware.

Cada segundo se calculan por separado la frecuencia PWM observada por callbacks MCPWM, los intentos de adquisicion procesados y las conversiones ADC exitosas. Estos contadores permiten detectar retrasos y descartes; la frecuencia derivada de interrupciones no sustituye medir el pin PWM con un osciloscopio. Una tasa ADC menor de 1000 Hz puede corresponder a ventanas rechazadas sin conversion. Las notificaciones acumuladas se contabilizan como intentos perdidos, sin ejecutar una rafaga de conversiones atrasadas.

Un mutex coordina los cambios seriales con la tarea de muestreo para evitar aplicar una salida PID calculada antes de una parada. La telemetria conserva una instantanea de la iteracion de adquisicion. Los cambios de PWM de una secuencia se revisan nominalmente cada 1 ms; sus marcas de tiempo corresponden a software, no a un flanco medido en el driver.

El CSV recibe instantaneas a unos 100 Hz, no todos los intentos de 1 kHz. Para analizar los intervalos reales, usar `sample_time_us` y los contadores de intentos, conversiones y muestras validas. La identificacion de dinamica electrica rapida puede requerir una captura de mayor frecuencia, aun no implementada. No se ha implementado ADC continuo, DMA ni digitalizacion a 20 kHz.

## Sensado de corriente revisado

Se retiro la resta fija de 0.057 A. Procedia de convertir un ADC crudo igual a cero en unos 0.142 V y dividir entre 2.49; eso no es una calibracion independiente del cero del sensor. El ADC en su piso no contiene informacion suficiente para recuperar una corriente mediante una resta.

Para PWM entre 0 y 100 %, en cualquiera de los sentidos, la adquisicion aplica estos pasos:

1. Descarta los primeros 105 us posteriores a un cambio de salida, para permitir la actualizacion del comparador y evitar el primer pulso parcial.
2. Busca una transicion bajo-alto del GPIO activo, con espera maxima de 250 us. Usa los tiempos de la ultima observacion baja y la primera alta como limites del flanco.
3. Espera al menos 5 us desde el limite superior del flanco antes de iniciar el ADC. Exige que la llamada completa a `adc_oneshot_read()` termine antes del final ON conservador, dejando 2 us de margen.
4. Comprueba los niveles de ambas salidas alrededor de la conversion y valida el resultado ADC y la calibracion. Una llamada que se retrasa fuera de la ventana se conserva solo como diagnostico, sin corriente utilizable.

El codigo no aumenta el duty para crear una ventana. A duty muy bajo puede no caber una conversion y se informa `CURRENT_NO_WINDOW`. Al 100 % se utiliza una ruta de nivel alto continuo, sin esperar un flanco que no existe; la llamada ADC tampoco puede superar 250 us. Al 0 % se permite una lectura ADC de diagnostico, pero la corriente queda no medida: apagar PWM no demuestra que la corriente fisica sea cero durante la recirculacion o el frenado.

Los margenes de 5 us y 2 us son provisionales. La hoja de datos DRV8876 indica valores tipicos de 1.6 us para el retardo del sensado y 0.65 us de propagacion; no constituyen una garantia del establecimiento de IPROPI, su filtro externo ni este montaje. Hay que observar PWM e IPROPI para confirmar los margenes. La comprobacion software de toda la llamada es conservadora y no identifica el instante exacto de apertura del ADC.

```text
adc_raw = 0..4095; -1 si no hubo conversion exitosa
adc_voltage = adc_cali_raw_to_voltage(adc_raw) / 1000
si current_status == CURRENT_VALID_ON:
    current_raw_a = max(0, adc_voltage / 2.49 - CURRENT_SENSOR_OFFSET_A)
    CURRENT_SENSOR_OFFSET_A = 0.0 A
    alpha = 1 - (1 - 0.10) ^ (dt_valido_us / 1000)
    current_ema_a += alpha * (current_raw_a - current_ema_a)
en otro caso:
    current_raw_a = NaN
    current_ema_a = NaN
```

Se utiliza calibracion line fitting de ESP-IDF con atenuacion de 12 dB y resolucion de 12 bits. `adc_calibrated=1` ahora significa que la conversion de voltaje de esa muestra concreta tuvo exito. Si falla la calibracion, no se sustituye por una aproximacion ideal de 3.3 V para alimentar el controlador. ADC crudo 0 o 4095 se considera fuera de una lectura utilizable; incluso lecturas positivas pequenas requieren validar la precision a baja tension.

La conversion de 2.49 V/A sigue siendo una suposicion del montaje. Debe verificarse el valor real de `R_IPROPI`, la relacion de espejo de corriente del DRV8876 y cualquier filtro o ganancia adicional; por ejemplo, 2.49 kohm con una relacion de 1000 uA/A daria 2.49 V/A. Cambiar `CURRENT_SENSOR_OFFSET_A` solo despues de una calibracion externa. Los 0.05 A de catalogo con el motor girando en vacio son consumo real, no un offset para restar.

El EMA conserva una constante de tiempo equivalente de unos 9.5 ms ajustando alpha al tiempo entre muestras validas. Se inicializa con la primera muestra y se reinicia si el hueco entre muestras validas supera 20 ms. Las muestras invalidas no alimentan el filtro; la telemetria publica `NaN` en lugar de repetir una corriente antigua o inventar cero.

`current_valid=1` significa que se cumplieron las comprobaciones de software. Todavia no demuestra exactitud en amperios, ni que la muestra ON sea igual a la corriente media del periodo. El modo de entradas/recirculacion del driver, el rango del ADC, la calibracion fisica y el rizado permanecen pendientes de comprobar.

## Control manual y de corriente

El ESP32 arranca en OPEN, detenido y con referencia de corriente cero.

| Comando | Efecto |
| --- | --- |
| `STOP` | Cancela secuencia, selecciona OPEN y detiene el motor. |
| `SET,25.5` | OPEN, horario a 25.5 %. |
| `SET,-25.5` | OPEN, antihorario a 25.5 %. |
| `SET,0` | OPEN, PWM cero. |
| `IREF,0.150` | Configura referencia de corriente de 0.150 A. |
| `MODE,PID,CW` | Activa PID horario si hay un punto manual con corriente valida reciente. |
| `MODE,OPEN` | Desactiva PID y cancela secuencia; no equivale a STOP. |
| `DIR,CCW` | Cambia la direccion configurada para el controlador. |
| `PID,0,8320,0` | Ajusta Kp, Ki y Kd y reinicia el estado del PID. |
| `LIMIT,0,40` | Limita la magnitud de salida del PID a 0..40 %. |
| `CTRL,CW,0.150` | Configura referencia y activa PID horario si cumple las condiciones de sensado. |

`SET` solo acepta porcentajes finitos entre -100 y 100. Ya no admite `SET,CW,valor` ni valores PWM en escala 0..4095. La conversion a ticks permanece dentro del firmware.

El PID usa integracion condicional anti-windup. Los valores iniciales son `Kp=0`, `Ki=8320`, `Kd=0`: corresponden a un controlador integral experimental, no a un ajuste validado. La salida PID es una magnitud positiva y la direccion se configura por separado. Tener PWM manual firmado no implica control de torque de cuatro cuadrantes ni corriente medida con signo.

Para entrar en PID mediante `MODE,PID,...` o `CTRL,...`, primero establecer en OPEN un punto manual con lectura valida, en el mismo sentido y dentro de los limites PID. La muestra debe tener como maximo 5 ms y ser posterior al ultimo cambio de salida. Tambien se comprueba que referencia, ganancias y limites sean finitos. De no cumplirse, el comando devuelve `ERR,PID,require_fresh_valid_current_same_direction_within_limits` y no activa el controlador. El integrador se inicia desde el duty manual para evitar un salto inicial de salida.

Durante PID solo se actualiza el controlador con corriente valida. Usa el intervalo real si ambas muestras consecutivas son validas, no hay notificaciones acumuladas y el intervalo es positivo y no supera 2 ms. En otro caso usa el periodo nominal de 1 ms y reinicia la memoria de derivada, sin integrar el tiempo sin realimentacion. Si una muestra es invalida, mantiene temporalmente la salida y congela la integracion. Una perdida de realimentacion de 20 ms o un error de lectura ADC provoca STOP y `ERR,CURRENT_SENSE,<estado>,STOP`. El error de lectura ADC tambien detiene una secuencia OPEN. Los cambios en el mismo sentido ya no fuerzan ambas salidas a cero en cada actualizacion PID.

`LIMIT` se aplica al PID; no limita los comandos manuales SET ni los niveles de SEQ. La referencia de corriente y las ganancias deben elegirse con cuidado: el parser de los comandos PID heredados no tiene la misma validacion estricta que SET y SEQ. No hay corte automatico de sobrecorriente implementado por software.

En Processing4 y CurrentPID, escribir el PWM en el cuadro y pulsar Enter guarda el valor; Aplicar PWM lo envia. En Processing4, soltar el deslizador tambien envia. Espacio detiene incluso durante la edicion. Delete vacia el cuadro, Backspace corrige y Escape cancela la edicion del deslizador.

## Encoder y conversion a RPM

PCNT cuenta ambos flancos de A y B (x4), usa filtro de pulsos menores de 1 us y acumula desbordamientos. Se mantiene un conteo firmado extendido a 64 bits. Los conteos se reinician al reiniciar el ESP32, no entre ensayos. Para invertir el signo, cambiar `ENCODER_SIGN` en `main/encoder.h` o intercambiar A/B.

El encoder esta antes de la reductora 100:1. Mientras se desconozca su resolucion, dejar **Conteos x4 / vuelta de SALIDA = 0** en StepExperiment: se exportan conteos, y posicion en vueltas y RPM quedan sin calcular.

Cuando se conozca la resolucion:

```text
CPR salida = ciclos de A por vuelta de rotor * 4 * 100
CPR salida = conteos x4 por vuelta de rotor * 100
RPM salida = delta_conteos * 60000000 / (CPR salida * delta_t_us)
```

Usar solo una de las dos primeras expresiones segun la especificacion disponible, sin multiplicar x4 dos veces. Alternativamente, medir la diferencia de conteos durante varias vueltas completas de salida y dividir entre las vueltas, sin forzar la reductora. Cero visual encoder solo cambia la referencia mostrada; el CSV conserva el conteo absoluto y el relativo al inicio de captura.

Processing estima RPM con ventanas de al menos 50 ms usando tiempos ESP32. La primera muestra y las ventanas insuficientes no tienen velocidad calculada.

## Ensayos de escalones multinivel

Abrir `processing/ControlCurrentN20_StepExperiment/ControlCurrentN20_StepExperiment.pde`, seleccionar puerto, conectar y esperar telemetria del firmware actualizado.

El campo de programa acepta `PWM:duracion_ms` separados por punto y coma:

```text
0:1000;20:2000;40:2000;20:2000;0:1500
```

Este ejemplo aplica 0 % durante 1 s, 20 % durante 2 s, 40 % durante 2 s, 20 % durante 2 s y 0 % durante 1.5 s. Tambien hay un ejemplo de ambos sentidos con pausas en cero. Enter termina la edicion; INICIAR envia todo el programa al ESP32.

```text
SEQ,0:1000;20:2000;40:2000;0:1500
ACK,SEQ,1,4
```

El ACK devuelve identificador de secuencia y numero de etapas. El firmware valida el programa completo antes de ejecutarlo:

- De 1 a 48 etapas.
- PWM entre -100 y 100 %, con decimales y signo.
- Duracion por etapa de 100 a 30000 ms.
- Duracion total maxima de 120 s.
- Linea serial de hasta 511 caracteres, sin contar terminador.

El ESP32 ejecuta los tiempos independientemente del dibujo de Processing y aplica PWM cero al terminar. STOP, SET y cambios de modo cancelan la secuencia. Un fallo de lectura ADC tambien detiene el motor. Evitar comandos manuales durante el registro.

Processing espera el ACK para capturar, descarta muestras repetidas y de otras secuencias, y detecta reinicios y perdida de telemetria. STOP o Espacio detienen; cerrar la aplicacion intenta enviar STOP. Si se pierde fisicamente la conexion, el programa finito del ESP32 continua hasta terminar: no hay watchdog de conexion en el firmware.

## Telemetria y CSV

Las lineas comienzan con HEADER, DATA, ACK o ERR. Cada DATA tiene 41 valores despues del prefijo, 42 campos en total. Se conserva el orden de los 31 valores anteriores y se agregan diez de diagnostico de sensado:

```csv
HEADER,time_ms,adc_raw,adc_voltage,current_raw_a,current_ema_a,direction_code,direction_text,duty_cw_percent,duty_ccw_percent,adc_sample_count,pwm_frequency_hz,adc_sample_rate_hz,adc_sample_tick,current_ema_alpha,control_mode_code,control_mode_text,current_ref_a,current_error_a,control_output_percent,duty_percent,pid_kp,pid_ki,pid_kd,pid_integrator,adc_calibrated,sample_time_us,encoder_count,sequence_id,sequence_index,sequence_start_us,sequence_stage_start_us,current_valid,current_status,adc_read_us,sample_phase_us,current_valid_count,sample_attempt_count,sample_missed_count,pwm_frequency_observed_hz,acquisition_rate_observed_hz,adc_rate_observed_hz
```

| Campos | Interpretacion |
| --- | --- |
| `time_ms`, `sample_time_us` | Fin de conversion o tiempo del intento fallido en el ESP32, en ms y us. |
| `adc_raw`, `adc_voltage`, `adc_calibrated` | ADC crudo (-1 sin conversion exitosa), voltaje y exito de calibracion de esa conversion. Pueden conservar diagnostico aunque la corriente sea invalida. |
| `current_raw_a`, `current_ema_a` | Corriente sin EMA y con EMA, en A; `NaN` cuando no es valida. Sin la resta fija de 0.057 A. |
| `direction_code`, `direction_text` | 0 STOP, 1 CW, 2 CCW; direccion de salida aplicada, no medida por encoder. |
| `duty_cw_percent`, `duty_ccw_percent` | Magnitudes de cada salida, 0..100 %. |
| `duty_percent` | PWM aplicado con signo, -100..100 %. |
| `adc_sample_count` | Numero de conversiones ADC exitosas, incluidas las de diagnostico que no producen corriente valida. |
| `pwm_frequency_hz`, `adc_sample_rate_hz`, `adc_sample_tick` | Configuracion nominal de PWM, intentos de adquisicion y evento de despertar. El tick no es la fase de conversion. |
| `current_ema_alpha` | Coeficiente nominal a 1 ms; el aplicado se adapta al intervalo valido. |
| `control_mode_code`, `control_mode_text` | 0 OPEN, 1 PID. |
| `current_ref_a`, `current_error_a` | Referencia y error de corriente. |
| `control_output_percent`, `pid_kp`, `pid_ki`, `pid_kd`, `pid_integrator` | Esfuerzo y estado del controlador. |
| `encoder_count` | Conteo firmado acumulado x4, entero de 64 bits. |
| `sequence_id`, `sequence_index` | ID y etapa desde cero; etapa -1 cuando no esta activa. |
| `sequence_start_us`, `sequence_stage_start_us` | Tiempos de inicio de secuencia y ultimo cambio de etapa en software. |
| `current_valid`, `current_status` | Validez software (1 solo para estado 1) y motivo de validez o rechazo. |
| `adc_read_us` | Duracion conservadora de la llamada ADC completa, no solo tiempo de apertura. Cero si no se llamo al ADC. |
| `sample_phase_us` | Inicio de la llamada respecto al limite superior del flanco observado. -1 sin referencia de flanco, incluido 100 % continuo. |
| `current_valid_count`, `sample_attempt_count`, `sample_missed_count` | Acumulados de muestras validas, intentos procesados y notificaciones acumuladas que no se procesaron por separado. |
| `pwm_frequency_observed_hz` | Callbacks MCPWM por segundo. Diagnostico software, no medida con osciloscopio. |
| `acquisition_rate_observed_hz`, `adc_rate_observed_hz` | Intentos procesados y conversiones ADC exitosas por segundo, actualizados aproximadamente cada segundo. |

| `current_status` | Nombre | Interpretacion |
| ---: | --- | --- |
| 0 | `CURRENT_UNINITIALIZED` | Todavia sin lectura. |
| 1 | `CURRENT_VALID_ON` | Ventana ON, rango y calibracion ADC aceptados por software; calibracion fisica pendiente. |
| 2 | `CURRENT_PWM_OFF` | PWM apagado; corriente no medida, aunque exista ADC de diagnostico. |
| 3 | `CURRENT_NO_WINDOW` | No se encontro ventana suficiente o la conversion quedo fuera de ella. |
| 4 | `CURRENT_ADC_FLOOR` | ADC crudo cero: corriente no resuelta. |
| 5 | `CURRENT_ADC_SATURATED` | ADC crudo 4095: saturacion. |
| 6 | `CURRENT_ADC_ERROR` | Fallo de la llamada ADC; provoca STOP. |
| 7 | `CURRENT_CALIBRATION_ERROR` | No se pudo obtener voltaje calibrado. |
| 8 | `CURRENT_OUTPUT_SETTLING` | Cambio reciente de salida; muestra descartada durante establecimiento. |

Exportar CSV en StepExperiment conserva corriente, ADC, PWM aplicado, conteos exactos, tiempos ESP32, etapa y programa ejecutado. Con CPR configurado agrega posicion y velocidad de salida. Los archivos se guardan en la carpeta `exports/` del sketch con prefijo `multistep_`.

Los tres sketches muestran el motivo de rechazo y dejan huecos en las curvas de corriente. StepExperiment y CurrentPID exportan la corriente invalida como celda vacia, con estado, tiempos y contadores de diagnostico; PWM y encoder siguen registrandose. En MATLAB, filtrar por `current_valid == 1` para cualquier analisis de corriente y conservar los tiempos originales, sin rellenar huecos con ceros. Si se recibe telemetria antigua sin diagnostico, la interfaz la identifica como corriente sin verificar.

El campo Alimentacion medida (V) es ingresado por el usuario. `estimated_armature_voltage_v = pwm_applied_percent * supply_voltage_user_v / 100` es una aproximacion, no una medicion de tension sobre el motor. El CSV tambien registra si el ensayo termino o fue interrumpido.

## Experimento realizado: 7 de septiembre de 2026

Fuente: [multistep_20260907_171539.csv](processing/ControlCurrentN20_StepExperiment/exports/multistep_20260907_171539.csv). Ensayo en lazo abierto, identificado como secuencia 5 y exportado con estado `terminado`. No se modifico el archivo original durante el analisis. **Corresponde al firmware anterior a la revision de sensado**: muestreo solicitado a mitad de periodo, resolucion solicitada de 40.96 MHz / 4096 ticks, resta de 0.057 A y sin los diez campos nuevos de diagnostico. Los resultados siguientes se conservan como evidencia historica.

### Condiciones y programa

- N20 con reductora 100:1 y encoder en el rotor, antes de la caja.
- Alimentacion ingresada en Processing: 12 V. No es una medicion automatica de tension durante el ensayo.
- CPR de salida configurado en cero: se registraron conteos, sin conversion a RPM.
- Todos los niveles fueron positivos; este ensayo no caracteriza el sentido contrario.
- El CSV no documenta la carga mecanica, temperatura ni tension real sobre el motor. No asumir que fue un ensayo en vacio.

```text
0:1000;20:2000;40:2000;60:2000;80:2000;100:2000;90:2000;70:2000;50:2000;30:2000;10:2000;0:2000
```

Duracion programada: 23 s. Se registraron 2302 filas durante 23.006876 s entre primera y ultima muestra. El desplazamiento acumulado fue de 134587 conteos x4.

### Resultados por nivel

La velocidad se calculo como diferencia de conteos dividida entre diferencia de tiempos ESP32, usando las muestras de los ultimos 0.5 s de cada etapa. La corriente indicada es la media de `current_raw_a` en esa misma ventana: ya incluye compensacion de cero, pero no EMA. Son valores al final de la etapa, no una demostracion de regimen estacionario.

| PWM objetivo (%) | Velocidad al final (conteos/s) | Corriente indicada media (mA) |
| ---: | ---: | ---: |
| 20 | 0 | 0 |
| 40 | 4627 | 0 |
| 60 | 7164 | 29.4 |
| 80 | 10413 | 14.5 |
| 100 | 13874 | 13.8 |
| 90 | 12725 | 13.9 |
| 70 | 8593 | 21.1 |
| 50 | 6213 | 0 |
| 30 | 3014 | 0 |
| 10 | 0 | 0 |

El PWM aplicado puede diferir ligeramente del objetivo por cuantizacion; por ejemplo, el CSV registra 90.012 % para el nivel de 90 %. Para identificar, usar `pwm_applied_percent` de cada muestra.

Interpretacion mecanica:

- No hubo avance al 20 % desde reposo; si hubo movimiento al 40 %. El umbral de arranque queda entre esos niveles bajo estas condiciones, pendiente de repetir con pasos mas pequenos.
- Al bajar el PWM, el motor seguia girando al 30 %, y termino detenido durante la etapa del 10 %. Es compatible con friccion de arranque y diferencias entre iniciar y sostener movimiento; no demuestra por si solo una ley de friccion.
- La curva del encoder es posicion acumulada: su pendiente es velocidad. Puede seguir creciendo mientras baja el PWM porque el giro continua en el mismo sentido.
- Se observo un incremento negativo aislado de un conteo cerca de 19.657 s. Revisar repetibilidad, vibracion o transiciones del encoder antes de interpretarlo como giro inverso.
- Estos datos permiten estudiar PWM a velocidad en conteos/s. Aun no permiten expresar velocidad absoluta en RPM ni torque de salida calibrado.

### Problema de medicion de corriente

En las etapas de 40 %, 50 % y 30 % hay movimiento sostenido, pero todas las muestras ADC de esas etapas son cero. Esos valores no prueban corriente fisica nula. La corriente maxima registrada en el ensayo fue de 0.182 A; no representa necesariamente el pico real, porque la exportacion no contiene todas las conversiones.

Hipotesis a verificar en ese firmware: la notificacion de muestreo estaba situada a mitad del periodo PWM. Con duty pequeno, la conversion podia ocurrir fuera del intervalo en el que IPROPI representa corriente. En ciertos estados de conmutacion el sensado IPROPI se desactiva. Ademas, la conversion se realizaba en una tarea, con latencia respecto al evento MCPWM.

El problema no queda explicado completamente por el duty: al 100 %, aproximadamente el 46 % de las muestras del ultimo medio segundo tienen ADC cero. Revisar tambien rango y precision del ADC a baja tension, cableado, modo del driver, regulacion interna y senal IPROPI con osciloscopio.

La resta de 0.057 A ajustaba la indicacion en reposo; no corregia ventanas de sensado invalidas ni recuperaba informacion perdida cuando el ADC entregaba cero. No usar la corriente de este archivo para identificar el modelo electrico, ajustar definitivamente el PI o estimar torque.

### Tiempos medidos y discrepancia de frecuencia

| Medida del CSV | Resultado |
| --- | ---: |
| Intervalo medio entre filas | 9.9986 ms |
| Intervalo minimo / maximo | 6.813 / 10.251 ms |
| Intervalo mediano | 10.240 ms |
| Huecos mayores de 15 ms | 0 |
| Incremento de `adc_sample_count` | 44936 |
| Lecturas ADC inferidas por segundo | 1953.16 Hz |
| Retraso de inicio de etapas segun marcas de software | 0.032 a 0.416 ms |

La telemetria recibida es aproximadamente de 100 Hz. Sin embargo, el contador ADC avanza a unos 1953 Hz, frente a los 1000 Hz nominales del firmware. La configuracion nominal no debe tratarse como frecuencia medida.

Una causa posible es el ajuste de la resolucion solicitada al divisor realizable del reloj MCPWM. La solicitud de ese firmware era 40.96 MHz y el driver podia ajustarla; habia que verificar la resolucion efectiva y medir el PWM fisicamente antes de atribuirle una frecuencia exacta. La revision ahora solicita PLL160M / 8 = 20 MHz y 2000 ticks, pero este CSV antiguo no valida esa correccion.

Si el PID se ejecuta a 1953 Hz usando `Ts = 1 ms`, su contribucion integral por segundo resulta aproximadamente 1.95 veces la prevista para el mismo error. La derivada y el filtro tambien deben analizarse con el intervalo real. El EMA con alpha 0.1 equivaldria aproximadamente a 4.9 ms a esa frecuencia, frente a 9.5 ms a 1 kHz. Este ensayo fue OPEN; no demuestra estabilidad ni inestabilidad del PID.

## Mejoras implementadas y pendientes

### 1. Frecuencia PWM y adquisicion: software actualizado, medida fisica pendiente

- Implementado: PLL160M con divisor entero, 20 MHz y 2000 ticks; PWM nominal de 10 kHz y paso de 0.05 %.
- Implementado: frecuencias nominales separadas de tasas observadas por callbacks, intentos y conversiones; contadores de muestras validas y notificaciones perdidas.
- Implementado: EMA adaptado al intervalo real entre muestras validas y PID con intervalo real cuando las muestras son consecutivas, congelando huecos de realimentacion.
- Pendiente: confirmar un periodo PWM de 100 us mediante osciloscopio o analizador logico y contrastarlo con los contadores del CSV nuevo antes de ajustar ganancias.

Criterio de verificacion: periodo PWM medido de 100 us dentro de la tolerancia instrumental, contador de conversiones coherente con la tasa objetivo y ausencia de perdidas no explicadas.

### 2. Ventana IPROPI: comprobacion software implementada, validacion electrica pendiente

Implementado: busqueda de flanco ON por GPIO y comprobacion conservadora del tiempo de la llamada ADC completa, con margenes de establecimiento y fin. Se distinguen PWM apagado, ventana insuficiente, ADC en piso, saturacion y errores. Las muestras rechazadas no se interpretan como cero y el PID no integra mientras no haya realimentacion valida.

Pendiente: medir simultaneamente PWM e IPROPI al 20 %, 40 %, 60 % y 100 %. Confirmar el modo de entradas y recirculacion del DRV8876 y que IPROPI este establecido durante la conversion, no solo al despertar la tarea.

A 10 kHz el periodo es 100 us. Los intervalos ON del 20 %, 40 % y 80 % duran 20, 40 y 80 us respectivamente. Un muestreo fijo alrededor de 50 us puede quedar fuera del ON para los niveles bajos.

Si hay demasiadas ventanas rechazadas, usar los diagnosticos de fase y duracion para decidir si reducir latencia, cambiar la arquitectura de adquisicion o revisar el circuito de sensado. La revision no alarga el PWM solicitado. Una lectura durante ON tampoco equivale automaticamente a la corriente media del periodo: validar esa relacion segun el rizado y el modo del puente.

Criterio de verificacion: muestras identificadas como validas dentro de la ventana observada y concordancia de la corriente con una medicion externa en varios niveles, incluido 100 %.

### 3. Mayor tasa de digitalizacion: pendiente segun el objetivo

Nyquist se aplica al ancho de banda de la senal que se desea reconstruir, no directamente a la frecuencia de la portadora PWM. Para una senal limitada en banda, la frecuencia de muestreo debe superar el doble de su frecuencia maxima y contemplar filtrado antialias.

PWM a 10 kHz no implica que ADC a 20 kHz sea suficiente: dos muestras por periodo no reconstruyen los armonicos, flancos o transitorios de IPROPI. Tampoco garantizan muestrear cuando el sensado es valido. El EMA posterior a la conversion no elimina aliasing ya producido.

Dos alternativas a evaluar:

| Objetivo | Alternativa propuesta | Condicion pendiente |
| --- | --- | --- |
| Control de corriente | Una muestra representativa por periodo PWM, sincronizada en una ventana valida. | Garantizar latencia, validez y relacion con corriente media; elegir el ancho de banda del lazo. |
| Diagnosticar rizado y transitorios | ADC continuo con DMA; explorar 100-200 kS/s, equivalentes a 10-20 muestras por periodo de 10 kHz. | Verificar capacidad, frecuencia efectiva, filtrado y alineacion con PWM. DMA por si solo no sincroniza la fase. |

Mantener la frecuencia de adquisicion separada de la frecuencia de envio a Processing. Una captura rapida necesitara buffers y/o un formato de transferencia apropiado; no imprimir una linea CSV larga por cada conversion dentro de la tarea de control.

### 4. vTaskDelayUntil: se conserva para telemetria

`vTaskDelayUntil()` ya se utiliza para programar la telemetria nominal de 100 Hz. Mantiene un calendario periodico respecto al tick de FreeRTOS y evita sumar siempre el tiempo de trabajo al periodo.

No garantiza una conversion en un instante preciso del PWM: al vencer el plazo, la tarea queda lista, pero su ejecucion puede retrasarse. Su resolucion es la del tick de FreeRTOS; comprobar `configTICK_RATE_HZ` antes de plantear tareas de 1 ms. No es la herramienta adecuada para conversiones cada 50 us a 20 kHz ni para seleccionar una ventana ON de pocos microsegundos.

Se conserva para telemetria y tareas lentas. La adquisicion revisada usa eventos MCPWM para despertar la tarea y observa el GPIO para comprobar la ventana; ADC continuo/DMA permanece como alternativa por evaluar. `adc_oneshot_read()` no se ha trasladado a una ISR.

### 5. Calibracion, ensayos y modelo

1. Calibrar CPR de salida y comprobar signo y repetibilidad del encoder.
2. Verificar corriente con instrumento externo, revisando ganancia, cero y validez del sensado antes de ajustar el lazo.
3. Repetir niveles alrededor del arranque, por ejemplo entre 20 % y 40 %, y ensayar ambos sentidos con pausas en cero y carga documentada.
4. Separar ensayos de identificacion y validacion; registrar alimentacion medida, carga, sentido y condiciones de cada prueba.
5. Identificar primero PWM/tension a velocidad. Incorporar corriente cuando sea fiable; comparar funciones de transferencia y un modelo en espacio de estados con corriente y velocidad.
6. Ajustar el PI considerando retardos, saturacion y muestreo real. Mejorar validacion de comandos PID, proteccion de sobrecorriente y comportamiento ante perdida de comunicacion.
7. Calibrar corriente frente a torque de salida antes de usar consignas en N.m. Los lazos de velocidad y posicion permanecen pendientes.

## Validacion del sensado sobre el motor: pendiente

1. Compilar y flashear esta revision, actualizar los tres sketches y conectar StepExperiment a 460800 baudios. Esperar al menos un segundo para que las frecuencias observadas se calculen. Mantener CPR de salida en cero mientras no se conozca su resolucion.
2. Con `SET,0`, comprobar que Processing indique PWM apagado y corriente no medida, sin trazar una linea falsa de 0 A. Registrar ADC crudo y voltaje de diagnostico; una lectura cruda cero con voltaje calibrado positivo no es una medicion de offset.
3. En OPEN, repetir el programa del experimento anterior para comparar condiciones, documentando tension de fuente, carga y sentido. Observar si los estados en 40 %, 50 % y 30 % pasan a lectura ON valida o explican el rechazo. Exportar un archivo nuevo; conservar el CSV historico intacto.
4. Medir el periodo de GPIO27/GPIO26: objetivo 100 us. Comparar con `pwm_frequency_observed_hz` cercano a 10000 y `acquisition_rate_observed_hz` cercano a 1000. Contrastar incrementos de `adc_sample_count`, `current_valid_count`, `sample_attempt_count` y `sample_missed_count`; no exigir 1000 corrientes validas/s si el programa contiene PWM cero o ventanas demasiado breves.
5. Observar PWM e IPROPI simultaneamente en varios niveles. Revisar `adc_read_us` y `sample_phase_us`, los margenes de 5 us y 2 us y el modo real de control/recirculacion. Al 100 %, la fase sin flanco es -1, pero debe poder haber muestras validas de nivel continuo. Si persiste `CURRENT_ADC_FLOOR` con movimiento, comprobar IPROPI directamente, su resistencia, alimentacion, masa y rango util del ADC; el codigo no recupera una senal ausente o fuera de rango.
6. Comparar corriente frente a un instrumento externo con corriente conocida en varios puntos. Verificar la ganancia 2.49 V/A y el cero antes de modificar la constante de offset. Una medicion de corriente de la fuente puede diferir de la corriente del devanado durante PWM: comparar magnitudes equivalentes.
7. Solo despues de validar el sensado, establecer un punto manual dentro de `LIMIT`, confirmar corriente valida reciente y activar PID en el mismo sentido. Comprobar primero que el intento de activarlo desde STOP se rechace. Ajustar ganancias progresivamente y revisar cualquier `ERR,CURRENT_SENSE` antes de continuar.

La correccion quedara validada cuando el nuevo registro concuerde con la medida fisica de periodo y corriente, explique las muestras rechazadas y muestre lecturas reproducibles en los niveles donde antes aparecian ceros. Hasta entonces, los cambios resuelven defectos de software identificados y hacen observable el fallo; no demuestran que el sensado fisico este calibrado.

## Referencias tecnicas

- [Texas Instruments: DRV8876, sensado IPROPI, tiempos y modos del puente](https://www.ti.com/lit/ds/symlink/drv8876.pdf).
- [Espressif: restricciones de ADC oneshot](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc/adc_oneshot.html). Consultar la version correspondiente a ESP-IDF 5.4.4 antes de modificar APIs.
- Evidencia experimental: CSV enlazado al inicio de la seccion del experimento. Las hipotesis sobre el sensado y el reloj requieren validacion fisica.
