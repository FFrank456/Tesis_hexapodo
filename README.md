<!-- Estado de la revisión 5: promedio de bloques DMA; pendiente de flash y validación física. -->
# ControlCurrentN20

## Estado actual

**La revisión 5 calcula internamente el promedio de cada bloque DMA de 200 muestras y lo presenta como medida principal en Processing.** Mantiene PWM de 10 kHz, ADC continuo de 200 kS/s y el EMA adicional de alpha 0.35 por muestra. No hay selección ni sincronización ON/OFF. Esta revisión está preparada en el código; su flash y validación física siguen pendientes. La última comprobación física documentada corresponde a la versión 4, en STOP, el 18/09/2026.

El proyecto funciona como banco de ensayos en lazo abierto. El lazo PID permanece bloqueado hasta validar el sensado y la temporización. No hay un modelo dinámico validado ni un controlador sintonizado para la planta.

### Conexiones y mando

| Elemento | Configuración |
|---|---|
| Driver | DRV8876, PMODE=1: avance/coast y reversa/coast |
| PWM horario / antihorario | GPIO27 / GPIO26; mando firmado de -100 a +100 % |
| PWM | 10 kHz nominales; reloj de 20 MHz y período de 2000 ticks |
| CS al ADC | GPIO32, ADC1_CH4; 12 bits y atenuación de 12 dB |
| Encoder A/B | GPIO18 / GPIO19; conteo x4, antes de la reductora 100:1 |
| Serial | 460800 baudios; telemetría nominal de 100 Hz |

En cada sentido se aplica PWM a una entrada del puente y se mantiene la otra en cero. STOP coloca ambas entradas en cero: coast. El encoder funciona después de reconectarlo; faltan los conteos por vuelta para convertir a RPM, por lo que los ensayos se expresan en conteos x4/s.

## Promedio de bloques DMA: revisión 5

La adquisición recibe 200 muestras cada 1 ms nominal: 5 us entre conversiones, aproximadamente 20 muestras por período PWM y diez períodos por bloque. Todas las muestras del bloque intervienen, incluidas ON, OFF y STOP. Un cambio de PWM puede quedar dentro de un bloque: el promedio representa ese intervalo completo.

Cada código ADC se convierte individualmente mediante la tabla de 4096 voltajes obtenida con line-fitting del SDK. Se calcula:

```text
V_procesado[k] = 0                       si raw[k] == 0
V_procesado[k] = V_calibrado_SDK(raw[k])  si raw[k] > 0

adc_raw_mean     = suma(raw[k]) / 200
adc_voltage_mean = suma(V_procesado[k]) / 200
current_mean_a   = adc_voltage_mean / 2.49
```

La media se calcula después de convertir cada muestra: no se convierte solamente el código ADC promedio. `adc_raw_mean` es un diagnóstico en cuentas ADC; `adc_voltage_mean` está en voltios y `current_mean_a` en amperios equivalentes de CS.

La asignación de cero para `raw==0` se hace antes del promedio y del EMA. El voltaje SDK de la última muestra se conserva como diagnóstico en `adc_voltage`; puede ser 0.142 V aunque su contribución procesada sea cero. No se resta 0.142 V a todos los códigos. Los códigos positivos conservan su calibración, incluso por debajo de 150 mV. Esta regla no recupera información bajo el rango útil del ADC ni demuestra corriente física nula.

Solo un bloque completo válido publica la media. Saturación, error de calibración, datos mal formados, antigüedad excesiva o pérdida DMA invalidan el bloque: las medias se publican como `NaN` y `dma_block_samples=0`. No se entrega un promedio parcial como si representara las 200 muestras. La invalidez queda explícita en estado y calidad. Las pérdidas DMA limpian y reinician las colas y provocan STOP desde la tarea de control.

**Esta revisión calcula una media aritmética de CS, no extremos ni una reconstrucción trapezoidal por pulso.** No calcula fase, área por ON, pendiente, mínimo/máximo de pulso ni una captura rápida de la forma de onda. Tampoco aplica un filtro de amplitud que elimine automáticamente los picos de conmutación.

### EMA adicional

Se conserva el filtro sobre cada muestra válida:

```text
y[k] = y[k-1] + 0.35 * (x[k] - y[k-1])
```

La primera muestra válida inicializa el filtro. Los cambios de PWM, sentido, STOP y los errores no reinician su memoria; durante una invalidez no se presenta el valor antiguo como una medida fresca. La media de bloque y el EMA son señales distintas: la media es la salida principal de la revisión 5 y el EMA continúa como diagnóstico adicional.

A 200 kS/s, alpha 0.35 equivale a una constante de tiempo aproximada de 11.6 us. Por ello, el último valor EMA de un bloque puede conservar rizado de la portadora; no equivale al promedio de las 200 muestras ni a aplicar el EMA a 1 kHz.

### Alcance físico de la medida

La salida es **CS promedio**, escalada con una ganancia supuesta de 2.49 V/A. No es automáticamente corriente media del devanado o torque. Durante coast, IPROPI no observa toda la corriente de recirculación; promediar los intervalos observados y OFF no recupera esa corriente. Referencia: [DRV8876, sección 7.3.3.1](https://www.ti.com/lit/ds/symlink/drv8876.pdf).

Quedan pendientes la calibración de voltaje ADC con referencias conocidas y la validación de la ganancia V/A con una referencia de corriente adecuada. Para comparar con el osciloscopio debe usarse la media de CS sobre una ventana comparable, preferentemente un número entero de períodos PWM; la media ON, el pico, RMS y una instantánea EMA representan magnitudes diferentes.

## Processing y protocolo serial

Los tres sketches usan la media de bloque como medida principal cuando reciben `sensing_version=5`, y conservan el EMA y los diagnósticos. Admiten registros anteriores sin reinterpretar sus instantáneas como promedios de bloque; un firmware anterior no proporciona los nuevos campos.

La trama `DATA` conserva sus primeros 48 campos e incorpora cuatro al final: **52 campos en total**, contando `DATA` como índice cero. La versión de sensado sigue en el índice 44.

| Índice | Campo añadido | Significado |
|---|---|---|
| 48 | `adc_raw_mean` | Media de las 200 cuentas ADC; diagnóstico |
| 49 | `adc_voltage_mean` | Media de voltajes calibrados individualmente, asignando cero a raw cero, en V |
| 50 | `current_mean_a` | `adc_voltage_mean / 2.49`, en A equivalentes de CS |
| 51 | `dma_block_samples` | 200 si la media corresponde a un bloque válido; 0 si es inválido |

Para un bloque inválido, los tres promedios son `NaN`. Los campos anteriores conservan su significado:

- `current_raw_a`: última muestra procesada del bloque dividida por 2.49. No es el promedio nuevo.
- `current_ema_a` y `adc_voltage_ema`: estado del EMA al final del bloque, en A equivalentes y V respectivamente.
- Estado 9 o 15 y `current_valid=1`: dato procesado válido; no certifica calibración física del devanado. El estado 15 describe cero asignado en la muestra correspondiente y no permite deducir que todas las muestras del bloque sean cero.
- `current_valid_count`: conversiones aceptadas. Los descartes de calidad se contabilizan en `dma_rejected_quality`; `dma_rejected_window=0`, porque no hay ventanas ON/OFF.
- `adc_sample_rate_hz=200000`, `current_ema_alpha=0.35`, `sample_phase_us=-1` y `dma_phase_validated=0`: la fase no se utiliza.
- `sample_time_us`: recepción ISR del bloque, no instante exacto de apertura del ADC. `adc_read_us=1000` representa la duración nominal del bloque.

**El cálculo de bloques es interno, nominalmente a 1 kHz. La telemetría a 100 Hz envía el último resumen disponible; no registra todos los promedios de 1 ms ni transmite las 200 muestras.** Sus tiempos y tasas deben distinguirse de la tarea que consulta las instantáneas. El CSV de Processing incorpora los campos nuevos y mantiene los huecos en medidas ausentes. No permite reconstruir la forma de onda de 10 kHz ni recuperar los bloques no enviados.

- [Processing manual](processing/ControlCurrentN20_Processing4/ControlCurrentN20_Processing4.pde): campo numérico PWM, deslizador y gráfica `abs(PWM)/100` de 0 a 1. `SHOW_EMA=false` oculta el filtro en pantalla, no lo desactiva en firmware.
- [Processing CurrentPID](processing/ControlCurrentN20_CurrentPID/ControlCurrentN20_CurrentPID.pde): interfaz disponible; el firmware sigue rechazando activar PID.
- [StepExperiment](processing/ControlCurrentN20_StepExperiment/ControlCurrentN20_StepExperiment.pde): ejemplo 0..100..0 %, pasos de 5 % cada segundo, ejecutado por ESP32; 41 etapas, límite de 48 etapas y 120 s.

## Verificación física y pendientes

La revisión 5 aún necesita flash y prueba física. Los ensayos descritos a continuación son de versiones anteriores y no validan el nuevo promedio.

Verificación de esta revisión: firmware compilado con ESP-IDF 5.4.4 (`build-codex/build-dma-mean.log`), tres sketches Processing compilados y pruebas C/Java aprobadas. Las pruebas comprueban media de las 200 muestras, asignación de cero, independencia del orden para la media, rechazo del bloque completo ante saturación, datos antiguos y pérdida DMA; también selección de la media v5, compatibilidad anterior y exportación CSV. Binario generado: `build-codex/ControlCurrentN20.bin`. No se flasheó ni se accionó el motor durante esta actualización.

La [comprobación del 18/09/2026](build-codex/integration-preflight-20260918-183939.txt) registró versión 4 en STOP, ADC alrededor de 200 kS/s, consulta alrededor de 1 kHz, sin pérdidas DMA registradas. Raw cero se mostraba como corriente procesada cero mientras el diagnóstico SDK conservaba 0.142 V. No se ejecutó un barrido de carga en esa comprobación. Ese resultado en STOP no descarta los descensos anteriores de consulta hacia 700 Hz bajo otros ensayos: la temporización sigue necesitando verificación durante movimiento.

Pendientes:

1. Flashear y comprobar medias v5 frente al osciloscopio, con factores de sonda/canal y acoplamiento DC confirmados.
2. Validar código ADC frente a voltajes DC conocidos y ganancia física CS frente a corriente de referencia.
3. Verificar tasas, calidad y latencia durante los escalones; distinguir adquisición DMA, consulta y telemetría.
4. Calibrar conteos por vuelta del encoder y diseñar un registro adecuado para identificar la dinámica, sin asumir que el CSV de 100 Hz contiene toda la información de 1 kHz.
5. Identificar y validar el modelo antes de habilitar y sintonizar el lazo de corriente.

Código: [adquisición](main/current_sense.c), [parámetros](main/current_sense.h). Las estrategias anteriores, incluidos oneshot y ventanas de muestreo, están en el [historial](README_HISTORIAL.md) y no describen el firmware actual.

## Ensayos anteriores: contexto y resultados

Los informes previos al tratamiento de raw cero conservaron una base SDK de 0.142 V, equivalente a 0.057 A bajo la conversión utilizada entonces. Ese tratamiento fue sustituido en v4 por cero asignado antes del EMA. Asimismo, el encoder 0..1 de los primeros ensayos se debió a que estaba desconectado, según confirmó el usuario; ya no se considera un fallo pendiente de PCNT. Informes históricos: [prueba DMA inicial](build-codex/dma-serial-probe-20260915-102838-analysis.md), [EMA alpha 0.1 antes de v4](build-codex/ema01-probe-20260915-104848-analysis.md) e [historial del proyecto](README_HISTORIAL.md).

### Barridos iniciales v4: 82 etapas

[Informe y capturas seriales](build-codex/many-pwm-v4-analysis.md). Barridos 0..100..0 % y 0..-100..0 %, en pasos de 5 %, un segundo por etapa y STOP entre ambos. Incluidos los reposos: 9206 tramas y 18.408 millones de conversiones procesadas/aceptadas, sin desbordamientos DMA ni descartes.

El ADC se mantuvo alrededor de 200 kS/s. La consulta fue cercana a 1 kHz en el primer sentido y 700 Hz en el segundo, con 13170 notificaciones omitidas; no atribuirlo al sentido sin aislar la causa. Cero asignado y EMA volvieron a cero en STOP, conservando 0.142 V SDK. El encoder estaba desconectado en estos registros. Las instantáneas EMA no aumentaron monótonamente con PWM y no equivalen al promedio de bloque añadido en v5.

### Encoder conectado — 15/09/2026, 15:40

El usuario confirmó que el encoder estaba desconectado antes y se repitieron 82 escalones: [resultados y logs](build-codex/encoder-connected-v4-analysis.md).

- Encoder: +276097 conteos CW y -273618 CCW. A 100 %, +13857/-13908 conteos x4/s; todavía sin CPR para RPM.
- Arranque sostenido a 35 % en ambos sentidos. Al descender, seguía girando a 20 %, casi detenido a 15 % y detenido a 10 %.
- Con alpha 0.1, a 100 %: 47.38/47.56 mA equivalentes de CS EMA. A 20 % de subida sin movimiento: 33.80/33.22 mA equivalentes. No son amperios calibrados del devanado ni medias DMA v5.
- 19.012 millones de conversiones procesadas y aceptadas sin pérdidas DMA ni descartes; consulta alrededor de 700 Hz.
- Dos intentos iniciales sin ACK no ejecutaron etapas y se excluyeron. Los válidos usaron envío fragmentado de SEQ y confirmación ACK. STOP comprobado entre secuencias y al terminar; no se flasheó durante la repetición.

### Carga desprendida — 15/09/2026, 16:08

Se ejecutaron 82 escalones con v4 y **alpha 0.35 observado**, frente a 0.1 del ensayo anterior. El usuario informó que **la carga se desprendió**. Ambos barridos habían terminado en STOP. El [informe y los registros](build-codex/loaded-v4-analysis.md) son diagnósticos, no una identificación con carga constante.

En CW se observó menor velocidad y mayor CS en niveles intermedios, seguido de recuperación cercana a la referencia a 80–100 %, compatible con cambio de carga. El instante de desprendimiento no se conoce. La adquisición mantuvo unos 200 kS/s sin pérdidas DMA.

### Carga fijada — 15/09/2026, 16:29

El usuario confirmó carga fijada y contenida. Se completaron 82 escalones, con ACK y STOP final, v4 y alpha 0.35: [informe y datos](build-codex/fixed-load-v4-analysis.md). La carga descrita después fue una botella de agua de 3 L colocada en el centro de una plataforma circular asociada al eje; eso no demuestra un par resistente constante ni permite deducir su inercia sin geometría y condiciones de apoyo.

A 100 %: CW, 355.38 mA equivalentes de CS EMA y +8498 conteos/s; CCW, 264.44 mA equivalentes y -10165 conteos/s. Respecto de +13857/-13908 sin esa carga, la velocidad cayó aproximadamente 38.7 % y 26.9 %. Entre 50–65 % hubo giro irregular: no se justifica un modelo lineal único en todo el rango. La comparación de corriente está limitada por el cambio de alpha 0.1 a 0.35, la calibración pendiente y las instantáneas de telemetría.

Se procesaron y aceptaron 18.975 millones de conversiones sin pérdidas DMA ni descartes. La consulta estuvo cerca de 700 Hz. No se modificó ni flasheó firmware durante esa repetición.

El [análisis de identificación](build-codex/fixed-load-identification.md) contiene ajustes estáticos preliminares de velocidad en 70–100 %: `CW: -14573 + 228.5*PWM_%` y `CCW: 13251 - 235.7*PWM_%`, con RMS de 582 y 471 conteos/s. Esos ajustes no son una función de transferencia dinámica ni una sintonía de controlador. Hay umbral de arranque, histéresis, fricción y asimetría de sentido. La calibración y los ensayos repetidos siguen pendientes antes de aceptar un modelo local.
