# Visualización y exportación del controlador — 28/09/2026

Contrato histórico v2. La revisión actual conserva estas columnas y añade posición en CSV v3: ver [control_posicion_20260928.md](control_posicion_20260928.md).

Archivo: processing/ControlCurrentN20_CurrentPID/ControlCurrentN20_CurrentPID.pde. Esta revisión solo cambia Processing y sus pruebas; no cambia las ganancias, el firmware ni la frecuencia del control. No se flasheó ni se enviaron órdenes a la planta al verificarla.

## Distribución

- Barra lateral de 294 px: vista Corriente/Velocidad, referencia, sentido, límites y botón explícito de activación. Cambiar de pestaña no envía comandos. En Corriente puede observarse también el PI interior mientras el modo activo sea SPEED.
- Ajustes y diagnóstico oculta las ganancias y el PWM manual cuando no se necesitan. Las referencias editadas no se aplican hasta pulsar el botón correspondiente.
- STOP permanece en la cabecera; Espacio también detiene en coast.
- Seguimiento con referencia y medida, error y PWM. En velocidad se añade una gráfica de corriente interior. Ejes autoajustados, tiempo relativo al último dato del ADC, última ventana de 20 s. No se unen huecos mayores de 100 ms, tiempos repetidos ni medidas inválidas.
- El almacenamiento de captura es independiente de la vista: 60000 muestras como máximo. Al llenarse se conserva el ensayo capturado y continúa la gráfica. Exportar guarda una instantánea sin detener el motor ni borrar la captura. Nuevo registro borra los datos acumulados.

## Indicadores

Se calculan sobre los últimos 5 s disponibles del modo continuo actual y segmento temporal. Un reinicio del reloj o una reconexión inicia otro segmento. En OPEN no se calculan errores de seguimiento genéricos.

Con e = referencia - medida y pesos dt del ESP32:

- RMSE = raíz(suma(e² dt) / suma(dt)).
- MAE = suma(abs(e) dt) / suma(dt).
- Error máximo = máximo abs(e) de los intervalos válidos.
- Tiempo en techo = 100 × duración con abs(PWM) >= techo - 0.1 puntos porcentuales / duración con techo conocido. El techo procede de ACK,LIMIT, no de una caja todavía sin aplicar. Tras reconectar/reiniciar queda desconocido hasta otra confirmación.

Se usa el error recibido/calculado en el extremo final del intervalo, sin interpolar transitorios de referencia. Se descartan intervalos no positivos, mayores de 100 ms o con extremos inválidos. Se muestra la suma de segundos válidos; «--» significa que no hay información suficiente. La referencia SPEED de 20 Hz se mantiene entre tramas y conserva su timestamp de origen. Las estadísticas son aproximaciones desde telemetría, no todos los ciclos del firmware.

En pantalla, las métricas de corriente se expresan en mA equivalentes CS; las de velocidad en rad/s. No se declara tiempo de establecimiento ni sobreimpulso: los indicadores actuales no segmentan automáticamente escalones ni evalúan estabilidad.

## CSV único v2

Ubicación: exports/control_n20_v2_YYYYMMDD_HHMMSS_mmm.csv, dentro del sketch. UTF-8, separador coma, valores decimales con punto y campos entre comillas. Es el mismo esquema para PID, SPEED y OPEN; puede incluir cambios de modo en un mismo archivo. La exportación se ejecuta en segundo plano con lotes de 256 filas y conserva las métricas usando el historial completo de la instantánea.

| Columnas | Interpretación |
|---|---|
| csv_schema_version, record_index | Versión 2 e índice de fila desde cero |
| segment_id, segment_time_s | Segmento por reinicio/reconexión y segundos desde su primera muestra ADC |
| adc_time_us, encoder_time_us | Tiempos originales del ESP32; no confundirlos con pc_time_ms |
| control_mode | PID, SPEED u OPEN |
| loop_unit | A_eq_CS para PID; rad/s para SPEED; vacío en OPEN |
| loop_reference, loop_measured, loop_error, loop_valid | Variables del lazo activo; vacío para no disponible |
| metrics_valid_s | Tiempo válido acumulado en la ventana de hasta 5 s |
| rmse_5s, mae_5s, abs_error_max_5s | Métricas en las unidades de loop_unit (corriente en A eq., no mA) |
| pwm_at_confirmed_limit_pct_5s | Fracción temporal en el techo PWM conocido, en porcentaje |
| pwm_limit_max_percent | Último máximo confirmado por ACK; NaN si se desconoce |
| current_mean_a, current_ref_a, duty_percent | Corriente interior, su referencia y PWM firmado en ambos modos |
| kp, ki, kd | Ganancias interiores recibidas, guardadas por muestra |
| speed_* | Extensión SPEED original, incluyendo ganancias, límite CS y timestamps |

Se conservan las columnas de diagnóstico ADC/DMA y encoder. En CCW, loop_reference y loop_measured de velocidad están firmadas respecto al eje; loop_error es su diferencia. speed_error_rad_s conserva el error proyectado que envía el firmware y puede tener signo opuesto. Para MATLAB, separar modos/segmentos y usar unidades/timestamps antes de crear iddata; no tratar un registro mixto como un único ensayo SISO de unidades constantes.

## Comprobación

Preprocesado y compilación con Processing/Java. TestDashboard prueba errores conocidos, ponderación temporal, datos inválidos, separación de modos/reinicios, límite confirmado, CSV escrito y releído bajo configuración regional española, 60000 filas sin pérdida silenciosa y pestañas que no accionan. TestCurrentPI y TestSensingProcessing conservan las regresiones de comandos y lectura. RenderSpeed genera vistas de velocidad, corriente y ajustes con datos simulados a 1366×768; se revisaron visualmente.
