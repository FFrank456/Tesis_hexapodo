# Posición en cascada — 28/09/2026

Implementado en ESP32: posición → velocidad → corriente CS → PWM. Processing envía consignas, representa las medidas y exporta CSV; no ejecuta los controladores.

## Control y límites

- PI de posición confirmado por el usuario: `6.166 + 3192/s`, Kd=0. Núcleo en `main/position_pi.h`, integración en `main/main.c`.
- Actualización exterior nominal de 20 ms, con el intervalo real del encoder. Se conservan PI de velocidad 0.0284/0.01392 e interior de corriente 8.832/80090.
- Posición acumulada en rad respecto al cero del encoder: no se aplica módulo 2π, camino más corto ni homing. Consigna entre −10000 y +10000 rad.
- Valores iniciales: referencia de velocidad limitada a ±2 rad/s, rampa de referencia de 5 rad/s² y tolerancia de 0.02 rad. La rampa limita la consigna; no garantiza esa desaceleración física.
- Antiwindup por saturación, rampa y límite del lazo interior. Los límites de la interfaz se inician en 100 mA equivalentes CS y 40 % PWM. La corriente sigue pendiente de calibración física.
- Al entrar en la banda se manda coast y se borran los integradores. Con velocidad absoluta ≤0.1 rad/s durante 100 ms pasa a `HOLD`: coast vigilado, sin torque activo de mantenimiento. Retoma seguimiento si sale de ±0.04 rad (dos tolerancias).
- El signo de la salida selecciona CW/CCW automáticamente. Antes de invertir, manda coast y exige velocidad absoluta ≤0.1 rad/s durante 100 ms. No implementa frenado activo ni corriente negativa.
- Datos inválidos/antiguos del encoder, cambio de su referencia, hueco excesivo de control o fallo DMA detienen el control. Un encoder desconectado que reporte cero válido no queda detectado por esta comprobación.

## Uso en Processing

Abrir `processing/ControlCurrentN20_CurrentPID/ControlCurrentN20_CurrentPID.pde` después de cargar el firmware nuevo. Las pestañas Corriente, Velocidad y Posición solo seleccionan la vista.

1. Conectar y esperar telemetría. Pulsar STOP y esperar a que el eje esté detenido.
2. En Posición, **Cero encoder ESP32** establece el origen cuando se necesita. Se rechaza mientras haya control, PWM manual o secuencia activos.
3. Introducir objetivo en rad mediante su caja numérica, velocidad máxima, límite CS y techo PWM. En Ajustes están Kp/Ki, rampa y tolerancia.
4. **Aplicar referencia / activar** envía límites y objetivo y activa POSITION explícitamente. La entrada exige modo OPEN, salida detenida y medidas válidas. Cambiar ganancias o límites de posición exige STOP.
5. Durante POSITION puede cambiarse el objetivo: se reinician los integradores y se manda coast antes de retomar seguimiento. No es una transferencia sin discontinuidad.
6. Espacio/STOP detiene. Objetivo **0 rad significa volver al origen**, no STOP. Cambiar de pestaña no detiene ni cambia el modo activo.

## Serial

DATA v6 conserva 59 campos y añade modo `POSITION` (código 3). Frecuencia DATA nominal 100 Hz; SPEED v1 y POSITION v1 nominales 10 Hz cada una. La tasa real depende de la ejecución y UART. Los visualizadores antiguos ignoran las extensiones.

```text
PPI,6.166,3192
PLIMIT,2,5,0.02
LIMIT,0,40
ILIMIT,0.1
PREF,1.5708
MODE,POSITION
STOP
```

PREF captura el identificador del origen actual; después de poner a cero se requiere otro PREF. PPI/PLIMIT se rechazan durante POSITION. IREF/WREF no pueden sustituir referencias pertenecientes a los lazos exteriores. MODE,OPEN desde POSITION ejecuta parada. STOP conserva la consigna, pero requiere activación explícita para reanudar. No se guardan parámetros en flash.

```text
POSITION,1,encoder_time_us,reference_rad,error_rad,speed_reference_signed,kp,ki,integral_rad_s,max_speed_rad_s,accel_rad_s2,tolerance_rad,state,limited,control_time_us,encoder_reference_id
```

Estados: 0 seguimiento, 1 espera de inversión, 2 coast vigilado, 3 llegada. Las extensiones conservan sus propios tiempos; repetir sus campos en DATA/CSV no crea nuevas adquisiciones.

## Gráficas y CSV v3

La vista de posición muestra ángulo y objetivo, error, velocidad y referencia, corriente CS y referencia, y PWM. Conserva indicadores ponderados por tiempo del ESP32: RMSE, error absoluto medio/máximo y tiempo de PWM en el techo confirmado. Las unidades del error son rad. Un cambio del origen interrumpe la ventana de métricas.

El CSV común pasa a `csv_schema_version=3`, archivo `control_n20_v3_...csv`, para los tres modos. Conserva el contrato anterior de relojes y datos originales descrito en `processing_dashboard_csv_v2.md` y añade:

```text
position_telemetry_time_us,position_ref_rad,position_error_rad,position_speed_ref_rad_s,
position_kp,position_ki,position_integral_rad_s,position_max_speed_rad_s,
position_accel_rad_s2,position_tolerance_rad,position_state,position_limited,
position_control_time_us,position_reference_id
```

Las columnas genéricas de referencia/medida/error usan rad en POSITION. Se mantiene el límite de 60000 filas y la exportación por lotes. Referencias ausentes, antiguas o de otro origen no se utilizan como medidas válidas.

## Verificación y estado

Compilación ESP-IDF v5.4.4 correcta, binario `build-codex/ControlCurrentN20.bin`. Pruebas host del PI exterior cubren tiempos reales, rampas, saturación, antiwindup, llegada, histéresis, inversión y entradas inválidas. Pruebas Java cubren activación, comandos, formatos numéricos, exportación, métricas y compatibilidad. Esquemas DATA/SPEED/POSITION comprobados. Se renderizaron y revisaron las dos vistas de posición con datos simulados.

No se flasheó ni accionó el motor; el usuario realiza el flasheo. Estas pruebas verifican software, no estabilidad física. El diagnóstico previo de polos inestables con modelos históricos en `posicion_pendiente_20260928.md` sigue siendo relevante: los límites y la lógica coast no validan la sintonía. Debe verificarse la cascada completa, incluyendo la ventana de velocidad de 50 ms, límites y discretización, antes de considerar terminado el ajuste en planta.
