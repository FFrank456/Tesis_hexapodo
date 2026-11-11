# Corriente y velocidad — 28/09/2026

Este documento describe la etapa de dos lazos. Posteriormente se incorporó posición: [estado y uso de los tres lazos](control_posicion_20260928.md). El dashboard actual usa CSV v3 y extensiones SPEED/POSITION a 10 Hz nominales.

Implementación solicitada: PI de corriente libre (8.832, 80090) y PI de velocidad (0.0284, 0.01392), tomados de N20_identificationDATAS.pdf y documentados en matlab/revision_cascada_20260927.md. No se modifica el archivo SLX ni se implementa posición. Se adapta el Processing CurrentPID existente.

## Ejecución

En main/current_pi.h se actualizan las ganancias iniciales. main/speed_pi.h implementa la integración con dt real y antiwindup. main/main.c conecta la velocidad del encoder al PI exterior, su salida limitada a la referencia del PI interior y la media DMA a la realimentación interior. Periodo nominal exterior de 10 ms, ventana encoder de 50 ms; el interior actúa sobre bloques nuevos disponibles, con tiempo del ADC y sin volver a integrar bloques repetidos. No se ejecuta control desde UART/Processing.

El cambio de modo SPEED verifica ambas medidas y sentido; cada actualización comprueba calidad y antigüedad. Referencia cero produce coast y borra integradores. No hay torque firmado, freno activo ni inversión automática. Las referencias CS siguen siendo A equivalentes sin calibración física completa. La elección de 32 rad/s como rango de comandos no garantiza alcanzar ese valor con el techo PWM/carga.

## Verificación sin hardware

- GCC host: test_current_pi.c, actualizado a las ganancias MATLAB. Comprueba cálculo con 1 y 2 ms, duplicados, huecos, saturación, recuperación, tracking y entradas inválidas.
- GCC host: test_speed_pi.c. Comprueba ganancias, intervalos de 10/12 ms, rechazo de huecos de 50 ms, antiwindup propio y por límite interior, recuperación, salida no negativa, reset y entradas inválidas. Comparaciones de coeficientes float usan tolerancia numérica.
- Processing preprocesado y compilado con Java de Processing. TestCurrentPI verifica ganancias, comandos con configuración regional española, orden de límites/activación, rechazo de activación sin extensión reciente, cambio de referencia sin reactivar, bloqueo de inversión, cambio SPEED→PID y CSV con timestamps/ganancias por muestra.
- TestSensingProcessing verifica compatibilidad DATA v6, media DMA, huecos inválidos y encoder SI en los tres visualizadores.
- test_serial_schema.py verifica DATA de 59 campos sin cambios y SPEED v1 de 13 campos.
- RenderSpeed genera una vista con datos simulados; se inspeccionó su disposición visual. No abre puerto serial.
- Compilación ESP-IDF v5.4.4, destino ESP32, en build-codex. El binario es build-codex/ControlCurrentN20.bin.

No se ha flasheado, enviado órdenes al motor ni validado experimentalmente esta cascada. El usuario realiza el flasheo. Las pruebas anteriores con otras ganancias no certifican esta versión. El siguiente ensayo debe comprobar primero el PI interior y después seguimiento de velocidad, sentido del encoder, saturación, tiempos observados y desaceleración coast.

## Uso de la interfaz

Abrir processing/ControlCurrentN20_CurrentPID/ControlCurrentN20_CurrentPID.pde. Después de flashear, conectar y comprobar los valores recibidos: corriente Kp=8.832/Ki=80090; velocidad Kp=0.0284/Ki=0.01392. Las pestañas Corriente/Velocidad solo cambian la vista. Aplicar referencia / activar manda la referencia y activa el modo elegido, con los límites mostrados. CW/CCW selecciona sentido desde STOP. Ajustes y diagnóstico permite aplicar las ganancias del panel visible y PWM manual. Espacio/STOP detiene el motor en coast. Editar una caja no confirma su aplicación. La revisión visual y el CSV común se documentan en processing_dashboard_csv_v2.md.
