# Revisión de MATLAB, Simulink y preparación del ESP32

Fecha: 27/09/2026. Revisión de las nueve páginas de N20_identificationDATAS.pdf, la estructura XML del archivo SimulinkController.slx y las fuentes locales de firmware y Processing. No se ejecutó Simulink ni se accionó la planta. Los resultados numéricos siguientes se calcularon con álgebra de polinomios en NumPy a partir de los coeficientes impresos en el PDF.

## Dictamen

El desarrollo permite avanzar hacia una implementación por etapas, pero la cascada y sus ganancias actuales no están listas para activarse conjuntamente en el ESP32. Hay dos conexiones que omiten la dinámica interior, un modelo de posición que no conserva la cinemática a todas las frecuencias y ausencia de límites y muestreo digital en el modelo examinado.

## Controladores del PDF

| Control | Expresión continua | Kp | Ki | Kd |
|---|---|---:|---:|---:|
| Corriente, bloqueado | (1.718 s + 6343)/s | 1.718 | 6343 | 0 |
| Corriente, libre | (8.832 s + 80090)/s | 8.832 | 80090 | 0 |
| Velocidad | (0.0284 s + 0.01392)/s | 0.0284 | 0.01392 | 0 |
| Posición | 14290/s | 0 | 14290 | 0 |

El bloque llamado PID rad es un integrador puro. No contiene término proporcional ni derivativo. El PI de corriente vigente en las fuentes usa Kp=504.9, Ki=7462, Kd=0, por lo que difiere del PDF. La nueva Ki libre es 10.73 veces la actual.

Unidades con la identificación actual: salida del PI de corriente en puntos porcentuales PWM y entrada en A equivalentes CS. El controlador de velocidad produce A equivalentes CS a partir de error en rad/s. El de posición produce rad/s a partir de error en rad. Si se usa duty de 0 a 1 en otro bloque, es necesaria una conversión explícita por 100. El archivo SLX convierte 45 grados a radianes antes del sumador; las conversiones inversas solo alimentan scopes y no son el fallo de las realimentaciones.

## Conexiones confirmadas en SLX

Se inspeccionó simulink/systems/system_root.xml:

- SID 12 (PI rads), salida ref_current, alimenta directamente SID 14 (Modelo velocidad). Sin embargo, best_model_rads fue identificado con corriente medida como entrada, no con referencia de corriente.
- SID 18 (PID rad), salida ref_rads, alimenta directamente SID 16 (Modelo posición). best_model_rad fue identificado con velocidad medida como entrada, no con su referencia.
- SID 4 (Current_model), salida current, solo alimenta el sumador de corriente y su scope; no alimenta el modelo de velocidad.
- SID 14, salida rads, alimenta la realimentación de velocidad y un scope; no alimenta el modelo de posición.

Por tanto, la posición simulada puede responder aunque el lazo de corriente no entregue la corriente solicitada. Los modelos no son los equivalentes de lazos interiores ya cerrados, por lo que estas conexiones no constituyen una reducción válida de la cascada mostrada en el PDF.

Primera corrección para evaluar las TF empíricas juntas: conectar current al modelo corriente→velocidad y su velocidad de salida al modelo de posición. Para la representación física, usar theta_dot=omega y poner el filtrado/medición del encoder aparte. El modelo velocidad identificado ya usa una medida con ventana de 50 ms: añadir esa ventana otra vez sin redefinir el modelo duplicaría el efecto del sensado.

## Modelo de posición y estabilidad

La relación física velocidad del eje de salida→posición del mismo eje es 1/s, con condición inicial theta(0). La TF empírica del PDF tiene numerador y denominador del mismo grado:

(0.04939 s² + 10.5 s + 317)/(s² + 317 s + 0.01188)

Tiene paso directo 0.04939, así que ante un salto de velocidad predice un salto instantáneo de posición. En una banda limitada puede aproximar la medida del ensayo, pero ese paso directo y sus ceros afectan radicalmente la sintonización de posición. El ajuste del 100 % en un registro no justifica utilizarlos como cinemática física.

Con un lazo de velocidad ideal de ganancia uno, el controlador 14290/s y la posición 1/s generan s²+14290=0: polos ±j119.54, sin amortiguamiento. Con los modelos interiores suministrados aparece inestabilidad:

Ti = feedback(Ci*Gi,1)

Tw = feedback(Cw*Ti*Gw,1)

Ttheta = feedback((14290/s)*Tw*(1/s),1)

Dos polos de Ttheta son aproximadamente +35.2517 ± j47.6337 s⁻¹. Esta es una comprobación algebraica del modelo lineal corregido, no un ensayo de la planta ni una simulación con saturación. Es suficiente para descartar cargar las tres ganancias directamente como una cascada validada.

Para reproducir en MATLAB con los coeficientes del PDF:

```matlab
s = tf('s');
Gi = (0.001184*s^2 + 1.441*s + 2.325)/(s^2 + 904.4*s + 3996);
Ci = 8.832 + 80090/s;
Gw = 110700/(s^2 + 110.1*s + 172.3);
Cw = 0.0284 + 0.01392/s;
Ti = feedback(Ci*Gi,1);
Tw = feedback(Cw*Ti*Gw,1);
Ttheta = feedback((14290/s)*Tw/s,1);
pole(Ttheta)
isstable(Ttheta)
```

Propuesta de diseño: empezar posición con un controlador proporcional que entregue una velocidad limitada, y sintonizarlo sobre el lazo de velocidad cerrado. Añadir integral solo si el error y las pruebas lo requieren. No se proponen ganancias nuevas definitivas con estos registros.

## Límites y tiempo

El SLX contiene bloques LTI continuos; no se encontraron bloques de saturación, control PID discreto, retardo explícito ni retención de orden cero. Su solver guardado es VariableStepAuto.

- Definir máximo PWM, referencia CS máxima, velocidad máxima de salida, rampa de referencia y tolerancia de posición. El máximo admitido por el comando IREF (1 A eq) no constituye un límite de corriente física validado.
- Implementar antiwindup en los lazos con integral, incluyendo el efecto de límites aguas abajo. Un límite en PWM puede impedir alcanzar la referencia CS y acumular integral en el lazo exterior.
- El step(feedback(Ci*Gi,1)) del PDF usa por defecto un escalón unitario: aquí 1 A equivalente. La ganancia DC de Gi exigiría unos 1719 % PWM en el modelo lineal para mantener 1 A. Es útil como respuesta lineal normalizada, pero no demuestra seguimiento de 1 A con el motor y PWM limitado.
- Sintonizar primero corriente, luego velocidad con el lazo de corriente cerrado, y finalmente posición con el lazo de velocidad cerrado. El interior debe ser sensiblemente más rápido que el exterior. Como objetivo inicial de diseño puede buscarse separación de 5–10 veces, verificando márgenes y respuesta con retardos reales.
- A partir de las TF del PDF, la banda de -3 dB del lazo corriente aislado es aproximadamente 20.28 Hz; la de velocidad aislada es 5.82 Hz. La razón es 3.49, antes de incorporar el lazo interior a velocidad. Estas cifras describen esos modelos y no una medición en el ESP32.
- El registro original de corriente a 100 Hz no resuelve bien el polo de 1.11 ms del modelo Gi. Las nuevas ganancias rápidas deben contrastarse con registro de bloques y osciloscopio.
- Fijar y medir los periodos reales antes de discretizar. Como arquitectura candidata: corriente ligada a bloques nuevos de 1 ms, velocidad cada 10 ms y posición cada 20 ms. Son objetivos de implementación por verificar, no tasas actualmente garantizadas ni bandas de control. La ventana de velocidad de 50 ms sigue importando aunque se ejecute su controlador más seguido.

## Estado de las fuentes ESP32

| Elemento | Estado revisado |
|---|---|
| PWM | MCPWM 10 kHz; avance/coast y reversa/coast |
| CS | ADC continuo 200 kS/s; promedio de 200 muestras, nominal 1 ms; sin EMA |
| PI corriente | Implementado en current_pi.h y main.c; Kp=504.9, Ki=7462, Kd=0 |
| Actualización PI | Solo bloques nuevos, dt real, antiwindup y parada ante fallo de sensado |
| Periodicidad control | Consulta nominal 1 kHz; ensayos anteriores observaron aproximadamente 500–600 Hz; no hay una tasa fija garantizada |
| Encoder | 2800 cuentas/vuelta de salida; rad y rad/s calculados en ESP32; ventana de velocidad 50 ms |
| Control velocidad/posición | No implementado: modos existentes OPEN y PID de corriente |
| Dirección y corriente | Referencia CS no negativa 0–1 A eq, dirección separada; invertir en PI requiere STOP |
| Arranque | OPEN, PWM cero, referencia cero |
| Telemetría | Protocolo v6 de 59 campos, nominal 100 Hz |
| CurrentPID Processing | Ganancias antiguas; Ki editable solo hasta 12000; PWM máximo mostrado inicialmente 40 % |

Para una cascada bidireccional faltan comandos de velocidad/posición, estados de los controladores, límites y rampas, tratamiento de inversiones y frenado, detección de encoder inválido y transiciones entre modos. El coast actual no produce frenado activo: una orden de velocidad/corriente negativa no puede trasladarse directamente a la interfaz de referencia no negativa. La media CS tampoco proporciona automáticamente el signo de torque ni observa la recirculación coast.

Processing debe continuar como visualizador y emisor de consignas. Toda la ejecución de la cascada y el tratamiento del encoder deben permanecer en el ESP32. Hay que ampliar el protocolo con referencias, errores y saturaciones de los nuevos lazos, manteniendo limitada la frecuencia serial.

## Reproducibilidad

La página 1 del PDF muestra una variable de datos no reconocida. Los resultados posteriores pueden proceder de secciones ejecutadas previamente; no quedan invalidados por ese mensaje, pero el documento no es una ejecución limpia de principio a fin. El script de preparación debe cargar el MAT explícitamente y convertir el tiempo una sola vez. En la revisión anterior, los MAT libre y bloqueado contenían tiempos en unidades diferentes.

## Siguiente paso recomendado

Corregir conexiones y cinemática, añadir límites y representación discreta, volver a sintonizar y validar la cascada completa. Después trasladar al ESP32 por etapas: corriente, velocidad y posición. Conservar los límites de sentido, la calidad del sensado y las protecciones durante cada prueba. El controlador externo de velocidad puede identificarse de forma más directa con referencia CS→velocidad una vez cerrado y validado el PI interior.

Fuentes técnicas: [MathWorks: diseño de cascadas](https://www.mathworks.com/help/control/ug/designing-cascade-control-system-with-pi-controllers.html), [antiwindup](https://www.mathworks.com/help/simulink/examples/anti-windup-control-using-a-pid-controller.html), [PID discreto](https://www.mathworks.com/help/simulink/slref/discretepidcontroller.html).
