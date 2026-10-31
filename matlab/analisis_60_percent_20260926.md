# Comparación al 60 %: eje libre y bloqueado

Se contrastaron los vectores numéricos de los MAT con los CSV de origen. PWM, corriente media CS, velocidad y posición coinciden exactamente como vectores double en los MAT descomprimidos:

- `60_percent.mat`: `multistep_20260923_213949.csv`. El tiempo guardado en el MAT está convertido a segundos.
- `60_percent_stall.mat`: `multistep_20260923_214220.csv`. Conserva `sample_time_us` en microsegundos.

MATLAB no pudo iniciarse en este entorno por un error de sistema de archivos. Se analizaron las medidas con Python/NumPy y se reprodujo analíticamente la respuesta de las transferencias de la captura, usando sus coeficientes redondeados. No se ejecutó una nueva identificación TFEST.

## Datos y resultados

Ambos registros tienen 301 muestras, separadas exactamente 10 ms: 0 % durante 1 s, 60 % durante 2 s y una última muestra a 0 %. Las 301 medidas son válidas en cada registro; no hay desbordamientos DMA ni incremento de descartes de calidad. El ADC observa 200 kS/s, pero el archivo registra resúmenes a 100 Hz.

Medias y desviaciones estándar calculadas entre 2.40 s inclusive y 2.95 s exclusive, con tiempo relativo a la primera muestra:

| Magnitud | Eje bloqueado | Eje libre |
|---|---:|---:|
| Primera corriente con PWM 60 % | 107.962 mA eq | 107.129 mA eq |
| Media CS final | 108.307 mA eq | 34.261 mA eq |
| Desviación estándar final | 0.299 mA eq | 0.754 mA eq |
| Máximo registrado | 108.940 mA eq | 107.129 mA eq |
| Velocidad final media | 0.0041 rad/s | 22.4049 rad/s |
| Cambio total encoder | 3 cuentas | 17886 cuentas |

Las tres cuentas del ensayo bloqueado representan unos 0.00673 rad en la salida; no se observó giro sostenido. La media CS bloqueada es 3.161 veces la media libre. La caída de CS al aumentar la velocidad es coherente con el aumento de fuerza contraelectromotriz. No demuestra por sí sola una calibración de la corriente física.

## Transferencias de la captura

Entrada en puntos porcentuales PWM; salida en A equivalentes CS.

### Bloqueado

G(s) = (0.001795 s² + 0.04793 s + 0.1065) / (s² + 26.46 s + 58.99)

- Polos: -24.0023 y -2.45768 s⁻¹.
- Ceros: -24.2559 y -2.44607 s⁻¹.
- Ganancia estática: 0.00180539 A eq / punto porcentual.
- Paso directo: 0.001795 A eq / punto porcentual.
- Predice 107.700 mA inmediatamente tras el escalón y 108.323 mA en régimen.
- Ajuste reproducido con los coeficientes de la captura: 99.5542 %.

Los polos y ceros casi se cancelan. En este ensayo el modelo se comporta casi como una ganancia constante. El 99.6 % describe bien el pulso observado; no identifica de forma fiable dos dinámicas eléctricas. Una ganancia estática local o un modelo más sencillo constituye una comparación necesaria antes de conservar cinco parámetros.

### Libre

G(s) = (0.001184 s² + 1.441 s + 2.325) / (s² + 904.4 s + 3996)

- Polos: -899.960 y -4.44020 s⁻¹, equivalentes a 1.111 ms y 225.215 ms.
- Ceros: -1215.445 y -1.61561 s⁻¹.
- Ganancia estática: 0.000581832 A eq / punto porcentual.
- Predice 71.040 mA inmediatamente tras el escalón y 34.910 mA en régimen.
- Ajuste reproducido: 86.5121 %.

El polo lento concuerda con la caída observada mientras el motor acelera. Un ajuste descriptivo de CS = c + a exp(-t/tau), entre 10 ms y 1.5 s después del flanco, da tau=0.2261 s, c=34.854 mA y a=61.082 mA. No incluye el primer punto ni pretende identificar la dinámica eléctrica.

El polo de 1.111 ms es mucho más rápido que los 10 ms entre filas. Ese polo y el paso directo no quedan validados por este ensayo. El modelo predice 71.04 mA en el primer punto frente a 107.13 mA medidos. No debe utilizarse ese polo para fijar por sí solo una banda rápida del controlador.

Ambos modelos se compararon con el mismo ensayo usado para estimarlos. Falta un ensayo independiente para validar generalización. El registro contiene solo una muestra después de retirar el PWM, por lo que no permite evaluar la descarga de corriente ni la desaceleración completa.

## Modelo conjunto y avance propuesto

La explicación física usa corriente y velocidad como estados acoplados, con magnitudes mecánicas referidas al mismo eje:

L di/dt = v_arm - R i - Ke omega_motor

J d(omega_motor)/dt = Kt i - B omega_motor - tau_carga

d(theta_salida)/dt = omega_salida; omega_motor = 100 omega_salida

El puente determina v_arm según PWM y el modo de recirculación. La salida observada requiere además un modelo de medición CS = h(PWM, corriente, modo del puente). No se debe sustituir sin validación la corriente física de esas ecuaciones por la media CS registrada.

En drive/coast, IPROPI no mide la corriente durante coast. Promediar todo el ciclo combina la corriente observable y el tiempo durante el que es observable. Dividir por duty tampoco recupera automáticamente la corriente media del devanado.

Para una cascada posición → velocidad → corriente:

1. Validar la medida CS contra una referencia de corriente y obtener registros más rápidos en los flancos. Conservar internamente el promedio DMA de 1 ms y registrar cada bloque durante ráfagas cortas permitiría evaluar mejor el lazo efectivo; para la dinámica eléctrica submilisegundo se necesita el osciloscopio o captura más rápida. No basta con aumentar la tasa ADC si el registro sigue a 100 Hz.
2. Usar ensayos bloqueados breves y variaciones pequeñas de PWM en torno a puntos de operación para estimar la respuesta eléctrica observable. Probar el mismo PI sobre datos de eje libre y bloqueado, incluyendo límites, antiwindup y retardo. Mantener ensayos independientes para validación.
3. Con el lazo de corriente cerrado y validado, identificar entrada referencia CS y salida velocidad en rad/s con el eje libre y carga representativa. Esto obtiene la planta que verá el controlador exterior e incluye el lazo interior. Si CS sigue sin calibración, denominarla referencia CS, no torque.
4. Cerrar velocidad antes de posición. La relación cinemática velocidad de salida → posición de salida es 1/s; el diseño de posición también debe incluir la dinámica del lazo de velocidad cerrado y su retardo. El encoder actual promedia velocidad en una ventana de 50 ms.

No se recomienda conmutar automáticamente entre las dos TF solo al detectar velocidad cero. El ensayo bloqueado y el libre son condiciones mecánicas diferentes del mismo sistema acoplado. Una eventual programación de ganancias requeriría más puntos de operación y validación de transiciones.

## Próximos registros

- Pequeños escalones alrededor de 60 % (por ejemplo 55/60/65 %, dentro del rango ya comprobado de la planta), con repeticiones y otros niveles centrales, para evaluar linealidad local. El dato bloqueado 0→60 % aporta una razón salida/entrada, no la pendiente local de toda la curva.
- Capturar antes del flanco y después de STOP hasta observar la relajación completa. Los 2 s actuales bajo PWM muestran el régimen, pero no hay cola temporal después de apagar.
- Registrar tiempos propios del ADC y encoder, voltaje de alimentación, PWM aplicado, calidad del sensado y, para la prueba en lazo cerrado, referencia y salida PI.
- Conservar el firmware y los datos originales durante esta evaluación. No se accionó la planta ni se cambiaron ganancias en este análisis.

Fuentes: [MathWorks tfest](https://www.mathworks.com/help/ident/ref/tfest.html), [validación con otro conjunto de datos](https://www.mathworks.com/help/ident/ug/comparison-of-various-model-identification-methods.html), [TI DRV8876 §7.3.3.1](https://www.ti.com/lit/ds/symlink/drv8876.pdf).
