# Ensayo de M1 con eje libre · 2026-11-16

Se conectó un motor N20 al canal M1 (J3A) de la MOD_DRV y se ensayó con el eje de salida de la reductora libre. Los CSV y las configuraciones de cada serie están en `../matlab/datos/`.

- **Encoder.** La posición aumentó con PWM positivo y disminuyó con PWM negativo. Un giro observado de aproximadamente media vuelta dio 3.128 rad (179.2°), consistente con la escala configurada de 2800 cuentas por vuelta de salida.
- **Arranque.** ±35 % fueron los primeros niveles probados con giro sostenido, en tres de tres intentos por sentido. Los pulsos fueron de 550 ms, con pasos de 5 puntos porcentuales. Se conservan los registros completos de ambas polaridades.
- **Posición.** Cada configuración repitió tres veces la secuencia 0 → +0.25 → 0 → −0.25 → 0 rad, con unos 3 s por consigna.

| Configuración | Escalones a ±0.25 rad que finalizaron en ±0.01 rad | Sobrepaso máximo |
|---|---:|---:|
| Sin compensación, 0.8 rad/s | 0/6 | — |
| Ic 0.015 A, Is 0.035 A, 0.8 rad/s, rampa de 200 ms | 2/6 | 0.069 rad |
| Ic 0.015 A, Is 0.035 A, 0.5 rad/s, rampa de 200 ms | 5/6 | 0.064 rad |
| Ic 0.015 A, Is 0.035 A, 0.5 rad/s, rampa de 400 ms | 2/6 | 0.062 rad |
| Ic 0.010 A, Is 0.035 A, 0.5 rad/s, rampa de 200 ms | 5/6 | 0.026 rad |

En el ensayo con menor sobrepaso (`2026-11-16_022108_M1_eje-libre_pos-escalones-v050-ic010.csv`), el techo de PWM fue 45 %, el límite CS fue 0.15 A y la aceleración máxima fue 3 rad/s². Los seis escalones entraron por primera vez en tolerancia en 0.30–0.38 s. El peor error final fue 0.01662 rad. Se confirmó STOP y se restauraron los parámetros originales al finalizar.

## Próximo paso

Reproducir los escalones de M1 en el simulador con ángulos objetivo como acción de la política. Comparar posición contra tiempo y extraer por separado el retardo al primer movimiento, el sobrepaso y el error final. Queda pendiente validar con la pata montada. El ensayo con eje libre caracteriza la respuesta de M1 en banco; el par bajo carga y un rango físico para `motor_strength` requieren ensayos adicionales.
