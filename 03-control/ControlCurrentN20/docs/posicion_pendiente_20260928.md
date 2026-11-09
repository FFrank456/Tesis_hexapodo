# Revision previa al lazo de posicion

**Nota histórica:** el usuario confirmó después el PI `(6.166*s+3192)/s` y solicitó implementarlo. La implementación actual se documenta en [control_posicion_20260928.md](control_posicion_20260928.md). Los apartados siguientes describen la revisión previa; el diagnóstico con modelos históricos no queda invalidado por haber implementado el software.

Solicitud: agregar posicion a la cascada existente. No se han cambiado firmware,
ganancias ni Processing en esta revision; faltan dos datos solicitados al usuario.

## Discrepancia entre capturas

La expresion `6.166*s + 3192/s = (6.166*s^2+3192)/s` contiene derivada e integral,
sin proporcional. El sintonizador muestra Kp=6.166, Ki=3192, Kd=0: un PI cuya
expresion es `6.166 + 3192/s = (6.166*s+3192)/s`. Se necesita confirmar cual es
el controlador exportado y cual fue la planta usada en PID Tuner.

## Comprobacion independiente

Con los coeficientes historicos de matlab/revision_cascada_20260927.md:

    Ti = feedback(Ci*Gi,1)
    Tvel = feedback(Cw*Ti*Gw,1)
    Tpos = feedback(Cpos*Tvel/s,1)

Para Cpos=6.166+3192/s se obtienen polos con parte real positiva aproximados
+18.5205 +/- j33.7962 por segundo. Para la expresion derivativa de la primera
captura tambien aparecen polos inestables, aproximadamente +1.17519 +/- j21.3010.
Se calcularon con operaciones de polinomios y numpy.roots; no se ejecuto MATLAB,
Simulink ni una prueba del motor. Son resultados de esos modelos historicos,
no prueba de que el archivo actual del usuario use la misma planta.

matlab/revisar_posicion_cascada.m permite reproducir el caso historico o pasar
el Tvel cerrado actual y el controlador elegido. No modifica el workspace ni
el firmware; devuelve polos y estabilidad del modelo lineal continuo.

## Arquitectura a concretar

    theta_ref [rad] -> controlador posicion -> w_ref [rad/s]
        -> PI velocidad -> I_ref [A eq. CS] -> PI corriente -> PWM

Realimentacion de posicion y velocidad desde el ESP32. La planta externa debe
incluir la respuesta del lazo interior cerrado y la relacion cinematica 1/s.

Propuesta inicial: posicion acumulada respecto al cero explicito del encoder,
sin envolver automaticamente a +/-pi; limite de velocidad y aceleracion de
referencia; antiwindup al saturar velocidad, corriente o PWM; entrada al modo
tomando la posicion actual para evitar saltos; tolerancia de llegada basada
en posicion y velocidad, con histeresis. Se debe acordar mantener posicion o
liberar en coast al llegar. No hay referencia absoluta al arrancar sin homing.

El firmware actual solo acepta magnitud de velocidad y sentido separado. La
salida de posicion necesita signo para corregir ambos lados del objetivo:
se requiere un gestor automatico de sentido con coast, espera de velocidad
cercana a cero y reinicio/tracking de los PI al invertir. No basta conectar
el numero negativo a WREF. Sin frenado activo no se promete parada precisa
ni mantenimiento de posicion frente a perturbaciones con la logica actual.

El periodo exterior de 20 ms es una propuesta previa, no una eleccion validada
para esta sintonizacion. La velocidad conserva ventana de medicion de 50 ms.
La respuesta mostrada en la captura (~0.12 s de establecimiento y ~13 % de
sobreimpulso, estimados visualmente) requiere contrastar muestreo, retardos
y limites con el modelo completo antes de trasladarla a la planta.
