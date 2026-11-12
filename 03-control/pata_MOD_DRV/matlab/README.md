# Identificación de las articulaciones de la pata (MATLAB)

Calcula la **fricción**, la **inercia** y la **gravedad** de cada articulación (coxa, fémur, tibia) a partir de los CSV que exporta el dashboard. Con eso sugiere los comandos del firmware:
- ganancias del lazo de velocidad (`SPI`) y de posición (`PPI`);
- compensaciones (`FRIC`, `GRAV`);
- límites (`ILIMIT`, `PLIMIT`).

Funciona en MATLAB y en GNU Octave, sin toolboxes, tanto con `pata_MOD_DRV` como con `pata_banco1`.

| Archivo | Qué hace |
|---|---|
| `leer_csv_pata.m` | Lee el CSV del dashboard (cabecera `MHEADER`, 100 Hz) |
| `identificar_articulacion.m` | Identifica una articulación a partir de uno o varios CSV |
| `identificar_pata.m` | Ejemplo para las 3 articulaciones; compara el fémur en varias posturas de la tibia |
| `tests/generar_csv_sintetico.py` | Genera CSV de una articulación simulada con parámetros conocidos (prueba) |

## Qué se calibra y dónde

| Parámetro | Depende de | Se mide |
|---|---|---|
| PI de corriente | Motor + driver | Una vez (ya está identificado) |
| Fricción (`FRIC`, Ic, b) | Motor + reductora (varía por unidad) | **Por motor**, montado |
| Inercia (Jeq) | Eslabón y lo que cuelga de él | **Por articulación** (el fémur, en 2–3 posturas de la tibia) |
| Gravedad (`GRAV`, θ₀) | Peso y postura | **Por articulación** (fémur y tibia) |
| `SPI`, `PPI` | Inercia + fricción | **Por articulación** |

Las 6 patas son iguales: identifica una completa; en las demás repite solo el ensayo 1 (fricción estática).

## Modelo

En las unidades del firmware: corriente en A equivalentes del CS, ángulo en rad, velocidad en rad/s.

$$I = J_{eq}\,\alpha + I_c\,\text{sign}(\omega) + b\,\omega + G\cos(\theta + \theta_0)$$

- $I$: corriente con signo, igual a signo(PWM) × corriente medida.
- $\alpha$: aceleración del eje de salida.
- $G$ y $\theta_0$: se usan tal cual en `Mx,GRAV,G,θ0`.
- **Ajuste:** todos los parámetros salen de un solo ajuste por mínimos cuadrados, usando las muestras con $|\omega| > 0.3$ rad/s de cualquier modo. Las muestras con freno no se usan, porque el CS no ve esa corriente.
- **Alineación:** la velocidad del firmware se adelanta 10 ms para alinearla con la corriente, porque es una diferencia en una ventana de 20 ms.

**Ganancias:** la planta corriente → velocidad es $\omega/I = 1/(J_{eq}s + b)$.
- **Lazo de velocidad:** el PI cancela el polo de la planta ($K_i/K_p = b/J_{eq}$) y fija el ancho de banda: $K_p = J_{eq}\,\omega_c$, $K_i = b\,\omega_c$. Por defecto, $f_c$ = 3 Hz.
- **Lazo de posición:** solo P, con $K_p = \omega_c/4$.
- **Fricción estática (`FRIC`):** 70 % del PWM de arranque, descontando la ayuda o el estorbo de la gravedad en ese instante.

## Ensayo automático (recomendado)

El dashboard tiene el panel **"Ensayo automático de identificación"**, que hace los ensayos 1 y 2 sin intervención y descarga un CSV por ensayo con el nombre que espera `identificar_pata.m`:
- `coxa_rampa.csv`, `coxa_vaiven.csv`;
- `tibia_rampa.csv`, `tibia_vaiven.csv`;
- `femur_rampa.csv` y `femur_tibia_<postura>.csv`.

1. **Antes de empezar:**
   - Pon el cero del encoder con el eslabón horizontal y fija la base con la pata en el aire.
   - Elige la articulación y su **rango seguro**: mínimo y máximo sin topes ni cables.
     - **Fémur y tibia:** los pasos 1 y 2 sueltan el motor, así que empieza con el eslabón **colgando quieto** y pon un rango que **incluya la posición colgando**.
   - Para el fémur, elige la postura en la que está **sujeta la tibia**.
2. **Iniciar ensayo.** El dashboard hace, en orden:
   1. **Sentido del encoder:** pulsos de +15, 25, 40, 60 y 80 % (0.6 s). Anota la corriente de cada uno. Si el encoder estaba invertido, lo corrige con `ESIGN` y te pide que vuelvas a poner el cero; si no se mueve, dice si el driver no entrega corriente o si el encoder no cuenta.
   2. **Rampa:** escalones cada 0.4 s (de 2 % hasta 30 %, luego de 4 % hasta 90 %) en ambos sentidos, 2 veces. Detiene el motor en cuanto arranca.
   3. **Vaivén** en modo Posición por todo el rango, con **0.1 rad de margen**: dos idas y vueltas a 0.5/3, 1/8 y 2/15 (vel. máx en rad/s / acel. en rad/s²). Usa una tolerancia de 0.0023 rad para que el lazo nunca suelte el eslabón, y termina sosteniéndolo en el centro.
3. **Seguridad:** el motor se detiene solo si la articulación sale del rango más de 0.08 rad, si la corriente pasa de 0.65 A durante 0.3 s o si salta nFAULT. También con **Abortar**, con **STOP TODO** o con la barra espaciadora.
4. **Al terminar:** mueve los CSV de *Descargas* a `matlab/datos/` y ejecuta `identificar_pata.m`.

Gravedad y Fricción no se tocan durante el ensayo: la identificación usa la corriente medida, así que no la sesgan, y la gravedad ayuda a que el eslabón no se deslice.

## Ensayos manuales (en el dashboard, por articulación)

**Antes de empezar:**
- Pon el eslabón **horizontal** y pulsa **Cero encoder**. El cero tiene que ser **el mismo** en todos los CSV de esa articulación.
- Pon **Gravedad = 0** y **Fricción = 0** para que el ensayo muestre la planta sin compensar.
- La pata debe estar **en el aire**: el pie no puede tocar nada.

Para cada ensayo: **Grabar** → hacer el ensayo → **Detener grabación** → **Exportar CSV**. Cambia el nombre del archivo al del ensayo (por ejemplo `femur_rampa.csv`) y guárdalo en `matlab/datos/`.

### Ensayo 1: fricción estática (rampa de PWM, lazo abierto)

En el fémur y la tibia, pon el eslabón **vertical**, colgando, para que la gravedad influya lo menos posible. Envía cada línea esperando entre ellas a que el eje se detenga (con `x` = 1, 2 o 3):

```
Mx,SEQ,2:400;4:400;6:400;8:400;10:400;12:400;14:400;16:400;18:400;20:400;22:400;24:400;0:500
Mx,SEQ,-2:400;-4:400;-6:400;-8:400;-10:400;-12:400;-14:400;-16:400;-18:400;-20:400;-22:400;-24:400;0:500
```

En cuanto el eje empiece a moverse, pulsa **STOP** del motor si se acerca a un tope.

### Ensayo 2: fricción, inercia y gravedad (vaivén en Posición)

Usa la pestaña **Posición**, con Límite CS de 0.5 A y Techo PWM de 100 %. Alterna el **Objetivo** entre dos ángulos dentro del rango seguro de la articulación, por ejemplo **−0.6** y **+0.6** rad, pulsando **Ir a posición** cada vez y esperando a que llegue. Haz 2 idas y vueltas con cada combinación:

| Vel. máx (rad/s) | Acel. (rad/s²) |
|---|---|
| 1 | 5 |
| 2 | 20 |
| 4 | 40 |

Lo que importa:
- que la articulación se mueva **en ambos sentidos**;
- que pase por **varios ángulos** (sin eso, la gravedad no se identifica);
- que haya **aceleraciones** (sin eso, la inercia no se identifica).

**Fémur:** repite este ensayo con la tibia **estirada**, a **90°** y **recogida**, y guarda cada uno en su archivo (`femur_tibia_estirada.csv`, `femur_tibia_90.csv`, `femur_tibia_recogida.csv`).

### Ensayo 3 (opcional): escalones en lazo abierto

Da el modelo PWM → velocidad (K, τ, u0), útil para comparar con Simulink. Usa amplitudes pequeñas para no llegar a los topes:

```
Mx,SEQ,20:200;-20:200;30:200;-30:200;20:200;-20:200;0:500
```

## Ejecución

```matlab
addpath('ruta/a/pata_MOD_DRV/matlab')
P = identificar_articulacion({'datos/femur_rampa.csv','datos/femur_tibia_estirada.csv'}, 2);
```

o edita y ejecuta `identificar_pata.m`. Se imprime algo así (con los datos sintéticos de `tests/`, cuyo valor verdadero es Jeq = 0.004, Ic = 0.05, b = 0.002, G = 0.12, θ₀ = 0.1):

```
Modelo en corriente  I = Jeq*alfa + Ic*sign(w) + b*w + G*cos(theta+theta0)
  Jeq    = 0.00422 A*s^2/rad
  Ic     = 0.0495 A
  b      = 0.00133 A*s/rad
  G      = 0.1205 A, theta0 = 0.0916 rad (5.2°)
  R^2    = 0.998 con 2263 muestras
Comandos sugeridos:
  M2,SPI,0.07963,0.02509
  M2,PPI,4.712,0
  M2,ILIMIT,0.300
  M2,PLIMIT,10.00,15.4,0.02
  M2,GRAV,0.1205,0.0916
  M2,FRIC,7.2
```

Los comandos se copian en el **cuadro de comando manual** del dashboard (panel "Mensajes del ESP32"), con el motor en STOP. Límite CS y Techo PWM se ponen además en sus campos de la pestaña Posición, porque **Ir a posición** los vuelve a enviar.

**Avisos que puede dar el script:**
- **Jeq ≤ 0:** faltan aceleraciones en el ensayo.
- **Pocos datos en un sentido:** la fricción y la gravedad se confunden.
- **Regresores casi colineales:** el rango de ángulos del ensayo es muy estrecho.
- **R² bajo:** el cero del encoder cambió entre archivos, o la pata tocó algo.

## Prueba sin hardware

```
python3 tests/generar_csv_sintetico.py /tmp/synth
octave-cli --eval "addpath('.'); identificar_articulacion({'/tmp/synth/femur_rampa.csv','/tmp/synth/femur_vaiven.csv'}, 2, 'graficar', false)"
```
