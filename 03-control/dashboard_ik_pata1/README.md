# Dashboard de cinemática inversa y control en cascada · Pata 1 (Unidad 2)

Herramienta para la sustentación. Valida dos cosas sobre la Pata 1 (motores N20 en la placa MOD_DRV):

1. **El error de posición** del efector final y de cada articulación, siguiendo trayectorias de prueba.
2. **El control en cascada** corriente → velocidad → posición del firmware `pata_MOD_DRV`. Se usa una réplica línea a línea de sus controladores, con las mismas ganancias, frecuencias y saturaciones.

Solo lee el resto del repositorio: la geometría sale de la medición de Fusion, los parámetros de `pata_MOD_DRV` y los datos reales de `pata_MOD_DRV/matlab/datos/`.

## Instalación (una sola vez, ~2 min)

Desde PowerShell, en esta carpeta (`03-control/dashboard_ik_pata1`):

```powershell
python -m pip install --user -r requirements.txt
```

- **Qué hace:** instala numpy, scipy, matplotlib, pyyaml, plotly, dash y pytest.
- **Qué debería pasar:** al final aparece `Successfully installed …`, o `Requirement already satisfied` si ya estaban.
- **Si falla:** comprueba que `python --version` diga 3.10 o superior. En este PC es 3.12.

## Uso

### 1. Generar todas las figuras (~20 s)

```powershell
python export_figures.py
```

- **Qué debería pasar:** imprime la lista de figuras y termina con `Listo: 20 figuras en …\figures`. Cada figura sale en PNG (300 dpi) y en SVG. Además se generan `tabla_metricas.csv`, `tabla_metricas.tex` y `reporte_cinematica.json`.
- Si no quieres la comparación con la pata real, usa `python export_figures.py --sin-real`.
- **Si falla al leer el CSV real:** revisa la ruta `datos_reales.csv` en `config.yaml`.

### 2. Dashboard interactivo (arranca en ~5 s)

```powershell
python app.py
```

- Abre **http://127.0.0.1:8050** en el navegador. Para cerrarlo, pulsa Ctrl+C en la terminal.
- **Cinemática:** deslizadores de la directa, inversa con las 4 ramas, y la pata en 3D (mallas STL opcionales).
- **Trayectorias:** pie deseado vs alcanzado con animación, error cartesiano y angular, y referencia vs respuesta de los tres lazos.
- **Cascada:** escalón de cada lazo, Bode de lazo cerrado y tabla de métricas, por articulación.
- **Datos reales:** sube un CSV del dashboard de la pata. Se simula la misma secuencia de objetivos y se superpone.
- **Si el puerto está ocupado:** cambia `port=8050` al final de `app.py`.

La vista de cinemática, la simulación de trayectorias y el modo simulado web
arrancan en `simulacion.postura_inicial_cad_rad` de `config.yaml`. En esa
postura el fémur está horizontal y la tibia queda casi alineada con él.

### 3. Dashboard en vivo con la pata real: `web/pata_ik_3d.html`

Página web que **se conecta a la pata por USB (Web Serial)**. Hace cuatro cosas:
- dibuja en 3D la pata real con los ángulos de los encoders, la referencia del firmware y el objetivo;
- mueve el pie a una posición x, y, z con la cinemática inversa;
- corre secuencias de puntos (punto a punto, círculo y ciclo de paso);
- graba un CSV con el error del pie en mm.

No usa internet ni librerías externas. La geometría está en `web/geometria_pata1.js`. Si cambias `config.yaml`, regenérala con `python export_web_geometry.py`.

Pasos:

1. **Cierra el dashboard de la pata (`pata_dashboard.html`) y cualquier monitor serie.** Solo un programa puede tener el puerto abierto.
2. **Abre `web/pata_ik_3d.html` en Chrome o Edge,** con doble clic o arrastrándolo al navegador. Arriba debe aparecer en verde *modelo OK*: el modelo del navegador coincide con el de Python.
3. **Practica primero con *Modo simulado*,** sin la pata: es un ESP32 falso que responde a los mismos comandos.
4. **Pulsa *Conectar a la pata*** y elige el puerto (COM6 o COM16). En ~1 s la pata azul se mueve igual que la real.
   - **Si el dibujo no coincide con la pata real** (una articulación gira al revés o está desplazada), corrige el signo o el offset en *Mapeo encoder → CAD*.
   - O pon la pata en la pose cero del CAD (fémur horizontal, tibia como en el modelo) y pulsa *La pata está en la pose cero del CAD*.
5. **Mueve el pie** con los botones ±5 mm o escribiendo x, y, z. La pata naranja muestra la solución antes de enviarla. Pulsa *Enviar a la pata*: manda `Mx,PREF` y, si hace falta, `Mx,MODE,POSITION` a las tres articulaciones.
6. **Corre una secuencia:** elige una en *Secuencias de prueba* y pulsa *Iniciar*. Al terminar se descarga el CSV con el error al llegar a cada punto.
7. **Para todo con STOP o la tecla Esc.**

Para que la placa reconozca la misma postura al encender, calibra los encoders,
coloca la pata en esa postura de reposo y usa **STOP TODO** y luego
**Guardar pose de arranque** en `pata_dashboard.html`. Coloca la pata en la
misma postura física antes de cada encendido porque los encoders son incrementales.

Limitaciones:
- **Por USB solo se pueden mandar objetivos punto a punto.** El comando `PREF` reinicia el perfil de velocidad, así que no sirve para seguir una trayectoria continua. Por eso las secuencias esperan a que las tres articulaciones lleguen (estado HOLD), o 8 s, antes de mandar el punto siguiente. El seguimiento continuo con *feed-forward* existe solo por CAN (SEGUIR, desde la maestra).
- El modo simulado es un modelo cinemático simple, sin fricción ni gravedad, solo para practicar la interfaz. Para el comportamiento dinámico está `app.py`.

El CSV grabado tiene las mismas columnas que el del dashboard de la pata, así que se carga tal cual en la pestaña *Datos reales* de `app.py`. Además trae `target_*_mm`, `foot_*_mm`, `foot_error_mm` y `seq_index`.

### 4. Tests (~1 s)

```powershell
python -m pytest -q tests
```

Debería decir `29 passed`. Comprueban:
- FK(IK(p)) < 1e-6 m en todo el espacio de trabajo y la FK contra el modelo MATLAB ya validado con Fusion;
- los polos del lazo de corriente discreto;
- los márgenes de los tres lazos y la separación de frecuencias;
- la llegada a la banda de tolerancia y el anti-windup.

## Archivos

| Archivo | Contenido |
|---|---|
| `config.yaml` | Todos los parámetros con su fuente (archivo:línea). `TODO` = dato no encontrado |
| `kinematics.py` | FK por giros de Rodrigues (misma convención que `cinematica_pata1_u2.m`), IK analítica con 2 ramas de coxa × 2 de codo, singularidades y espacio de trabajo |
| `motor_model.py` | Planta: modelo eléctrico (identificado o físico) + mecánica identificada con fricción de Karnopp, gravedad y freno; encoder PCNT ×4 con ventana de 20 ms |
| `cascade_control.py` | Réplica de `pi_update_sample`, `speed_pi_update`, `position_pi_update`, `friction_feedforward`, `cascade_current`, `dither_term` y de la cascada de `motor_control_step`; simulador y métricas del escalón |
| `loop_analysis.py` | Modelos lineales del diseño: Bode de lazo cerrado, ancho de banda, cruce y márgenes |
| `trajectories.py` | Punto a punto, línea recta, círculo y ciclo de paso (apoyo + vuelo cicloidal) |
| `experiments.py` | Ensayos estándar (escalones, verificación de la IK) |
| `real_data.py` | Lector del CSV de la pata y comparación con la simulación |
| `mesh_utils.py` | Carga y simplificación de las STL |
| `app.py`, `export_figures.py` | Dashboard y exportación de figuras |
| `web/pata_ik_3d.html` | Dashboard en vivo con la pata real: Web Serial, vista 3D, IK, secuencias y CSV |
| `export_web_geometry.py` → `web/geometria_pata1.js` | Geometría y vectores de prueba para la página web |

## Convenciones

- **Ángulos:** `q_cad` (Fusion) = `signo · q_enc + offset`. La cinemática trabaja en `q_cad`; el control, en `q_enc`, igual que el firmware.
- **Corrientes:** en **A equivalentes del sensado** (corriente real × ciclo de trabajo), como el firmware y la identificación. El par de la planta se expresa en esas mismas unidades: `Jeq·α = T − Ic·sign(ω) − b·ω − gravedad`.
- **Posiciones:** en mm, en el marco raíz de Fusion.

## Modelo de la planta

- **Por defecto, `planta.modelo: identificado`:** la corriente sensada vale `a·(PWM − 20 %)` con τe = 0.2 ms. Es el modelo con el que se diseñaron las ganancias.
- **`fisico`:** usa R y Ke derivados del catálogo Pololu, más L (`TODO`).
- Con los dos modelos el escalón de posición casi no cambia (diferencia de < 0.1 s en ts).

## CSV de datos reales

**No hay que cambiar el firmware.** Basta con el CSV que ya guarda `pata_dashboard.html` (telemetría `TEL,MULTI` a 100 Hz). Columnas:

```
time_ms, selected,
m{k}_mode, m{k}_duty_percent, m{k}_current_a, m{k}_current_ref_a,
m{k}_position_rad, m{k}_speed_rad_s, m{k}_speed_ref_rad_s, m{k}_position_ref_rad,
m{k}_position_state, m{k}_current_status, m{k}_flags        (k = 1 coxa, 2 fémur, 3 tibia)
```

- Para validar el **error cartesiano** en la pata real, las tres articulaciones tienen que moverse en el mismo registro, por ejemplo con una secuencia SEGUIR desde la maestra. Con el CSV de una sola articulación solo se compara ese eje.
- La pestaña *Datos reales* y `fig 30` usan el modo posición (`mode = 3`) para recortar el registro.

## Simplificaciones de la réplica (frente a `main.c`)

- No se replican FMAP, FRIC en PWM (se deja en 0, como recomienda el firmware), nFAULT, fallos de sensado ni el *ride-through*.
- La dinámica de cada articulación está desacoplada. El acople es solo por la gravedad (GRAV2).
- Gravedad de la planta = la calibrada el 20-nov. `factor_gravedad_planta` permite estudiar errores de modelo.
- No hay topes mecánicos en la planta.

## Pendientes (`TODO` en `config.yaml`)

1. Ganancias vigentes: viven en la NVS. Copia la línea `CFG,Mx,...` de cada motor (comando `INFO`). Hoy se usan las del 14-nov.
2. ILIMIT que envía el dashboard. Hoy se usa 0.1 A, el valor por defecto del firmware.
3. Signo de la coxa y offsets entre el cero del encoder y el del CAD.
4. Límites mecánicos reales. Hoy se usan los rangos probados.
5. Inductancia L, solo para el modelo físico.
6. `a` (A/%) de la pata modificada: se usa el del 11-nov.
7. STL de la versión `alfred_mejorado`: las actuales son del 4-nov y solo sirven como referencia visual. La de la tibia no coincide con la geometría nueva.
