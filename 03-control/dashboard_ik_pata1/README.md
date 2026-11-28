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

### 3. Gemelo digital en vivo con la pata real: `web/pata_ik_3d.html`

Página web que **se conecta a la pata por USB (Web Serial)** y la compara en vivo con un **gemelo digital**:
- El gemelo es una réplica en JavaScript de la cascada del firmware más la planta identificada (`web/gemelo.js`, la misma lógica que `cascade_control.py`).
- Recibe los mismos objetivos que la pata. Usa las ganancias de la placa en cuanto llegan las líneas `CFG`; antes de eso, las de `config.yaml`.

Qué muestra:
- **Vista 3D** de la pata con las mallas de Fusion: la real en azul y gris, el gemelo en naranja translúcido y la referencia del firmware en gris discontinuo.
  - El origen O está en el eje de la coxa, con X a lo largo de la pata estirada, Y en los ejes de J1 y J2, y Z sobre el eje de la coxa hacia arriba.
  - La rejilla es el plano del pie en home.
- **Gráficas sincronizadas** de cada articulación, con la pata real, el gemelo y la referencia:
  - ángulo, velocidad, corriente (con ILIMIT) y esfuerzo de control (PWM, con saturación y zona muerta);
  - par estimado τ = Kt·I/d;
  - error del pie.
  - Ventana de 10 a 180 s, pausa y cursor común (pasa el ratón).
- **Indicadores por articulación:** RMSE real–gemelo y real–referencia, corriente rms, PWM medio y saturado, y par máximo.
- **IK, secuencias y CSV**, como antes. Además, *Home* manda las tres articulaciones a 0. Los botones *Guardar imagen 3D* y *Guardar gráficas* descargan PNG.

**Home = pata estirada = q = 0.** El encoder tiene que valer 0 en esa pose al encender. Si en la placa hay guardada una pose de arranque distinta, bórrala una vez desde el dashboard de la pata con el comando `NVS,POSE,CLEAR`. Si no, el dibujo saldrá girado.

Pasos:

1. Cierra `pata_dashboard.html` y cualquier monitor serie: solo un programa puede abrir el puerto a la vez.
2. Enciende la pata **estirada**. Abre `web/pata_ik_3d.html` en Chrome o Edge con doble clic. Arriba debe aparecer *modelo = Python ✓*.
3. Practica con *Modo simulado*. En ese modo, otro gemelo con la planta alterada hace de "pata real" (fricción +10 %, inercia +20 %, gravedad +10 %), así ves cómo se notan las diferencias.
4. Pulsa *Conectar a la pata* y elige el puerto (COM6 o COM16). En ~1 s debería aparecer *gemelo: ganancias de la placa*. Mueve la pata a mano y comprueba que el dibujo azul gira igual. Si no, corrige el signo en *Mapeo encoder → CAD*.
5. Mueve el pie con IK o corre una secuencia con *grabar CSV* marcado. La secuencia termina con el RMSE de llegada y el RMSE real–gemelo.
6. Genera las figuras de la tesis a partir de ese CSV:
   ```powershell
   python figuras_gemelo.py "$env:USERPROFILE\Downloads\pata1_secuencia_gait.csv"
   ```
   En ~30 s salen en `figures/gemelo/`: g01 ángulos, g02 velocidad, g03 corriente, g04 esfuerzo de control, g05 par, g06 pie en 3D sobre la pata de Fusion y g07 error del pie, más `tabla_gemelo.csv` y `tabla_gemelo.tex`.
7. Para todo con STOP o la tecla Esc.

**Limitación:** por USB solo se mandan objetivos punto a punto, porque `PREF` reinicia el perfil de velocidad. El seguimiento continuo con *feed-forward* existe solo por CAN (SEGUIR desde la maestra).

### Geometría desde Fusion (MCP)

`python fusion/actualizar_desde_fusion.py` lee la pata directamente de Fusion (con el documento abierto y el add-in MCP activo en `http://127.0.0.1:53157/mcp`). Solo usa scripts de lectura. Genera:
- **`fusion/geometria_pata1_v2.json`:** los ejes, tomados de los cilindros de los bujes; el pie, como centro de la esfera de `TibiaPunta`; y el marco de la pata.
- **`fusion/mallas_pata1_v2.npz`:** las mallas por eslabón.

Después de actualizar, corre `python export_web_geometry.py` para refrescar la página web. La asignación de piezas a cada eslabón está al principio del script (`ESLABONES`): si agregas piezas en Fusion, añádelas ahí.
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
| `mesh_utils.py` | Carga de las mallas de Fusion (y de las STL antiguas) |
| `app.py`, `export_figures.py` | Dashboard y exportación de figuras |
| `web/pata_ik_3d.html` + `web/visor3d.js`, `web/gemelo.js`, `web/graficas.js` | Gemelo digital en vivo: Web Serial, vista 3D WebGL con las mallas de Fusion, gemelo de la cascada, gráficas, IK, secuencias y CSV |
| `export_web_geometry.py` → `web/geometria_pata1.js`, `web/mallas_pata1.js` | Geometría, parámetros del gemelo, vectores de prueba y mallas para la página |
| `figuras_gemelo.py` | Figuras de la tesis "real vs gemelo" desde el CSV de la página |
| `fusion/actualizar_desde_fusion.py`, `fusion/fusion_mcp.py` | Lectura de la geometría y las mallas desde Fusion por MCP |

## Convenciones

- **Ángulos:** `q_cad` (Fusion) = `signo · q_enc + offset`. La cinemática trabaja en `q_cad`; el control, en `q_enc`, igual que el firmware.
- **Corrientes:** en **A equivalentes del sensado** (corriente real × ciclo de trabajo), como el firmware y la identificación. El par de la planta se expresa en esas mismas unidades: `Jeq·α = T − Ic·sign(ω) − b·ω − gravedad`.
- **Posiciones:** en mm, en el **marco de la pata**: origen en el eje de la coxa (J0), Z = eje de la coxa hacia arriba, X = a lo largo de la pata estirada, Y = Z × X (paralelo a J1 y J2).
- **Home:** q = 0, la pata estirada tal como está el ensamble de Fusion. Ahí J1 está en x = 50 mm, J2 en x = 110 mm (los dos a z = 6.05 mm) y el centro de la esfera del pie en (206.1, 0.1, −20.6) mm.
- **Sentidos positivos** (los de Fusion): +q0 gira la coxa en sentido horario visto desde arriba (hacia −Y), +q1 sube el fémur y +q2 baja el pie.

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
3. Signo de la coxa. Los offsets son 0 porque la pata arranca estirada en la pose cero (requiere `NVS,POSE,CLEAR` en la placa).
4. Límites mecánicos reales. Hoy se usan los rangos probados.
5. Inductancia L, solo para el modelo físico.
6. `a` (A/%) de la pata modificada: se usa el del 11-nov.
7. En Fusion falta el joint coxa–fémur (J1); su eje se toma del buje. El joint de la tibia está 2.5 mm por debajo de sus bujes: se usa el eje físico de los bujes.
