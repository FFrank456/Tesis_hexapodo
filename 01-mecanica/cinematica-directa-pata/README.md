# Cinemática de la Unidad 2

El primer objetivo es obtener la posición del punto de referencia del pie en `base_link` a partir de los tres ángulos de la pata 1. La derivación será geométrica y el URDF servirá para comprobar la implementación en MATLAB.

El modelo de referencia confirmado es **`ROBOT_ESAMBLE_alfred_mejorado`** en Autodesk Fusion.

## Geometría disponible

- **Inspección en Fusion.** La [medición del ensamble](medicion_fusion_u2.json) identifica J0 (`join_coxa_1_J0`, base–coxa), J1 (`Revolución 38`, coxa–fémur) y J2 (`join_femur_1_J1`, fémur–tibia) por los componentes que conecta cada unión. Las tres registraron 0 rad en la pose de referencia y los ejes J1 y J2 son paralelos.
- **Giro positivo de J2.** Las vistas laterales de Fusion a 0° y +10° muestran que el pie baja respecto al fémur al aumentar el ángulo de J2. La observación confirma el sentido del giro en torno al eje medido y es una comprobación cualitativa.
- **Giro positivo de J1.** Las vistas laterales a 0° y +10° muestran que el fémur eleva J2 y el pie al aumentar el ángulo de J1 con J2 en cero. Para reproducir el ángulo de mando de Fusion mediante la regla de la mano derecha, usar el eje J1 con signo opuesto al vector leído de la API y conservar el signo leído para J2.
- **Giro positivo de J0.** Las vistas superiores a 0° y +10° muestran un giro horario alrededor de J0 visto desde arriba, coherente con el eje medido hacia −Z. Las capturas recientes muestran las etiquetas `join_coxa_1_J0`, `join_femur_1_j1` y `join_tibia_1_J2`. La medición conserva los nombres anteriores. El código identifica las uniones por sus componentes.
- [ROBOT_ESAMBLE.xacro](../urdf/ROBOT_ESAMBLE_description-20261104T154508Z-1-001/ROBOT_ESAMBLE_description/urdf/ROBOT_ESAMBLE.xacro) es la exportación histórica del 4 de noviembre. Declara 18 articulaciones y permite reconstruir los orígenes y ejes de las seis patas de esa versión. Las articulaciones se declaran continuas, así que sus límites de trabajo deben venir del diseño y del prototipo.
- La [nota de rediseño de la pata 1](../notas/rediseno-juntas-n20-unidad2.md) describe el archivo `ROBOT_ESAMBLE_alfred Copy` y sirve como antecedente de diseño.
- [Vistas de la exportación](unidad-2-geometria-xacro.png) muestran los montajes en planta y la pata 1 en perfil, con orígenes del Xacro y siluetas de sus STL en `q = 0`. En esa exportación, las distancias entre los orígenes consecutivos de la pata 1 son aproximadamente 50 y 60 mm. El marco `base_link` de ese archivo se conserva en el gráfico.

## Ruta de trabajo

1. **Fijar la geometría de la pata 1.** Identificar la revisión de Fusion, fijar la pose cero y marcar un punto nominal del pie. Registrar los orígenes, las direcciones de los ejes y el vector entre J2 y el punto de referencia del pie en esa pose. Un pie curvo o deformable puede cambiar de contacto real con la orientación y la carga.
2. **Derivar la cinemática directa.** Dibujar planta y perfil. Si se confirma la geometría de los ejes, calcular el alcance y la altura con los giros de fémur y tibia en el plano de flexión y aplicar el giro de coxa. Incluir los desplazamientos y los ángulos fijos que aparezcan en el CAD. Documentar las unidades y la relación entre el cero del encoder y el cero del modelo.
3. **Verificar la pata 1.** Comparar en MATLAB la pose cero, giros aislados y poses combinadas con el URDF de la misma revisión. Reportar las diferencias cartesianas y su norma en milímetros. Usar muestras medidas en CAD o en el banco para comprobar la geometría de forma independiente.
4. **Replicar en las seis patas.** Usar una sola función con parámetros por pata. Registrar el montaje respecto a `base_link`, la geometría local y la convención de ángulos. Compartir parámetros geométricos si el CAD confirma que corresponden a la misma geometría. Verificar el sentido de giro también en el lado opuesto.
5. **Extender a inversa y marcha.** Resolver los ángulos para una posición objetivo, comprobar la solución con la directa y delimitar el espacio alcanzable considerando topes y colisiones. Después simular trayectorias de apoyo y vuelo de los seis pies.

## Datos para empezar

La pose de referencia es `q = [0, 0, 0]` en `ROBOT_ESAMBLE_alfred_mejorado`. El punto nominal del pie es el punto más bajo de la malla de `soporte_tibia_A` en esa pose, aproximadamente (244.83, −109.68, −39.55) mm respecto al origen del ensamble. Los ceros, signos y límites de los encoders harán falta para contrastar con el banco y definir el espacio de trabajo real.

## MATLAB

La primera comparación cuantitativa con Fusion usa el vértice donde empieza el zapato. En la pose cero se midió `[222.264, -130.525, -21.475]` mm en el marco raíz. Para `[J0, J1, J2] = [0, 20, 15]°`, Fusion muestra `[221.234, -129.054, 7.678]` mm y el modelo predice `[220.999, -129.260, 7.519]` mm. La diferencia Fusion menos modelo es `[0.235, 0.206, 0.159]` mm y su norma es **0.351 mm** en esta pose.

MATLAB R2025b está instalado en el equipo. Desde PowerShell se pueden ejecutar los scripts de esta carpeta en una sesión separada con `matlab -batch`. Si se trabaja desde la ventana de comandos de MATLAB, se puede abrir esta carpeta y ejecutar el mismo archivo por su nombre.

La función `cinematica_pata1_u2` usa las medidas del ensamble actualizado. Recibe los ángulos de Fusion en radianes y devuelve el pie en milímetros en el marco raíz de Fusion. Por ejemplo, `cinematica_pata1_u2(deg2rad([10 0 0]))` evalúa la pose de la segunda captura de J0. La función conserva los desplazamientos laterales entre los ejes. Cada giro deja fija la componente paralela a su eje y gira la perpendicular en su plano.

Para abrir la simulación interactiva, ejecuta `simular_pata1_u2` en MATLAB con esta carpeta en el path. Los controles mueven las tres articulaciones en grados y la línea discontinua muestra la pose cero. **Pose combinada** aplica [10, 20, 15]° y **Volver a cero** restablece la referencia. El recorrido de los controles de ±30° es para explorar la cinemática. Los límites mecánicos y las colisiones de la pata completa requieren una verificación aparte.

```powershell
matlab -wait -batch "addpath('01-mecanica/cinematica-directa-pata'); diagnostico_u2"
```

El comando se ejecuta desde la raíz de `TESIS_2`. El diagnóstico requiere Robotics System Toolbox y usa `importrobot` y `getTransform` para ubicar J0, J1 y J2 en la exportación histórica. Se ejecutó correctamente desde la sesión de escritorio de MATLAB R2025b y devolvió los orígenes de las seis patas en la pose cero. La ejecución desde el entorno restringido de la terminal sigue dependiendo del acceso a la carpeta de preferencias. La futura función de cinemática geométrica se puede implementar con MATLAB base.
