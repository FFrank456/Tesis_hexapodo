---
tipo: componente-tecnico
estado: diseñado
tags:
  - hardware
---

# Diseño y análisis de esfuerzos — justificación de la forma del hexápodo

## ⚙️ Especificaciones

### 1. La idea central: diseño y análisis no son dos pasos, son un ciclo

Un error común es pensar que primero se "diseña la forma" (en CAD, o por inspiración biológica) y después se "analiza" para ver si aguanta. En la práctica de ingeniería es un ciclo iterativo:

```
Definir cargas → Modelo simplificado (a mano) → Geometría inicial (CAD)
     → Análisis de esfuerzos (FEA) → ¿Cumple factor de seguridad y rigidez?
     → No: cambiar geometría/material → repetir análisis
     → Sí: esa geometría final ES tu justificación de diseño
```

Esto encaja directamente con la metodología que ya usas ([[metodologia-vdi]] — VDI 2206/Pahl & Beitz): la fase de "diseño de detalle" de VDI 2206 incluye explícitamente un paso de verificación/cálculo antes de aceptar una solución. **El argumento de "por qué esta forma" para tu tesis no es una explicación a posteriori — es literalmente el historial de iteraciones de este ciclo.**

### 2. Paso 1 — Definir las cargas (esto es lo que más se suele hacer mal)

Antes de calcular cualquier esfuerzo hay que tener claro **qué fuerzas actúan, cuándo, y en qué dirección**. Para un hexápodo esto no es solo "el peso del robot":

- **Carga estática de soporte, por pata.** Si usas marcha trípode (3 patas en apoyo, 3 en balanceo — la más común en hexápodos bioinspirados, ver [[06-literatura-robotica-bioinspirada-insectos]]), cada pata de apoyo soporta hasta **peso_total / 3**, no peso_total / 6. Este es el error de dimensionamiento más común en proyectos estudiantiles de hexápodos.
- **Carga dinámica de impacto al aterrizar el pie.** Cuando el pie toca el suelo al final de la fase de balanceo, la fuerza pico es mayor que el peso estático — se suele modelar con un **factor de impacto** (multiplicar la carga estática por 1.5-3x como primera aproximación conservadora, hasta que tengas datos reales de tus pruebas en [[resultados-pruebas]]).
- **Fuerzas horizontales de reacción**, no solo verticales: al empujar el cuerpo hacia adelante, el suelo empuja la pata también horizontalmente (fricción). Esto genera flexión en direcciones que un análisis "solo vertical" no captura.
- **Momento torsor en las articulaciones** por la reacción del torque del motor/servo contra el eslabón.
- **Carga cíclica (fatiga).** Cada ciclo de marcha es una repetición de carga-descarga sobre el mismo material. En piezas impresas en 3D (PLA/PETG/nylon) esto importa mucho porque la adhesión entre capas suele ser el punto más débil ante fatiga, no el material "en bulto".

**Para tu tesis:** una tabla de casos de carga (caso, magnitud, dirección, ¿estático o dinámico?, ¿dónde actúa?) es el primer artefacto que un jurado espera ver antes de cualquier resultado de esfuerzos — sin eso, cualquier número de FEA parece sacado de la nada.

### 3. Paso 2 — Modelo analítico simplificado (a mano, antes de abrir cualquier software)

Antes de simular, conviene tener un cálculo de "orden de magnitud" con teoría de vigas clásica. Ejemplo ilustrativo (con valores de ejemplo — reemplaza por los reales de tu diseño):

Un eslabón de pata se puede aproximar como una **viga en voladizo (cantilever)**: fija en la articulación proximal, con la carga aplicada en el extremo (el pie).

- Esfuerzo normal por flexión: **σ = M·c / I**
  - M = momento flector en la sección crítica (fuerza × distancia al punto de análisis)
  - c = distancia del eje neutro a la fibra más alejada (mitad del alto de la sección)
  - I = segundo momento de área de la sección transversal
- Factor de seguridad: **n = σ_fluencia / σ_trabajo** (para material dúctil) — en diseño mecánico estudiantil se suele buscar n ≥ 2-3 para piezas estructurales, más si hay incertidumbre en las cargas (que en un robot bioinspirado casi siempre la hay).
- Deflexión en el extremo de una viga en voladizo con carga puntual: **δ = F·L³ / (3·E·I)** — importante porque una pata puede "no romperse" pero deformarse tanto que arruina la cinemática planeada en [[]] (URDF/Gazebo).
- Pandeo (buckling): si algún eslabón trabaja a compresión pura (por ejemplo, una pata casi vertical soportando peso en fase de apoyo), no basta con verificar esfuerzo — hay que verificar la **carga crítica de Euler**: **P_cr = π²·E·I / (L_eff)²**. Un eslabón puede estar muy por debajo del esfuerzo de fluencia y aun así pandear si es esbelto.

### 4. Paso 3 — Por qué la sección transversal define la forma (el argumento clave para "por qué esta forma")

Esta es la pieza que conecta directamente con tu pregunta de investigación. La resistencia y rigidez a flexión dependen del segundo momento de área **I**, no solo de la cantidad de material (masa). Para la misma área de sección transversal (mismo peso de material):

- Una sección **hueca (tubular)** tiene mucho más I que una sección **maciza** equivalente en peso, porque I crece con la distancia del material al eje neutro (∝ r⁴ en un tubo vs área simple en una sección maciza) — alejar el material del centro aporta más rigidez/resistencia por gramo.
- Esto es exactamente por qué el exoesqueleto de los insectos (tu referencia bioinspirada, [[09-bill-ant-p-acromyrmex-versicolor]]) es un tubo hueco y no una varilla sólida: máxima rigidez a flexión con mínima masa — el mismo principio de ingeniería que vas a usar para justificar tus eslabones.
- **Esto es literalmente tu argumento de tesis:** "la forma de la pata no se eligió solo por inspiración biológica, sino porque optimiza rigidez/resistencia por unidad de masa — que es la misma razón evolutiva detrás de la forma del exoesqueleto del insecto de referencia." Esa frase, respaldada con tus números de FEA, es mucho más fuerte que solo "se parece a una hormiga".
- Otros elementos de forma que se justifican por el mismo principio, no por estética:
  - **Nervaduras (ribs)** en vez de paredes gruesas sólidas: añaden I en la dirección de flexión dominante sin agregar masa en toda la sección.
  - **Filetes (fillets) en las esquinas/uniones**, nunca ángulos vivos: reducen el **factor de concentración de esfuerzos (Kt)** — un ángulo vivo puede multiplicar el esfuerzo local varias veces respecto al valor "nominal" calculado con la fórmula simple de viga, y es la causa más común de rotura en piezas impresas en 3D en la articulación, no en el cuerpo del eslabón.

### 5. Paso 4 — Validar con FEA (Análisis de Elementos Finitos)

El cálculo a mano de la sección 3 asume geometría simple (prismática, sin agujeros). En cuanto tu pieza tiene agujeros de tornillo, cambios de sección, uniones en T, etc., necesitas FEA para capturar concentraciones de esfuerzo que la fórmula de viga no ve. Herramientas típicas para un proyecto como el tuyo: SolidWorks Simulation / Fusion 360 (si tienes el CAD ahí, ver [[]]), o FreeCAD FEM (gratuito) para validar.

Qué reportar de cada corrida de FEA (esto es lo que va en [[resultados-pruebas]] o en el capítulo de Resultados):
1. **Condiciones de frontera y cargas aplicadas** (deben coincidir con la tabla de casos de carga del paso 1 — si no coinciden, el resultado no es defendible).
2. **Mapa de esfuerzo de Von Mises** (criterio estándar para materiales dúctiles como la mayoría de plásticos de impresión 3D y aluminios) — identifica dónde está el esfuerzo máximo y si coincide con la intuición (uniones, cambios de sección).
3. **Factor de seguridad resultante** en el punto crítico.
4. **Deflexión máxima** y si es compatible con la precisión que necesita tu cinemática/control (un eslabón que se deforma más de lo esperado introduce error de posición que tu lazo de control en cascada tiene que compensar innecesariamente).

### 6. Paso 5 — La tabla de iteraciones ES la justificación de la forma

El artefacto más convincente para un jurado no es "aquí está el resultado final", es la evolución:

| Iteración | Cambio respecto a la anterior | Masa | Esfuerzo máx. (Von Mises) | Factor de seguridad | Deflexión máx. |
|---|---|---|---|---|---|
| v1 | Eslabón sólido, sección rectangular | — | — | — | — |
| v2 | Se vacía el eslabón (sección hueca) | ↓ | ↑ (esperado, menos material) | ¿cumple? | ↑ |
| v3 | Se agregan nervaduras internas | ↑ leve | ↓ | ✅ cumple | ↓ |
| v4 | Se agregan filetes en la unión con la articulación | = | ↓ (se elimina la concentración de esfuerzo) | ✅ mejora | = |

Esta tabla, con capturas de los mapas de esfuerzo de cada versión, es exactamente lo que responde "por qué esta forma y no otra": cada cambio de geometría tiene una razón medible, no estética. Usa `templates/componente-tecnico.md` para documentar cada iteración como una nota o como versiones dentro de esta misma nota.

## 🔧 Estado
Metodología definida. Pendiente aplicarla con las cargas reales del hexápodo (masa total, distribución por pata, torque de los motores N20) y las iteraciones de CAD/FEA correspondientes.

## 📊 Resultados de prueba
_(pendiente — completar con la tabla de iteraciones de la sección 6 cuando existan las corridas de FEA)_

## 🔗 Relacionado
- [[06-literatura-robotica-bioinspirada-insectos]] — por qué 6 patas / marcha trípode
- [[09-bill-ant-p-acromyrmex-versicolor]] — referencia de exoesqueleto hueco bioinspirado
- [[15-robots-insectos-espacios-estrechos]]
- [[marco-teorico]]
- [[TESIS]]
