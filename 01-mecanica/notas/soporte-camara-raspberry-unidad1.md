---
tipo: componente-tecnico
estado: diseñado
fecha: 2026-10-17
unidad: Unidad 1 (hexápodo de servos)
archivo_cad: FrameAssembly_URDF v8
tags:
  - hardware
  - mecanica
  - impresion-3d
  - camara
  - raspberry-pi
---

# Soporte de cámara Orbbec Gemini 2 y Raspberry Pi 5 — Unidad 1

> [!summary] Resumen
> Estructura impresa en **PETG** que se monta sobre la placa superior del chasis (`base_link`) **sin modificarlo**. Lleva:
> - Al frente, un **mástil de altura regulable** con una **cuna basculante** para la cámara de profundidad **Orbbec Gemini 2**.
> - Atrás, el **case CanaKit Turbine** de la **Raspberry Pi 5**, encajado a presión.
>
> La torre y la base forman **una sola pieza**. Se fija con los 6 agujeros M3 que ya tiene el marco del chasis. Se diseñó en Fusion 360 y se verificaron las interferencias con todo el robot.

## ⚙️ Especificaciones

### Sistema de referencia
- **Coordenadas locales de `base_link`, en mm.** El frente del robot es **−Y** (patas `r3`/`l3`), +Z hacia arriba y X lateral.
- **Placa superior del chasis (`Cuerpo1 (4)`):** 109 × 159 × 6 mm (z 14–20). El marco está en z = 16 y la placa central en z = 17.
- **Restricción de altura:** la parte superior de los servos de las coxas llega a z 22,2–27,7. Por eso la base del soporte va en **z 22–28** y su contorno se desplazó 3 mm hacia adentro.

### Piezas a imprimir

| Componente (Fusion) | Función | Volumen | Masa aprox. (PETG 1,27 g/cm³) |
|---|---|---|---|
| `soporte_torre_camara_rpi5` | Base + mástil + encaje del case de la Pi (**una sola pieza**) | 89,0 cm³ | 113 g |
| `soporte_camara_carro` | Carro que se desliza por el mástil; pivote e inclinación | 10,5 cm³ | 13 g |
| `soporte_camara_cuna` | Cuna con la forma de la Gemini 2 | 18,3 cm³ | 23 g |
| `soporte_camara_contraplaca` | Platina trasera con tuercas M3 encastradas | 1,55 cm³ | 2 g |
| **Total impreso** | | **≈ 119 cm³** | **≈ 151 g** |

Cuerpos de referencia que **no se imprimen**:
- `ref_gemini2`: carcasa y vidrio reales de la cámara, importados del STEP oficial.
- `ref_case_canakit_pi5`: bloque del case de la Pi.

![[soporte-torre-vista-superior.png]]
*Vista superior: contorno estilo Cuerpo1 (4), mástil al frente (arriba en la imagen), encaje del case, ventanas de ventilación y pasos de cables.*

---

### 1. Base (`soporte_torre_camara_rpi5`)

**Fijación al chasis**
- **6 agujeros M3 pasantes** que ya tenía el marco, en x = ±50 e y = −51,8 / 14 / 35,6.
- Patas cilíndricas de Ø10 apoyadas en el marco (z 16–22).
- Agujero de Ø3,4 con avellanado de Ø6,5 × 3 mm. Tornillo M3 con tuerca por debajo.

**Contorno**
- Es el perfil exterior real de `Cuerpo1 (4)`, leído del modelo, **desplazado 3 mm hacia adentro**.
- Conserva las muescas laterales (±44,5 / ±51,5), los chaflanes a 45° y la **nariz delantera** (hasta y = −78,3, x ±26,8).
- Refuerzos de Ø10 alrededor de cada agujero M3.

**Espesor y unión**
- Placa de **6 mm** (z 22–28).
- La zona entre el mástil y la Pi es **maciza**: la torre queda unida por detrás, sin huecos.

**Vértices del contorno (mm)**

| | |
|---|---|
| Lado −X | (−51,1,−55,3) (−51,5,−55,0) (−51,5,−30,7) (−44,5,−30,7) (−44,5,12,5) (−51,5,12,5) (−51,5,37,2) (−44,5,37,2) (−44,5,51,3) (−27,5,68,3) (−33,2,74,0) (−32,7,74,5) |
| Atrás | (32,7,74,5) … simétrico en +X |
| Frente | (26,8,−69,0) (26,8,−78,3) (−26,8,−78,3) (−26,8,−69,0) (−45,8,−50,0) |

### 2. Mástil de la cámara (integrado en la base)
- **Columna:** 20 × 12 mm (x ±10, y −58 a −46), desde la base hasta **z = 150**.
- **Ranura vertical** de Ø3,4 entre z 40 y 145, para los 2 tornillos del carro.
- **2 nervios triangulares** traseros (x ±6 a 10, y −46 a −34, 40 mm de alto). Dejan un hueco de 12 mm para la contraplaca.

### 3. Carro (`soporte_camara_carro`)
- **Placa:** 32 × 6 × 30 mm, con guías laterales que abrazan la columna (sin rotación).
- **Fijación:** 2 agujeros M3 a ±8 mm del centro, que atraviesan la ranura.
- **Orejas** con **pivote M3** (y = −78, z = H) y **ranura en arco** de radio 11 mm para fijar la inclinación entre **0° y −25°**.
- **Altura del carro H regulable entre 50 y 135 mm.** El valor modelado es H = 95.

### 4. Contraplaca (`soporte_camara_contraplaca`)
- **Platina:** 11 × 5 × 30 mm. Cabe entre los nervios con 0,5 mm por lado, aunque el carro esté en su posición más baja.
- **2 alojamientos hexagonales** para tuerca M3 (5,7 mm entre caras × 3,1 mm de profundidad).
- **Lengüeta de centrado** de 3,0 × 2 × 20 mm que entra en la ranura de 3,4 mm.

![[soporte-contraplaca-trasera.png]]
*Vista trasera del mástil: contraplaca con las tuercas encastradas, carro y nervios.*

### 5. Cuna de la Gemini 2 (`soporte_camara_cuna`)

**Geometría de la cámara, medida en el STEP oficial** (`[3D][STP]Gemini 2 Gemini 335 Asm with FOV_20240523`)
- **Carcasa:** 89,5 × 25 × 30 mm (ancho × alto × fondo), 98 g.
- **Cuerpo principal en forma de píldora con conicidad:**
  - Extremos semicirculares centrados a ±32,2 mm.
  - El radio pasa de **12,55 mm atrás a 10,55 mm al frente**.
  - El fondo plano sube de y −12,55 a −10,55 (unos 5,4°).
- **Bloque trasero** de x ±34 y 8,9 mm de fondo.
- **USB-C** en el costado −X del bloque trasero.
- **Rosca 1/4-20 UNC** en el centro de la cara inferior, cerca de la parte trasera.

**Diseño de la cuna**
- Se construyó con esa **geometría analítica**: conos en los extremos y fondo inclinado.
  - La resta directa de la malla del STEP copiaba pernos internos que impedían insertar la cámara, y dejaba un fondo escalonado difícil de imprimir.
- **Holgura:** 0,3 mm en los extremos curvos. El fondo apoya en toda la superficie.
- **Espesor del fondo:** 3,5 mm atrás y 5,5 mm adelante. Bloque exterior de 93,4 mm de ancho.
- Las paredes de los extremos suben **4 mm por encima del eje** de la cámara, lo que da retención al encajarla desde arriba.
- **Topes traseros** macizos detrás de los extremos.
- **Abertura para el USB-C** en el lado −X.
- **Agujero de Ø6,6** para el tornillo 1/4-20.
- **2 pestañas** (x ±11,8 a 15,8) con agujero de pivote y de bloqueo.

**Posición en el robot (H = 95)**
- Conversión de coordenadas: robot X = x, Y = −z − 84, Z = y + H + 32.
- **Frente de la cámara:** y = −95,35, unos 14 mm por delante de la placa del chasis.
- **Eje del lente:** z = H + 32.

![[soporte-cuna-gemini2.png]]
*Cuna conforme a la carcasa de la Gemini 2: laterales curvos con conicidad, fondo inclinado, agujero 1/4-20 y pestañas al pivote del carro.*

### 6. Encaje del case de la Raspberry Pi 5 (CanaKit Turbine)

**Medidas y posición**
- **Case medido por el usuario:** **65 × 95 × 32 mm** (ancho × largo × alto). Pesa 83 g según Amazon.
- **Orientación:** eje largo en Y, con los puertos USB/Ethernet hacia atrás (+Y).
- **Posición:** y −24 a 71, x ±32,5. Su frente queda a **10 mm de los nervios** del mástil.

**Encaje a presión**
- Sin tornillos.
- **Holgura:** 0,3 mm por lado, más **nervios de presión** (crush ribs) de 0,6 mm en cada pared.
- **4 esquinas en L** de 2,4 mm de espesor y 12 mm de alto. Son más cortas donde están los puertos:
  - Esquinas traseras (USB/Ethernet): 4 mm de pata y 5 mm de alto.
  - Lado largo delantero (USB-C / micro-HDMI): pata de 8 mm.

**Ventilación**
- El case va elevado **3 mm** sobre apoyos.
- **2 ventanas** de 48 mm de ancho bajo el case: el ventilador del case expulsa el aire por su base.

### 7. Pasos de cables hacia el interior del chasis
Cada paso está alineado con una abertura existente de la placa del chasis. Las encontré midiendo el chasis con rayos.

| Paso | Posición (mm) | Medida | Abertura del chasis debajo | Uso |
|---|---|---|---|---|
| Ranuras laterales delanteras (×2) | x ±35,6 a 45, y −50 a −32 | 9,4 × 18 | Canales laterales (x ±32 a 42) | USB-C de alimentación, micro-HDMI |
| Ranuras laterales traseras (×2) | x ±35,6 a 45, y 17 a 36 | 9,4 × 19 | Canales laterales | Otros cables |
| Muesca trasera abierta | x ±20, desde y = 59 hasta el borde | 40 mm de ancho | Ventana trasera (x ±24, y 62–73) | USB / Ethernet |

> [!note] El ancho de 9,4 mm deja pasar un conector USB-C (unos 8,3 mm). Un micro-HDMI con carcasa gruesa puede no pasar: en ese caso, pasa primero el cable.

---

### Tornillería

| Uso | Elemento | Cant. |
|---|---|---|
| Base → chasis | Tornillo M3 (largo según el espesor del marco) + tuerca + arandela | 6 |
| Carro → columna | M3 × 25 + tuercas en la contraplaca | 2 |
| Pivote de la cuna | M3 × 40 + tuerca nyloc | 1 |
| Bloqueo de la inclinación | M3 × 40 + tuerca + arandela | 1 |
| Cámara → cuna | 1/4-20 UNC × 8 mm | 1 |
| Raspberry Pi | — (encaje a presión) | — |

### Secuencia de montaje
1. Atornilla la **base** al chasis con los 6 M3.
2. Encaja el **case de la Pi** a presión y pasa los cables por las ranuras.
3. Fija la **cámara a la cuna** con el 1/4-20. **Hazlo antes del paso 4**, porque después el tornillo del pivote tapa el acceso al tornillo de la cámara.
4. Une la **cuna al carro**: primero el pivote M3 × 40 con la tuerca nyloc, apretada solo hasta que gire con algo de resistencia. Después pon el bloqueo en el arco.
5. Mete las 2 tuercas M3 en la **contraplaca** y colócala detrás de la columna, con la lengüeta en la ranura.
6. Pasa los 2 M3 × 25 desde el frente, por el carro y la ranura. Ajusta la altura H y aprieta.

### Impresión (PETG)
- **Base y torre:** acostada con la placa en la cama y el mástil vertical, sin soportes.
- **Contraplaca:** con la cara de los hexágonos hacia arriba.
- **Cuna:** con el fondo en la cama. Las pestañas quedan hacia abajo, así que conviene imprimirla de costado o con soportes bajo las pestañas.
- Si las tuercas quedan justas, empújalas con la punta del cautín.

## 🔧 Estado
**Diseñado y verificado en CAD.** Todavía no se ha impreso.

| Verificación | Resultado |
|---|---|
| Interferencias de las piezas nuevas con el robot (pose actual) | ✅ Ninguna (las 228 interferencias que existen son del ensamblaje original de las patas) |
| Contraplaca en H = 50 y H = 95 | ✅ Sin choque con los nervios |
| Cuna y cámara inclinadas −10° y −25° alrededor del pivote | ✅ Sin choque con el carro, la torre ni la contraplaca |
| Patas dentro del campo de visión de profundidad (91° × 66°) | ✅ Ninguna, para H 50–135 y tilt 0° a −25° (pose actual) |

### Campo de visión (pose actual de las patas)

| H del carro | Lente sobre el suelo | Suelo visible desde (tilt −20°) |
|---|---|---|
| 50 | ≈ 166 mm | ≈ 126 mm |
| 95 | ≈ 211 mm | ≈ 160 mm |
| 135 | ≈ 251 mm | ≈ 190 mm |

> [!tip] Para empezar se recomienda **H ≈ 50–70 con −20°**: el centro de masa queda más bajo y el suelo se ve desde unos 13 cm. La distancia mínima de profundidad de la Gemini 2 es de 0,15 m.

### Historial de iteraciones (resumen)
1. **Primera versión con piezas separadas:**
   - Mástil con puente sobre 2 agujeros M3.
   - Carro y cuna rectangular de 96 × 28 mm, con la cámara modelada como un bloque.
   - Bandeja de la Pi con postes M2,5.
2. **Contraplaca con tuercas encastradas**, para apretar el carro solo desde el frente.
3. **Soporte de la Pi a presión** para el case CanaKit, con medidas estimadas al principio: 98 × 69 × 32 y luego 94 × 66 × 30.
4. **Unión de la torre y la base de la Pi en una sola pieza**, con placa puente y largueros.
5. **Contorno estilo `Cuerpo1 (4)`, con la base maciza detrás del mástil** y la Pi a 10 mm de los nervios.
6. **Contorno real de `Cuerpo1 (4)` desplazado 3 mm**, con la nariz delantera, para evitar los servos.
7. **Medidas reales del case (65 × 95 × 32)** y **pasos de cables** alineados con las aberturas del chasis.
8. **Cuna conforme a la carcasa real de la Gemini 2**, a partir del STEP oficial.

## 📊 Resultados de prueba
_Pendiente: imprimir y medir el ajuste real del case y de la cámara, y la rigidez del mástil con el robot caminando._

## ⚠️ Pendientes
- [ ] Confirmar de qué lado salen el USB-C y los micro-HDMI de la Pi, para eliminar las ranuras del lado que no se use.
- [ ] Repetir la verificación del campo de visión con la pata delantera en lo más alto de su paso.
- [ ] Unir las piezas a `base_link` antes de exportar el URDF. El documento está en diseño directo, que no admite juntas as-built.
- [ ] Guardar `FrameAssembly_URDF v8` en Fusion.
- [ ] Batería y potencia, fuera de alcance de esta nota. El XL6009 (1,5 A) no sirve; se necesita un buck de 6 V ≥ 15 A y otro de 5 V / 5 A. Ver [[arquitectura-electronica-hexapodos]].

## 🔗 Relacionado
- [[analisis-esfuerzos-diseno-hexapodo]]
- [[RECURSOS MECÁNICOS]]
- Archivo de impresión exportado: `Soporte camara orbec y raspbery.3mf` (en esta carpeta)
- Datasheet de la Gemini 2: <https://www.orbbec.com/wp-content/uploads/2023/04/ORBBEC_Datasheet_Gemini-2-1.pdf>
- Case CanaKit Turbine: <https://www.canakit.com/canakit-raspberry-pi-5-turbine-case-black.html>
