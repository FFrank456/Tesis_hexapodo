---
tipo: componente-tecnico
area: sensado
fecha: 2026-11-09
estado: por decidir alcance
tags:
  - gases
  - sensores
  - normativa
  - ds-024-2016-em
---

# Gases a monitorear y umbrales

> [!warning] Alcance actual
> El **MQ-9** de la Unidad 1 solo cubre **CO y gases combustibles (CH₄, GLP)**. Para minería faltan **O₂, H₂S, NO₂ y SO₂**, y para laboratorios, **CO₂**. Hay dos caminos: añadir sensores electroquímicos o justificar en la tesis un alcance limitado a CO y CH₄.

## Tabla de gases

| Gas | Dónde aparece | Riesgo | ¿Lo mide el MQ-9? | Sensor sugerido | Límite DS 024-2016-EM, Anexo 15* |
|---|---|---|---|---|---|
| CO | Voladuras, motores diésel, incendios | Tóxico, asfixia química | Sí | MQ-9 o electroquímico | 25 ppm |
| CH₄ (grisú) | Minas de carbón, socavones | Explosivo | Sí | MQ-9 / MQ-4 | 5000 ppm (0,5 %) |
| O₂ (déficit) | Socavones mal ventilados | Asfixia | No | Electroquímico de O₂ | Mínimo 19,5 % (art. 246) |
| H₂S | Minas polimetálicas, aguas estancadas | Muy tóxico | No | Electroquímico de H₂S | 10 ppm (TWA) / 15 ppm (STEL) |
| NO₂ | Voladuras, gases diésel | Tóxico, irritante | No | Electroquímico de NO₂ | 3 ppm / 5 ppm |
| SO₂ | Minerales sulfurados, voladuras | Tóxico, irritante | No | Electroquímico de SO₂ | 2 ppm / 5 ppm |
| CO₂ | Ventilación deficiente, laboratorios | Desplaza al O₂ | No | NDIR (p. ej., SCD30) | 5000 ppm / 30000 ppm |
| NO | Voladuras | Tóxico | No | Electroquímico | 25 ppm |

\* **Verificar contra el texto oficial antes de citar.**
- Los valores del Anexo 15 se tomaron de una copia de los anexos (SlideShare) y de un resumen (Zamtsu). Las columnas TWA/STEL se dedujeron del orden de la tabla, porque el texto extraído no las rotula.
- El mínimo de 19,5 % de O₂ está en el **art. 246, literal b**, según la modificación del **DS 023-2017-EM** (El Peruano, 18-08-2017). Ese decreto **no modifica** el Anexo 15.
- El mismo art. 254 limita el escape de los equipos diésel a 500 ppm de CO y 100 ppm de NO₂ (es un límite de emisión, no de exposición).

## Integración propuesta
- **Unidad 1:** MQ-9 y los sensores que se añadan → ADC **ADS1115** por I²C → Raspberry Pi 5 → tópico ROS 2 con la concentración y la pose del robot (marca sobre el mapa de RTAB-Map).
- **Unidad 2:** sensor en la maestra ESP32-S3 si la tarea cooperativa le asigna medir en zonas estrechas.
- **Alarmas:** usar como umbrales los valores TWA/STEL de la tabla, y para el CH₄ las alarmas de 0,5 % (baja) y 1,0 % (alta).

> [!note] Calibración del MQ-9
> El MQ-9 alterna el calentador entre 5 V y 1,5 V, necesita un precalentamiento largo y tiene sensibilidad cruzada entre CO y CH₄. Para resultados cuantitativos hace falta calibrarlo con gas patrón.

## Pendiente
- [ ] Elegir qué gases entran en el alcance de la tesis.
- [ ] Verificar el Anexo 15 en el texto oficial (El Peruano / MINEM).
- [ ] Definir en qué unidad va cada sensor.
- [ ] ADC para los sensores analógicos (ADS1115).

> [!question] Pregunta abierta
> ¿El alcance se limita a **CO + CH₄** (con lo que basta el MQ-9 y se declara la limitación) o se suman sensores electroquímicos de O₂ y H₂S, que son los más críticos en socavones?

## Fuentes
- DS 023-2017-EM, modificación del DS 024-2016-EM: https://www.ighgroup.com/NormasLegalesPDF/MODIFICACI%C3%93N%20D.S.024-2016-EM.pdf
- DS 024-2016-EM, Anexos (copia en SlideShare): https://es.slideshare.net/slideshow/ds-0242016emanexos/250098724
- Resumen de límites: https://zamtsu.com/blog/limites-permisibles-gases-mineria/
- Texto oficial del DS 024-2016-EM: https://cdn.www.gob.pe/uploads/document/file/901782/DS-024-2016-EM.pdf

Relacionado: [[05-minem-osinergmin-gases-toxicos-ventilacion]] · [[normas-y-reglamentos]] · [[arquitectura-electronica-hexapodos]]
