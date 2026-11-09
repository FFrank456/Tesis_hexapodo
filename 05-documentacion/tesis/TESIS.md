---
tipo: indice-maestro
tags:
  - tesis
---

# TESIS.md — Convenciones del sistema

## Datos del proyecto

- **Título:** Desarrollo de sistema robótico cooperativo inspirado en insectos para el monitoreo de terrenos accidentados
- **Curso:** Proyecto Final de Ingeniería Mecatrónica I (PFC I)
- **Universidad:** UTEC (Universidad de Ingeniería y Tecnología)
- **Equipo:** Alfredo Erazo Yalopalin, Aleskyjubinsl Alcarraz Jaime, Flavio Alarcón Muñoz
- **Asesor:** Dante Inga Narváez
- **Fecha de la presentación actual:** 2025-12-04
- **Formato de citación:** IEEE (numeración entre corchetes [n])
- **Idioma:** Español

## Resumen del proyecto (para contexto de Claude)

Dos **hexápodos** bioinspirados cooperan para entrar, mapear y detectar gases tóxicos en lugares irregulares o peligrosos: minas y socavones, zonas de desastre y laboratorios (ver [[problematica-y-aplicacion]]). **U1 (líder):** 18× MG996R, Pololu Mini Maestro 18, Raspberry Pi 5, Orbbec Gemini 2, RTAB-Map + Nav2, LiPo 3S. **U2 (modular):** 18 micromotores Pololu HPCB 12 V 100:1 (formato N20), 6× MOD_DRV (ESP32 + 3× DRV8876) en **bus CAN** hacia una maestra ESP32-S3, respaldo Wi-Fi/ESP-NOW, batería de ~14,4 V (4S), placas PCBicho v1.3. Control en cascada corriente-velocidad-posición en cada pata. Metodología VDI 2206/2225 + Pahl & Beitz. Detalle en [[arquitectura-electronica-hexapodos]] y [[2026-11-08-cambios-acordados]].

> [!note]- Resumen anterior (agosto 2026)
> Sistema robótico colaborativo (crawler + explorador cuadrúpedo bioinspirado) para inspeccionar minas subterráneas, mapear terreno y detectar gases tóxicos (CO, NOx) antes de que ingrese personal. Metodología VDI 2206/2225 + Pahl & Beitz. Arquitectura de control en cascada de tres lazos (corriente-velocidad-posición) sobre motor DC N20, hardware con ESP32 + TB6612FNG + ACS712, simulación en Gazebo/URDF, aprendizaje de marcha con Q-Learning, e integración planeada con ROS2/micro-ROS, SLAM y detección YOLO.

## Mapa de la bóveda

- [[TESIS]] (`05-documentacion/tesis/`) → archivo maestro: pregunta de investigación, objetivos, cronograma PFC I / PFC II
- `01-mecanica/` → [[01-mecanica]]: CAD, URDF, soportes, análisis de esfuerzos
- `02-electronica/` → [[02-electronica]]: arquitectura, MOD_DRV (Altium), PCBicho 6× MP1584 (KiCad), diagramas
- `03-control/` → [[03-control]]: control en cascada, calibración de servos, firmware
- `04-navegacion/` → [[04-navegacion]]: RTAB-Map + Nav2
- `05-documentacion/` → [[05-documentacion]]: capítulos (`tesis/capitulos/`), estado del arte, metodología, gases, cooperación, normativa, resultados, reuniones, referencias, adjuntos
- `daily-notes/` → avance diario
- `inbox/` → descarga rápida sin procesar
- `templates/` → plantillas reutilizables
- `resources/` → normativa (VDI 2206/2225), guías de estilo IEEE

## Pregunta de investigación

_(pendiente — completar)_

## Objetivos

_(pendiente — completar: objetivo general y específicos)_

## Cronograma PFC I / PFC II

_(pendiente — completar)_

## Convenciones de Obsidian (OBLIGATORIAS)

- SIEMPRE usa `[[doble corchete]]` para enlaces internos
- SIEMPRE usa tags con `#` (`#fuente`, `#capitulo`, `#hardware`, `#control`)
- Citas en formato IEEE: `[n]` en el texto, detalle completo en la nota de fuente
- Nombres de archivo en minúsculas con guiones
- Fechas en formato `YYYY-MM-DD`
- Toda afirmación técnica en un capítulo debe enlazar a la fuente `[n]` que la respalda

## Reglas de comportamiento

- Fuente nueva → `05-documentacion/estado-del-arte/` con `templates/fuente.md`, numerada según su `[n]` de aparición
- Avance en diseño mecánico/electrónico/control → `templates/componente-tecnico.md` en la carpeta correspondiente
- Reunión de equipo o con el asesor → `05-documentacion/reuniones/`
- Redacción de capítulo → `05-documentacion/tesis/capitulos/`, enlazando fuentes y componentes técnicos usados
- Prioriza trazabilidad entre: fuente citada → argumento del capítulo → componente técnico que lo sustenta
