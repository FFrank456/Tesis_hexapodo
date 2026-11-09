---
tipo: capitulo-borrador
capitulo: introduccion
fecha: 2026-11-09
fuente: [[2026-11-08-cambios-acordados]]
tags:
  - tesis
  - problematica
  - introduccion
---

# Problemática y aplicación

> [!abstract] Enunciado
> Dos hexápodos bioinspirados cooperan para **entrar, mapear y detectar gases tóxicos** en lugares de terreno irregular o peligrosos para las personas: **minas y socavones, zonas de desastre (por ejemplo, después de un terremoto) y laboratorios**.

## Por qué robots con patas
- Las patas superan escombros, escalones y suelos sueltos donde las ruedas se atascan o vuelcan.
- Un hexápodo mantiene 3 apoyos en todo momento (marcha trípode): es estáticamente estable en terreno irregular.
- Bioinspiración: cada pata tiene control local y la marcha surge de la coordinación, como en los insectos (Walknet). Ver [[06-literatura-robotica-bioinspirada-insectos]] y [[15-robots-insectos-espacios-estrechos]].

## Papel de cada unidad
| Unidad | Rol | Hardware clave |
|---|---|---|
| **U1**, hexápodo de servos (líder) | Mapeo con RTAB-Map + Nav2, navegación autónoma y medición de gases | 18 MG996R, Mini Maestro 18, Raspberry Pi 5, Orbbec Gemini 2, LiPo 3S. Ver [[navegacion-unidad1-nav2-rtabmap]] |
| **U2**, hexápodo modular | Exploración; cada pata opera aunque falle la comunicación | 18 micromotores Pololu HPCB 12 V 100:1 (formato N20), 6 MOD_DRV (ESP32 + DRV8876) en bus CAN, maestra ESP32-S3, batería de ~14,4 V. Ver [[arquitectura-electronica-hexapodos]] |

## Escenarios de aplicación
1. **Minería subterránea y socavones:** gases de voladura y de motores diésel (CO, NO₂), falta de O₂, metano en minas de carbón y H₂S/SO₂ en minas polimetálicas. Antecedentes en [[05-minem-osinergmin-gases-toxicos-ventilacion]], [[11-minem-osinergmin-accidentes-ventilacion]] y [[14-minem-osinergmin-accidentes-ventilacion]].
2. **Zonas de desastre:** estructuras colapsadas, con fugas de gas y acceso restringido para rescatistas.
3. **Laboratorios:** fugas de gases (CO₂, gases combustibles) en espacios cerrados.

Gases y umbrales: [[gases-a-monitorear]]. Marco normativo: [[normas-y-reglamentos]]. Cooperación: [[nivel-de-cooperacion]].

## Alcance (validación)
La evaluación se hace **en condiciones controladas de laboratorio**: precisión del mapeo, efectividad en la detección de gases y estabilidad de la comunicación (objetivo específico 4). La operación en atmósferas explosivas reales queda fuera del alcance (ver IEC 60079 en [[normas-y-reglamentos]]).

> [!question] Preguntas abiertas
> - ¿Las tres aplicaciones (minería, desastres, laboratorio) van al mismo nivel, o la minería es la principal y las otras quedan como extensiones? El título y el objetivo general actuales hablan solo de minería subterránea.
> - Falta definir la **tarea cooperativa concreta** que se va a evaluar (ver [[nivel-de-cooperacion]]).
