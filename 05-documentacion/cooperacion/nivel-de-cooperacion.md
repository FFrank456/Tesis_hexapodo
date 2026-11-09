---
tipo: marco-teorico
fecha: 2026-11-09
fuente: Farinelli, Iocchi y Nardi (2004)
tags:
  - cooperacion
  - multirobot
  - marco-teorico
---

# Nivel de cooperación entre las unidades

**Propuesta:** un equipo **heterogéneo**, **consciente**, **fuertemente coordinado** y **débilmente centralizado**, según la taxonomía de Farinelli, Iocchi y Nardi (2004).

| Dimensión | Niveles posibles | Propuesta para la tesis |
|---|---|---|
| Composición | Homogéneo / heterogéneo | **Heterogéneo:** la U1 tiene servos y cámara; la U2 es modular con micromotores N20 |
| Conocimiento | Consciente / no consciente del otro | **Consciente:** comparten su estado por micro-ROS (Wi-Fi UDP) |
| Coordinación | Fuerte / débil / ninguna | **Fuerte:** la U1 asigna metas y comparte el mapa |
| Organización | Centralizada fuerte / débil / distribuida | **Débilmente centralizada:** la U1 es la líder y la U2 sigue operando si pierde el enlace |
| Comunicación | Explícita / implícita | **Explícita** (mensajes ROS 2) |

## Dos niveles de cooperación
1. **Entre robots:** la U1 (líder) y la U2 se comunican por micro-ROS/Wi-Fi.
2. **Dentro de la U2:** cada pata es un agente local. El bus CAN es el enlace principal y el Wi-Fi o ESP-NOW, el respaldo. Ver [[justificacion-esp32-por-pata]].

## Tarea cooperativa (por definir)
- **Opción A, división por capacidad:** la U1 mapea y la U2 entra a zonas estrechas a medir gas en los puntos que la U1 le asigna sobre el mapa.
- **Opción B, división del área:** ambas reparten el área y fusionan sus mediciones en un mapa común.

> [!question] Pregunta abierta
> ¿Qué tarea cooperativa se evaluará en el laboratorio, la A o la B? Y si la U2 no lleva cámara de profundidad, ¿cómo se localiza en el mapa de la U1 (marcador visual, odometría de patas, seguimiento con la cámara de la U1)?

> [!warning] Riesgo de comunicación
> En socavones, el Wi-Fi tiene un alcance limitado por las curvas y la roca. Conviene medir la estabilidad del enlace (objetivo específico 4) y definir qué hace la U2 si lo pierde.

## Referencia
A. Farinelli, L. Iocchi y D. Nardi, "Multirobot systems: A classification focused on coordination," *IEEE Trans. Syst., Man, Cybern. B, Cybern.*, vol. 34, n.º 5, pp. 2015–2028, oct. 2004. https://profs.scienze.univr.it/~farinelli/pubs/farinelli-TSMC-04-abstract.html

Relacionado: [[problematica-y-aplicacion]] · [[navegacion-unidad1-nav2-rtabmap]]
