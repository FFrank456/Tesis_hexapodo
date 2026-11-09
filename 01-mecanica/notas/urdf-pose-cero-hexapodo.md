---
tipo: componente-tecnico
area: mecanica
unidad: Unidad 1
fecha: 2026-10-19
archivo_cad: FrameAssembly_URDF
tags:
  - mecanica
  - urdf
  - ros2
---

# Pose cero del hexápodo para URDF (FrameAssembly_URDF)

Paquete exportado: `01-mecanica/urdf/Hexapodurdf_description/`. Las mallas STL no se suben a git por su tamaño: se regeneran al exportar desde Fusion.

## Estado del modelo en Fusion
- Pose cero = **estrella**: coxas a 45° / 0° / −45°, fémur y tibia horizontales y en línea, y las 18 articulaciones en 0 rad.
- Servo a 90° equivale a 0 rad en el URDF. Límites de los 18 joints: [−90°, +90°] (servo de 0 a 180°).
- Orientación: la **Gemini 2 mira hacia +X**, Y hacia la izquierda y Z hacia arriba (REP-103). El centro del cuerpo está en el origen (XY).
- Los soportes de la cámara y la Gemini 2 están fusionados en `base_link` (sin frame propio). Ver [[soporte-camara-raspberry-unidad1]].
- Joints de tibia recreados. Los puntos `old_tibia_*_joint_pt` quedan ocultos y no se usan.

## Nombres de las patas (convención de ROS)
- `r` = derecha (−Y), `l` = izquierda (+Y); `1` = delantera, `2` = media, `3` = trasera.
- Renombre aplicado: r3→l1, r2→l2, r1→l3, l3→r1, l2→r2, l1→r3.

## Convención de signos (corregir en el URDF exportado)
Fusion no deja invertir el sentido de los ejes, así que hay que invertir `axis xyz` en estos 9 joints:
- coxa_r1, coxa_r2, coxa_r3
- femur_r1, femur_r2, femur_r3
- tibia_l1, tibia_l2, tibia_l3

Resultado: coxa + = la pata va hacia adelante (+X); fémur + = sube; tibia + = baja.

## Pose de pie
- Coxa 0°, fémur 0° (horizontal), tibia +90° (vertical hacia abajo).
- En el servo: coxa 90, fémur 90 y tibia en un extremo del rango (0 o 180, según el lado).
- La punta queda ≈13,5 cm por debajo del eje del fémur. La tibia queda en el tope del servo, sin margen para flexionar más.

## Exportación
- Exportador: fusion2urdf-ros2 (`URDF_Exporter_Ros2`), sobre la versión guardada con las articulaciones en 0.
- **No guardar el modelo después de exportar:** el script lo modifica (deja `old_component`).

Relacionado: [[calibracion-tibias-hexapodo]] · [[navegacion-unidad1-nav2-rtabmap]]
