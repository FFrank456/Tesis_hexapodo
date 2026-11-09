# Tesis — Sistema robótico cooperativo bioinspirado (dos hexápodos)

UTEC · Ingeniería Mecatrónica · Asesor: prof. Dante Inga Narváez
Equipo: Alfredo Erazo Yalopalin, Aleskyjubinsl Alcarraz Jaime, Flavio Alarcón Muñoz

Dos hexápodos que cooperan para **mapear y detectar gases tóxicos** en terrenos irregulares o peligrosos: minas y socavones, zonas de desastre y laboratorios.

| Unidad | Resumen |
|---|---|
| **U1**, líder | 18× MG996R, Pololu Mini Maestro 18, Raspberry Pi 5, Orbbec Gemini 2, RTAB-Map + Nav2, LiPo 3S |
| **U2**, modular | 18 micromotores Pololu HPCB 12 V 100:1 (formato N20), 6× MOD_DRV (ESP32 + 3× DRV8876) en **bus CAN** hacia una maestra ESP32-S3, respaldo Wi-Fi/ESP-NOW, batería de ~14,4 V (4S), placas de alimentación PCBicho v1.3 |

## Estructura (bóveda de Obsidian)

| Carpeta | Contenido |
|---|---|
| `01-mecanica/` | Notas de diseño, CAD (STL/3MF), paquetes URDF, soportes |
| `02-electronica/` | Arquitectura, MOD_DRV (Altium), PCBicho 6× MP1584 (KiCad), diagramas |
| `03-control/` | Control de corriente/velocidad/posición (firmware `ControlCurrentN20`, ESP-IDF), calibración de servos |
| `04-navegacion/` | RTAB-Map + Nav2 (U1) |
| `05-documentacion/` | Tesis y capítulos, estado del arte, gases, cooperación, normativa, decisiones |

Abrir la carpeta como bóveda en Obsidian. Cada área tiene una nota índice con el mismo nombre que la carpeta.

## Qué no está en este repositorio (ver `.gitignore`)
- Mallas de los URDF, `base_link.stl` y archivos STEP: se regeneran desde Fusion 360 o Altium. Las piezas STL/3MF pequeñas sí se versionan.
- Historial, vistas previas y logs de Altium; historial de KiCad.
- Repositorios independientes:
  - `02-electronica/unidad2-PCB-pata-v1` → Altium 365
- PDF de referencias (en `05-documentacion/referencias/`, solo en local).
