# TESIS_2 — contexto para Claude

Tesis de Alfredo Erazo (UTEC): sistema cooperativo de dos hexápodos bioinspirados para mapear y detectar gases tóxicos en minería subterránea. Responder en español, directo. Si no encuentras un dato, búscalo antes de responder.

- **Unidad 1:** hexápodo de servos MG996R con Raspberry Pi 5, Orbbec Gemini 2, Nav2 y RTAB-Map.
- **Unidad 2:** hexápodo modular. Cada pata tiene una placa MOD_DRV (ESP32 + 3 DRV8876) con motores N20, y las patas se comunican por CAN con una maestra ESP32-S3.

| Carpeta | Contenido |
|---|---|
| `01-mecanica/` | Diseño mecánico (Fusion) |
| `02-electronica/` | Placas, incluida la MOD_DRV |
| `03-control/` | Firmware. El trabajo activo está en `03-control/pata_MOD_DRV/` (control en cascada de una pata, con su propio `CLAUDE.md`) y en `03-control/pata_lazo_abierto/` (ciclo de paso de toda la pata en lazo abierto, también con su `CLAUDE.md`) |
| `04-navegacion/` | Navegación de la Unidad 1 |
| `05-documentacion/` | Notas de la tesis (bóveda tipo Obsidian) |

Cuando le pidas al usuario ejecutar algo, explica cada paso: qué hacer, cuánto tarda, qué debería pasar y qué hacer si no funciona.
