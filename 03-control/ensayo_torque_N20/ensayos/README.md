# Registro de ensayos

Elige **esta carpeta** con el botón *Carpeta ensayos…* del panel. Cada sesión crea `AAAA-MM-DD_HHMMSS_motorNN/` con:

| Archivo | Contenido |
|---|---|
| `sesion.json` | Todo lo de la sesión; se puede reabrir en el panel |
| `resumen.csv` | Una fila por pulso, pesa o lectura |
| `0_vacio_*.csv`, `A_*.csv`, `B_*.csv`, `C_*.csv` | Telemetría (100 Hz; la C, 10 Hz) |

Las carpetas terminadas en `_SIMULADO` son de la demostración: **no son mediciones**.
