# pata_lazo_abierto — contexto para Claude

Ciclo de paso de **toda la pata** (coxa, fémur, tibia) en **lazo abierto**, sobre la placa MOD_DRV. Es una copia del firmware de `../pata_MOD_DRV/` (mismos pines, sensado y telemetría, mismos comandos) con un modo nuevo. `pata_MOD_DRV` no se toca desde aquí. Tesis de Alfredo Erazo (UTEC). Responder en español, directo.

## Por qué existe
En `pata_MOD_DRV`, el control en cascada (corriente → velocidad → posición) pelea con el *stick-slip* de los bujes de PLA: la fricción (0.06–0.11 A) es del tamaño de la gravedad, y el control le quita corriente a la articulación cuando está trabada (tope del empujón, REVERSING con solo la corriente de gravedad). Aquí no hay lazo de corriente: el motor toma la corriente que necesite.

## Cómo funciona (`main/gait_open_loop.c`, lógica pura sin ESP-IDF)
- Las tres articulaciones van a la vez hacia la postura de turno. Cada una recibe un PWM fijo en el sentido del objetivo y frena (DRV8876 en freno) al llegar a la tolerancia o al pasarse.
- **Si no avanza 0.01 rad en `stall_ms` (150 ms), el PWM sube `pwm_step` (10 puntos), hasta 100 %.** Así toma lo que necesita aunque el PWM inicial no alcance.
- Si una articulación frenada se aleja más de 3 × tolerancia (gravedad), se vuelve a empujar ("re-empuje").
- Con las tres en su sitio durante `dwell_ms` (200 ms) pasa a la siguiente postura; al terminar la lista vuelve a la primera, hasta completar los ciclos.
- Se detiene y frena si: una articulación sale de su rango (`GAIT,RANGE`), una postura tarda más que `timeout_ms` (`GAIT,TIMEOUT`, dice qué articulación), salta el nFAULT o llega un STOP.
- Mientras corre, los motores están en modo OPEN; cualquier otro comando a un motor responde `ERR,GAIT_RUNNING` (STOP sí se acepta).
- Integración en `main/main.c`: `gait_tick` corre en la tarea de control (1 kHz) antes del paso de cada motor; los eventos se imprimen desde el lazo principal.

## Comandos
| Comando | Qué hace |
|---|---|
| `GAIT,DEFAULT` · `GAIT,CLEAR` · `GAIT,POSE,coxa,fémur,tibia` | Ciclo por defecto · vaciar · agregar una postura (máx. 8) |
| `GAIT,PWM,a,b,c` · `GAIT,TOL,a,b,c` · `GAIT,LIM,j,min,max` | PWM inicial (%) · tolerancia (rad) · rango de seguridad de la articulación j |
| `GAIT,STALL,paso,ms` · `GAIT,TIME,timeout_ms,pausa_ms` | Subida de PWM sin avance · tiempos |
| `GAIT,RUN,ciclos` · `GAIT,STOP` · `STOP` · `GAIT,INFO` | Correr · detener (freno) · detener todo · ver la configuración |

Eventos: `GAIT,POSE,ciclo,postura,ms,re-empujes,θc,θf,θt,PWMmáx_c,PWMmáx_f,PWMmáx_t`, `GAIT,DONE,ms`, `GAIT,TIMEOUT,ciclo,postura,Mj,ms,θ…`, `GAIT,RANGE,ciclo,postura,Mj,θ…`, `GAIT,STOPPED,ms`.

**Ciclo por defecto** (rad): apoyo adelante (0.30, 0, 0) → empujar (−0.30, 0, 0) → levantar (−0.30, −0.40, 0.20) → avanzar en el aire (0.30, −0.40, 0.20). En el fémur "arriba" es θ negativo (el 14-nov cae hacia +). **El sentido de la tibia no está verificado.**

## Dashboard
`dashboard/pata_lazo_abierto.html` (Chrome o Edge, Web Serial, Ctrl+F5 tras cambios). Edita el ciclo y los parámetros, "Enviar y correr", STOP con la barra espaciadora. Tabla por postura: tiempo, PWM máximo y corriente real máxima y media de cada articulación (corriente real estimada = medida ÷ ciclo de trabajo, porque el DRV8876 solo mide con el PWM encendido). "Descargar CSV" guarda la telemetría con los eventos al final (líneas `#`). Los CSV van a `datos/`.

## Compilar y probar
- Desde PowerShell: `. C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1`, luego `idf.py build` en esta carpeta. Puerto en `.vscode/settings.json` (COM16).
- La lógica se probó con una réplica en Python sobre una pata simulada (14-nov): ciclo completo, subida de PWM desde 30 %, timeout sin subida, rango y gravedad que no se puede levantar. En este PC no hay gcc para tests en C.
- El dashboard se probó con un ESP32 simulado en el navegador.
- **Todavía no se probó en la pata.**

## Cuidado con
- El disco C: de este PC se llena (14-nov: 0 MB libres, luego 0.47 GB). Una compilación necesita ~150 MB.
- Flashear este firmware reemplaza al de `pata_MOD_DRV` en el ESP32; para volver, flashear aquel.
