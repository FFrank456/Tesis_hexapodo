# Prueba CAN: maestra ESP32-S3 ↔ MOD_DRV

Prueba la comunicación CAN de la Unidad 2 en pasos cortos. Cada paso prueba una sola cosa, de modo que si algo falla se sabe en qué parte está el problema.

| Carpeta | Placa | Qué contiene |
|---|---|---|
| `comun/` | las dos | Toda la lógica de la prueba (`prueba_can.c`). |
| `maestra_s3/` | ESP32-S3 DevKitC-1 | Proyecto ESP-IDF de la maestra: TX = GPIO4, RX = GPIO5. Consola por el conector **USB** (no el UART). |
| `pata_mod_drv/` | MOD_DRV | Proyecto ESP-IDF de la pata 1: TX = IO21, RX = IO22. Consola por el USB-C. Pone a 0 las 6 entradas de los DRV8876. |

## Protocolo (v2, 21-nov)

Lo define `../pata_MOD_DRV/main/can_protocol.h`, y la maestra compila con ese mismo archivo. Funciona a 500 kbit/s con ID estándar.

**Idea:** la maestra solo dice **adónde** ir. La pata hace todo el movimiento con su propio control: cascada, trayectoria PLIMIT, gravedad, fricción y dither, igual que *Ir a posición* en el dashboard.

| ID | Sentido | Para qué |
|---|---|---|
| `0x0FF` | maestra → todas | PARO general (vacío). Responde DETENIDA |
| `0x101` | maestra → pata 1 | Órdenes (el primer byte dice cuál) |
| `0x201` | pata 1 → maestra | Respuestas (el primer byte dice cuál) |

| Orden (`0x101`) | Bytes | Respuesta |
|---|---|---|
| PING / latido | vacío | `00` PONG |
| ARMAR | `01` | `01` ACEPTADA o `03 motivo` RECHAZADA |
| IR_A | `02 máscara cL cH fL fH tL tH`: ángulo **absoluto** en mrad (int16, byte bajo primero). Máscara: bit0 coxa, bit1 fémur, bit2 tibia | ACEPTADA, RECHAZADA o `02` DESARMADA |
| ESTADO | `03` | `05 flags cL cH fL fH tL tH`: posición actual en mrad. Flags: bit0–2 llegó, bit3–5 nFAULT, bit6 armada |
| CERO | `04 máscara`: el encoder vale 0 en la pose actual (solo desarmada) | ACEPTADA o RECHAZADA |
| DESARMAR | `05` | `04` DETENIDA |
| IDENT | `06` | `06 mac0…mac5`: la MAC de la placa. Sirve para detectar dos placas con el mismo ID |

**ID de pata (22-nov):** las órdenes de la pata *n* van por `0x100 + n` y sus respuestas por `0x200 + n`, con n de 1 a 6. El ID ya no se compila: se asigna una vez por serie con `CAN,ID,n` (`CAN,ID` lo muestra) y queda en NVS, aparte de las ganancias, así que `NVS,ERASE` no lo borra. Una placa **sin ID** no atiende órdenes; solo obedece el PARO, sin responder.

Ejemplo: "fémur a 0.5 rad" se envía como `02 02 00 00 F4 01 00 00` (`0x01F4` = 500 mrad).

**Motivos de RECHAZADA:** 1 formato, 2 fuera de rango, 3 nFAULT, 4 no está parada, 5 ya armada, 6 el control no arrancó o cambió el cero.

**Seguridad:**
- **Rangos** respecto del cero del encoder: coxa −0.5…1.5, fémur −1…1 y tibia −1…0.7 rad. Son los del dashboard.
- **Latido:** con la pata armada, si pasan **0,5 s sin ninguna orden** de la maestra, la pata suelta las articulaciones. La maestra manda ESTADO cada 100 ms automáticamente.
- Mientras la pata está armada, rechaza los comandos serie de motor, como las ganancias del dashboard. `STOP` por serie la desarma.
- **El cero importa:** el firmware pone el cero del encoder donde esté la pata al arrancar. La gravedad calibrada supone el cero con el **fémur horizontal y la tibia alineada con él**. Pon la pata así y usa `cero todo` (o *Cero los 3* en el dashboard) antes de armar.

Solo el firmware de prueba de la pata (`pata_mod_drv/`) envía además `0x201` con 4 bytes de contador (tecla `p`).

Las dos placas arrancan en modo **ESCUCHA**: en ese modo no transmiten ni dan ACK. Solo usan el bus después de pulsar `n`.

## Cableado

| ESP32-S3 | Módulo CAN | | MOD_DRV (J4) | Módulo WCMCU-230 |
|---|---|---|---|---|
| GPIO4 | CTX / TXD | | IO21 (pin 3) | CTX |
| GPIO5 | CRX / RXD | | IO22 (pin 4) | CRX |
| 3V3 | 3V3 / VCC | | 3V3 (pin 1) | 3V3 |
| GND | GND | | GND (pin 2) | GND |

Entre los dos módulos se conectan **CANH con CANH**, **CANL con CANL** y **GND con GND**. En la MOD_DRV el bus sale por P1 o P2: pin 1 CANH, pin 2 CANL, pin 3 GND. TX va con CTX y RX con CRX, sin cruzar: el transceptor no funciona como un UART.

## Compilar y cargar

Usa VS Code con la extensión ESP-IDF y **abre cada carpeta por separado** (`maestra_s3` o `pata_mod_drv`), no la carpeta `prueba_can`. El puerto se toma de `.vscode/settings.json` de cada una: COM4 para el S3 y COM16 para la MOD_DRV. Si el Administrador de dispositivos muestra otros números, cámbialos ahí.

Desde PowerShell:

```powershell
. C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1
cd 03-control\prueba_can\maestra_s3
idf.py -p COM4 flash monitor
```

```powershell
. C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1
cd 03-control\prueba_can\pata_mod_drv
idf.py -p COM16 flash monitor
```

Si la carga falla con `ninja: failed recompaction: Permission denied`, la extensión de VS Code tiene bloqueada la carpeta `build` y la placa se queda con el firmware anterior, aunque parezca que cargó. Compila y carga en una carpeta aparte con `idf.py -B build_cli -p COMx flash monitor`. La carga es correcta solo si al final aparece `Hash of data verified`.

La primera compilación tarda de 3 a 5 minutos y las siguientes, segundos. Usa una terminal para cada placa para ver los dos monitores al mismo tiempo. Los comandos se **escriben y se confirman con Enter**, y se sale del monitor con `Ctrl+]`. Si aparece "Acceso denegado", el dashboard u otro monitor tiene el puerto abierto.

## Comandos

| Tecla | Placa | Qué hace |
|---|---|---|
| `h` | las dos | Ayuda |
| `s` | las dos | Estado: modo, controlador, nivel de CRX, TEC/REC (contadores de errores de TX/RX), errores de bus, contadores |
| `r` | las dos | Mide CRX durante 100 ms; en reposo debe estar siempre en 1 |
| `t` | las dos | Autotest local: 10 tramas que salen por CTX y vuelven por CRX, sin necesitar a la otra placa |
| `n` / `l` | las dos | Modo NORMAL / modo ESCUCHA |
| `z` | las dos | Contadores a cero |
| `p` | maestra | Ping a la pata 1 y espera el PONG |
| `p` | pata | Envía un contador a la maestra |
| `b` | maestra | Ráfaga de 1000 pings: pérdidas y tiempo de ida y vuelta |
| `k` | maestra | PARO general `0x0FF` |

Comandos de movimiento de la maestra, contra el firmware real `pata_MOD_DRV`. Antes hay que poner la maestra en NORMAL con `n`:

| Comando | Qué hace |
|---|---|
| `a` | Armar la pata. Las tres articulaciones tienen que estar paradas |
| `f 0.5` | Fémur a 0.5 rad |
| `t -0.3` | Tibia a −0.3 rad |
| `c 0.2` | Coxa a 0.2 rad |
| `ft 0.3 -0.2` | Fémur y tibia a la vez |
| `e` | Estado: posiciones, nFAULT y si está armada |
| `d` | Desarmar: suelta las articulaciones |
| `cero f` / `cero t` / `cero c` / `cero todo` | El encoder vale 0 en la pose actual (con la pata desarmada) |

Con varias patas:

| Comando | Qué hace |
|---|---|
| `escanear` | Pregunta a los IDs 1–6: `SCAN,n,OK,<MAC>`, `SCAN,n,NO` o `SCAN,n,DUPLICADA` |
| `p3` | Elige la pata 3; las órdenes siguientes van a ella |
| `p3 f 0.5` | Elige la pata 3 y le manda la orden |
| `d todas` | Desarma todas las patas |
| `k` | PARO a las 6; lista cuáles confirmaron |

La maestra manda el latido a **todas** las patas armadas y no arma ni mueve una pata cuyo ID salió duplicado en el último escaneo.

### Interfaz web: `maestra_dashboard.html`

El selector de arriba muestra las 6 patas (*responde*, *armada*, *nFAULT*, *ID repetido*, *no responde*). Los controles y la gráfica son de la pata elegida, y cada orden sale con su número de pata.

Ábrela en Chrome o Edge. Pulsa *Conectar maestra* y elige el puerto del S3 (COM4); la interfaz pone sola el bus en NORMAL y pide el estado continuo (`v 1`). Desde ahí puedes:

- armar, desarmar y poner el cero;
- mover cada articulación con su barra, su casilla o los botones ±0.1;
- mover fémur y tibia a la vez;
- reproducir una secuencia de poses (`fémur tibia [pausa]` por línea), que espera a que lleguen las dos antes de pasar a la siguiente;
- ver las posiciones en una gráfica de 20 s.

**PARO** (o la tecla **Esc**) manda el paro general. La interfaz usa el puerto de la **maestra**, así que el dashboard de la pata puede seguir abierto en COM6 para ver corrientes, aunque mientras la pata está armada rechaza los comandos serie.

Mientras algo se mueve, la maestra imprime `POS,...` cada 0,5 s y `LLEGO,femur,+0.498 rad,1.4 s` cuando la articulación llega. Se puede enviar un objetivo nuevo en pleno movimiento.

## Secuencia de prueba

Haz los pasos en orden y no pases al siguiente hasta que el anterior dé el resultado esperado. **Los motores van desconectados** durante toda la prueba.

### Paso 0. Sin energía: medir el bus

Con las dos placas apagadas y los dos módulos unidos por CANH/CANL/GND, mide con el multímetro la resistencia entre CANH y CANL.

- **≈ 60 Ω:** correcto. Cada WCMCU-230 trae su resistencia de 120 Ω y las dos quedan en paralelo.
- **≈ 120 Ω:** solo hay una terminación. Funciona en banco con cables cortos, pero anótalo.
- **≈ 40 Ω o menos:** hay tres o más resistencias de 120 Ω en paralelo.
- **0 Ω:** CANH y CANL están en corto. No enciendas las placas hasta corregirlo.
- **Abierto:** el cable no hace contacto.

### Paso 1. Cada placa sola: arranque y CRX en reposo

Desconecta el bus entre los módulos. Carga el firmware en las dos placas y abre su monitor.

Qué debe pasar en cada placa:

```
=== PRUEBA CAN: ... ===
MODO,ESCUCHA
CRX,muestras=2000,en_0=0 -> OK, reposo en 1
ESTADO,modo=ESCUCHA,controlador=CORRIENDO,CRX=1,...
```

En el S3 el arranque ocurre antes de que se abra el monitor y esas líneas se pierden. Pulsa `r` y luego `s` para verlas.

- Si sale **"CRX pegado en 0"**, el transceptor, su alimentación o la soldadura del zócalo tienen un problema. Es la falla que tuvo una MOD_DRV el 07-11. Mide con el multímetro 3,3 V en el pin VCC del módulo y el voltaje en CRX: en reposo debe dar ≈ 3,3 V.
- Si sale **"ERROR,instalar_twai"**, los pines están mal definidos en el `main.c` de esa placa.

### Paso 2. Cada placa sola: autotest

Sin conectar todavía el bus, pulsa `t` en cada placa. La trama sale por TX → CTX, pasa por el transceptor y vuelve por CRX → RX.

Debe terminar en `AUTOTEST,PASA,10/10`.

- Si da **`tx=ACK_OK` pero `eco=NO`**, la trama sale pero no regresa: revisa el cable CRX → RX.
- Si da **`tx=NO_SALIO`**, el controlador no puede transmitir: revisa CTX → TX o un CRX en 0.
- Si da **menos de 10/10 de forma intermitente**, hay un contacto flojo. Presiona el módulo en el zócalo y repite.

Esto confirma que cada placa funciona por su cuenta antes de juntarlas.

### Paso 3. Bus conectado, las dos en ESCUCHA

Apaga las dos placas, conecta CANH/CANL/GND entre los módulos y vuelve a encenderlas. Pulsa `r` y `s` en cada una.

Debe salir `OK, reposo en 1` en las dos, con `errores_bus=0`. Si al conectar el bus una placa pasa a CRX = 0, una de las dos está forzando el bus. Desconecta la otra para saber cuál es.

### Paso 4. Maestra → pata (con ACK)

1. En la **pata**, pulsa `n`. Debe salir `MODO,NORMAL`.
2. En la **maestra**, pulsa `n` y después `p`.

En la maestra debe aparecer:

```
TX,id=0x101,dlc=0 (ping) -> ACK_OK
RESPUESTA,id=0x201,PONG,ida_y_vuelta_us=...
PASA,ping
```

En la pata debe aparecer:

```
RX,id=0x101,dlc=0,datos=
  -> respondo PONG en 0x201
```

- **`SIN_ACK`:** la trama salió, pero nadie la confirmó. Revisa que la pata esté en NORMAL, que CANH y CANL no estén invertidos y que GND sea común. Si fallan varios seguidos, TEC sube de 8 en 8; al pasar de 255 la maestra entra en BUS_OFF y vuelve sola a ESCUCHA.
- **`ACK_OK` pero sin respuesta:** la pata recibe, pero su respuesta no llega. Pasa directamente al paso 5 para revisar ese sentido.

### Paso 5. Pata → maestra

Con las dos en NORMAL, pulsa `p` en la **pata** varias veces.

En la pata debe salir `TX,id=0x201,dlc=4,contador=1 -> ACK_OK`. En la maestra debe salir `RX,id=0x201,dlc=4,datos=01 00 00 00`, y el contador debe aumentar cada vez.

Con los pasos 4 y 5 superados, el enlace funciona en los dos sentidos.

### Paso 6. Ráfaga de 1000 pings

En la maestra, pulsa `b`. Tarda menos de 2 segundos.

```
RAFAGA,fin,enviados=1000,ok=1000,sin_ack=0,sin_respuesta=0,ida_y_vuelta_us_min=...,prom=...,max=...
PASA,rafaga
```

Anota en la nota de la tesis cuántas tramas se perdieron y el tiempo de ida y vuelta. Repite la ráfaga con un cable de 1 m o más, y después con la fuente de motores encendida cerca. La pata imprime como mucho una línea cada 100 ms para no frenar la consola, así que en su monitor verás `(N tramas mas sin imprimir)`.

### Paso 7. Contra el firmware real de la pata

Carga en la MOD_DRV el firmware de control de `../pata_MOD_DRV` y deja el S3 con esta prueba. En el monitor de la pata debe aparecer `CAN,READY,0x101,500000`.

En la maestra, pulsa `n` y después `p`: debe responder `PONG`, porque el firmware real atiende el mismo ping. Pulsa `k` y debe responder `DETENIDA`. Con esto se comprueba que la maestra puede hablar con el control real de la pata.

### Paso 8. Mover la pata desde la maestra

1. Con el dashboard, envía las ganancias de cada articulación (*Enviar al ESP32*), porque se pierden al reiniciar. Después cierra el dashboard o desconéctalo.
2. Pon el fémur horizontal y la tibia alineada con él. En la maestra: `n`, `cero todo` y `e`, que debe dar 0 en las tres.
3. Enciende V_MOT. Después `a` para armar, `f -0.3` para que el fémur suba 0.3 rad (el encoder negativo lo sube), `t 0.3`, `ft 0 0`.
4. `d` para soltar. Ojo: al desarmar, el fémur queda sin fuerza y cae.
