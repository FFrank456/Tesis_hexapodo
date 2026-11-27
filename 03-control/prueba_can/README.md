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

### Robot completo, fase 1: `robot_dashboard.html` (25-nov)

Lectura de las seis patas, cinemática inversa y marcha **simulada**. No arma ni mueve motores: `comun/robot_sim.c` no usa CAN y la página solo envía `n`, `v`, `escanear`, `s`, los comandos de abajo y el PARO (que suelta los motores).

| Archivo | Qué hace |
|---|---|
| `comun/robot.c` | Directa, inversa, montaje de las 6 patas, marchas, validación y margen de estabilidad (cálculo puro) |
| `comun/robot_sim.c` | Comandos y simulación en la maestra |
| `tests/test_robot.js` | `node tests/test_robot.js`: traduce `robot.c` a JavaScript y prueba la misma lógica (doble precisión, no la del S3) |

| Comando | Qué hace |
|---|---|
| `robot` · `robot defecto` | Muestra la configuración · la restablece |
| `marcha ondulada\|ripple\|tripode` | Tipo de marcha |
| `config radio altura x y giro elevacion ciclo marcha` | Todo a la vez (también `postura`, `paso`, `marcha` y `ciclo` por separado). Solo se aplica si el ciclo entero es posible: alcance, límites, estabilidad, separación ≥ 25 mm entre patas vecinas y velocidad ≤ 2 rad/s |
| `guardar` | Configuración y mapa en la flash de la maestra; al arrancar se usan solo si siguen siendo válidos |
| `ciclo s` | Duración del ciclo, 4 a 60 s |
| `sim 1` · `sim pausa` · `sim sig` · `sim 0` | Simulación |
| `mapa` + 6 IDs | ID CAN de cada posición: DD MD TD TI MI DI |
| `ci x y z` · `ci test` | Inversa de una pata (mm, marco de J0) · 2000 poses de ida y vuelta en la propia placa |

**Referencia (vista superior del usuario, 25-nov):** la pata *n* lleva el ID *n* y el frente es el lado de las patas 1 y 6, donde va la cámara. Vistas desde arriba van en sentido antihorario: 1 delantera, 2 media y 3 trasera de un lado; 4 trasera, 5 media y 6 delantera del otro. "Derecha" (1-2-3) e "izquierda" (4-5-6) son las de quien mira al robot de frente: el lado 1-2-3 es el izquierdo del propio robot y es +y. La numeración de `fusion-ensamble/1_estructura.py` no coincide con esta: no se usa para los IDs.
**Seguridad de la maestra (25-nov):** el PARO sale aunque la maestra esté en ESCUCHA; con patas armadas se bloquean `t`, `l` y `r`, que cortaban el latido, y `n` ya no reinicia el bus; tras un BUS_OFF las patas dejan de figurar armadas; un ESTADO o PONG atrasado no cuenta como respuesta a una orden; una línea de más de 63 caracteres se descarta entera.

**Recuperación tras BUS_OFF (25-nov):** la maestra escucha sin transmitir y vuelve sola a NORMAL si CRX estuvo en 0 menos del 60 % del tiempo (espera de 1 s, hasta 5 s si se repite). La espera supera los 0,5 s del latido para que las patas ya se hayan soltado; nunca rearma. `n`, `l` y `t` la cancelan. Con `v 1` y ninguna pata armada, busca cada 0,5 s una pata que no responde. No corrige un bus bloqueado por hardware.

**Velocidad del bus (25-nov):** la maestra y las seis patas usan CAN a **500 kbit/s**, fijo en el firmware (`TWAI_TIMING_CONFIG_500KBITS()` en `comun/prueba_can.c` y en `pata_MOD_DRV/main/main.c`). En ESP-IDF 5.4.4 eso es un reloj de 10 MHz, 20 cuantos por bit (1 + 15 + 4), muestreo al 80 % y SJW 3, igual en el S3 y en el ESP32. Cambiarla exige recompilar y cargar las siete placas. La consola USB es otra cosa y no tiene que coincidir: 115200 en la maestra y 460800 en las patas.

| Dato (trama de 8 bytes, ~130 bits) | Valor |
|---|---|
| Una trama | ≈ 0,26 ms |
| Ida y vuelta medida (ping, 25-nov) | 0,26–0,35 ms de media, máx. ≈ 1 ms |
| Ronda a las 6 patas | ≈ 3 ms |
| Bus ocupado a 20 / 50 / 100 / 200 Hz | ~6 / 16 / 31 / 62 % |

A 500 kbit/s el bus admite unos 100 m de cable, y aquí mide menos de 2 m. El control rápido corre dentro de cada placa, y por el CAN solo viajan consignas y estados (20–50 Hz alcanzan), así que la velocidad sobra. Bajarla no arregla un bus bloqueado por hardware.

**Pruebas de aislamiento del bus (25-nov), sin armar:**

| Conexión | Resultado |
|---|---|
| Torre 1-5-6 al mover cables | Bus bloqueado (CRX en 0, BUS_OFF); 0 de 153 escaneos completos |
| Maestra + 5 | 3000/3000 pings, 0 errores; CRX en 1 |
| Placa 5 sola (`CAN,STATUS` por USB) | leg_id 5, 0 errores |
| Maestra + 5 + 6 | 10/10 escaneos con las dos, 4000/4000 pings, 0 errores |
| Maestra + 1 + 5 + 6 | Bloqueado al conectar la 1: CRX en 0 desde el inicio, 0/10 escaneos, BUS_OFF |
| Maestra + 5 + 6, sin CANH/CANL de la 1 | Bus libre otra vez, la 5 da 1000/1000. La 6 se cae porque se conectaba por el conector de la 1 |
| Placa 1 aislada (`CAN,STATUS` por COM6) | CRX en 0, REC 128, 6,1 M errores de bus que suben ~30 000/s, **TEC 0**: su programa no transmite, pero su módulo ve dominante |
| Maestra + 2 + 5 + 6, con `diag_placa` | 121/121 escaneos por pata, 1000/1000 pings a cada una (ida y vuelta 262–385 µs de media, máx. 1,2 ms), TEC/REC 0, 0 errores |
| Falta | Placas 3 y 4. Reparar la placa 1 |

**Registro de errores del 25-nov:**

- **Placa 1: bloquea el bus.** Basta conectar **solo su CANL** para que el bus quede en dominante. Aislada, su propio módulo ve dominante. Causas posibles, de más a menos probable:
  - un puente de estaño entre CANL y GND, que en P1/P2 son pines vecinos (2 y 3);
  - el cable o el conector cruzado;
  - un corto CANL–GND en el módulo;
  - CTX (IO21 / J4-3) en 0 V;
  - el transceptor dañado.

  Para distinguirlas, medir con la placa apagada CANL–GND (< 10 Ω = corto) y, encendida, CANH, CANL y CTX (≈ 2,3 V, ≈ 2,3 V y 3,3 V). Pendiente de medir.
- **Maestra: módulo con CRX flotando (18:55).** CRX saltaba al azar y no seguía a CTX, con 0/200 tramas en el autotest. Era un cable suelto o falta de alimentación del módulo. Se arregló reacomodando los cables: después pasó tres veces seguidas (200/200).
- **Bus otra vez en dominante (≈ 19:40)**, con la maestra sana (su CTX en 1). Ocurrió después de manipular las placas. Pendiente de aislar pata por pata.
- **COM9 ocupado ("acceso denegado")** al cargar: lo tenía el dashboard abierto. Hay que desconectar el dashboard antes de usar `esptool`.
- **Consola de la maestra por el conector UART (CH343, COM9):** `diag_placa` usa allí UART a 115200. `prueba_can` sigue usando el USB nativo (COM4).

**Registro del 26 y 27-nov:**

- **Maestra con MCP2551 (CJMCU-2551) + conversor de nivel SparkFun BOB-12009 en RXD + 5 V externos.** Sin 5 V daba `MODULO_NO_TRANSMITE`. El CJMCU-2551 viene **sin la 120 Ω** (R2 y C2 vacías), y sin terminación el regreso a recesivo tardaba 6,9 µs: TWAI 0/200. Con 120 Ω entre CANH y CANL: TWAI 200/200, aguante 20 000/20 000 y subida de ≈ 630 ns. Con una 10 kΩ de CTX a 3,3 V, `ctx` pasa. Después, una prueba dio CRX fijo en 0 (`MODULO_DOMINANTE`): falta revisar los 5 V, el conversor y la soldadura de la 10 kΩ.
- **Placa 6: bloquea el bus otra vez, ahora con el módulo nuevo.** El módulo se quedó **sin VCC** (soldadura abierta). Sin VCC, el SN65HVD230 se alimenta por su entrada D (CTX) a través del diodo de protección, CTX cae a 0–0,7 V y el bus queda dominante. Al resoldar VCC (3,29 V), el ESP32 ya sube CTX, pero el chip quedó dañado. Con `ctx1` mide: D 1,5 V, IO21 0,78 V, IO21↔D 0,3 Ω, CANH 2,7 V, CANL 0,3 V, **CANH−CANL 2,4 V (dominante en reposo)**, Rs 1,1 V. Su entrada D tiene una fuga a GND: hay que **cambiar el SN65HVD230 (VP230)**, o usar el VP231 o el VP232, que son compatibles pata a pata. Hasta entonces, la placa 6 va fuera del bus.
- **Lección:** un transceptor sin VCC con el ESP32 encendido se daña y bloquea el bus. Revisar la soldadura de VCC del módulo en todas las placas.

**Supuestos por confirmar en el robot:** los signos de encoder son iguales en las seis (en Fusion son copias giradas, no espejadas); el cero es la pose cero del CAD; el centro de masa está en el centro del cuerpo. Los límites usados son la intersección del CAN y los provisionales de MATLAB: coxa −0.5…0.6, fémur ±1 y tibia −1…0.7 rad.

**Resultado:** con tibia ≤ 0.7 rad, radio 182 y altura 75 mm, el paso máximo es de 28 mm con 15 mm de elevación, 18 mm con 20 mm y ninguno (< 2 mm) con 25 mm. El límite de la tibia es lo que más restringe la marcha; medir su rango real antes de ampliarlo.
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

## Diagnóstico de una placa sola (`diag_placa/`)

Prueba el CAN de **una placa sin otra en el bus**. Una placa sola no puede hacer un intercambio CAN real, porque nadie le da el ACK, así que prueba cada tramo por separado: la línea CTX, el eco del módulo, el reposo del bus y el controlador TWAI en modo NO_ACK con autorrecepción. Sirve para encontrar la placa que bloquea el bus. El mismo código compila para la MOD_DRV (`esp32`) y para la maestra (`esp32s3`).

| Prueba | Qué hace | Si falla |
|---|---|---|
| 1 `ctx` | CTX libre, solo con el pull-up interno | Corto de CTX (J4-3) a GND: esa placa bloquea el bus |
| 2 `eco` | CTX=1 → CRX debe dar 1; CTX=0 → CRX debe dar 0, por GPIO | Corto en CTX, módulo sin 3,3 V, CANH/CANL en corto o módulo dañado |
| 3 `reposo` | CTX=1 durante 100 ms: CRX nunca debe bajar | Módulo o bus en dominante |
| 4 `twai` | 200 tramas a 500 kbit/s con datos verificados | Controlador, pines o contacto intermitente |
| 5 `motores` | nFAULT de los 3 DRV8876 y encoders (solo MOD_DRV) | Driver en falla o encoder sin señal |

Al final imprime `RESULTADO,CAN,PASA` o la falla con la causa probable y qué medir. Para que el eco y el reposo culpen a la placa, sus CANH y CANL deben estar sueltos: conectada, un bus forzado por otra placa da CRX en 0.

**Bus con las patas, desde la maestra** (solo IDENT, PING y ESTADO; nunca arma):

- `escanear [N]`: qué patas responden, con su MAC, en N rondas.
- `rafaga P [n]`: n pings a la pata P.
- `salud S H`: S segundos pidiendo ESTADO a todas las patas, H ciclos por segundo. Informa cada segundo de las respuestas, la latencia, TEC/REC, los errores, los BUS_OFF y la carga.

Los resultados posibles son:

- `SIN_ACK`: nadie más hay en el bus.
- `NO_RESPONDE`: hay placas en el bus, pero ninguna con ese ID.
- `NO_SALIO`: el bus está pegado en dominante.

**Robustez y medidas:**

- `aguante N`: repite N veces las pruebas 1–4 y resume cuántas rondas pasan.
- `retardo`: lazo CTX→módulo→CRX en ns. Se juzga por el promedio, porque la medida incluye ~550 ns del propio ESP32.
- `ctx1`, `ctx0` y `ctxlibre`: dejan CTX fijo para medir el transceptor con el multímetro. Con CTX=1, CANH y CANL deben dar ≈ 2,3 V y R ≈ 3,3 V. Con CTX=0, CANH−CANL debe dar más de 1,5 V.

Otros comandos: `escucha` (5 s sin transmitir, con la placa conectada al bus), `m` (monitor cada 1 s; gira a mano una articulación y deben cambiar sus cuentas), `cero`, `i` (MAC e ID de pata) y `h`.

**Seguridad.** Los 6 IN de los DRV8876 quedan en 0, así que no se mueve ningún motor. La NVS solo se lee: el ID de pata y la configuración de `pata_MOD_DRV` se conservan. Las pruebas 1 a 4 ponen el bus en dominante unos microsegundos, por eso hay que correrlas con **CANH y CANL desconectados**.

**Compilar** (desde `diag_placa`, después de cargar el perfil de ESP-IDF):

```
idf.py -B build_esp32 -D SDKCONFIG=build_esp32/sdkconfig -D IDF_TARGET=esp32 build
idf.py -B build_s3 -D SDKCONFIG=build_s3/sdkconfig -D IDF_TARGET=esp32s3 build
```

**Cargar** (desde la carpeta `build_esp32` o `build_s3`; el puerto es el de esa placa):

```
esptool.py --chip esp32 -p COM6 -b 460800 --before default_reset --after hard_reset write_flash '@flash_args'
```

**Monitor:** 115200 baudios en las dos placas; en la maestra, por el conector «UART». La prueba se corre sola al arrancar; `todo` la repite.

**Dashboard:** `diag_placa/diag_dashboard.html`, que se abre en Chrome o Edge. Tiene botones para cada prueba y tarjetas con PASA/FALLA. Muestra el diagnóstico, la escucha del bus, el monitor con gráfica de CRX y de los encoders, y la consola. Lleva además un registro de las placas probadas, que se guarda en el navegador y se puede descargar como CSV. La placa se reconoce por su MAC.

**Volver al firmware normal.** En la pata, desde `../pata_MOD_DRV/build`, se usa el mismo `esptool.py` con `--chip esp32`. En la maestra, desde `maestra_s3/build_cli`, con `--chip esp32s3`.
