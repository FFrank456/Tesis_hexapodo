---
tipo: reporte-tecnico
estado: en-curso
fecha: 2026-11-28
tags:
  - can
  - comunicacion
  - hardware
  - unidad-2
  - fallas
---

# Comunicación CAN de la Unidad 2: funcionamiento, fallas de hardware y medidas adoptadas

Reporte del bus CAN entre la maestra ESP32-S3 y las seis placas MOD_DRV (una por pata), del 7 al 28 de noviembre de 2026. Reúne:
- las pruebas registradas en el repositorio;
- los datos que guardan los dashboards;
- el historial de las sesiones de trabajo.

**Tesis que sostiene el reporte:**
- **El protocolo y el firmware CAN funcionan.** Con las seis patas en el bus se intercambiaron **71 928 de 71 928 respuestas, sin un solo error de bus** (26-nov).
- **El bus completo es tan confiable como su nodo más débil.** Varias veces, **una sola placa con un defecto de soldadura o con el transceptor dañado bloqueó el bus entero**. En esos casos ninguna pata podía comunicarse, y eso impidió avanzar con la locomoción.
- Las causas son de **ensamblaje del hardware**, no del diseño lógico del protocolo:
  - soldaduras abiertas o intermitentes;
  - pads sin contacto;
  - un módulo transceptor sin alimentación;
  - contactos flojos en el cableado.

> [!info] Cómo leer el reporte
> Cada dato indica su origen. **Medido** sale de una prueba con firmware o multímetro y quedó registrado. **Inferido** es una causa probable que no se confirmó. **Informado** es lo que reportó el autor y no se midió en estas pruebas.

---

## 1. Arquitectura del bus

| Elemento | Valor |
|---|---|
| Nodos | 1 maestra ESP32-S3 DevKitC-1 + 6 MOD_DRV (ESP32-WROOM-32E) |
| Controlador | TWAI integrado del ESP32 (CAN 2.0A, ID de 11 bits) |
| Transceptor en las patas | Módulo WCMCU-230 (**SN65HVD230**, 3,3 V) soldado en el zócalo J4 de la MOD_DRV |
| Transceptor en la maestra | Módulo SN65HVD230 y, desde el 26-nov, CJMCU-2551 (MCP2551, 5 V) con conversor de nivel en RXD |
| Velocidad | **500 kbit/s**: bit de 2000 ns, muestreo al 80 % (1600 ns) |
| Topología | Línea (*daisy chain*): la maestra en un extremo y las patas encadenadas por P1/P2 (CANH, CANL y GND), con **120 Ω solo en los dos extremos** (≈ 60 Ω medidos entre CANH y CANL) |
| Pines de la MOD_DRV | CTX = IO21 (J4-3), CRX = IO22 (J4-4). CANH y CANL salen del transceptor por J4-5/6 hacia P1/P2 |
| Cableado | Menos de 2 m en total, cables cortos de banco |

**Protocolo v2** (`03-control/pata_MOD_DRV/main/can_protocol.h`), compartido por la maestra y las patas:

| ID | Sentido | Uso |
|---|---|---|
| `0x0FF` | Maestra → todas | PARO general. Es el ID más bajo, así que gana siempre el arbitraje |
| `0x100 + n` | Maestra → pata *n* | PING, ARMAR, IR_A, ESTADO, CERO, DESARMAR, IDENT y SEGUIR (27-nov) |
| `0x200 + n` | Pata *n* → maestra | PONG, ACEPTADA, RECHAZADA, ESTADO, DETENIDA e IDENT (MAC) |

**Seguridad del protocolo:**
- **Latido:** una pata armada que pasa **0,5 s sin recibir órdenes** suelta sus motores. La maestra pide ESTADO cada 100 ms.
- **ID de pata en NVS** (22-nov).
- **IDENT con la MAC** para detectar IDs duplicados.
- **Recuperación automática tras un BUS_OFF**, sin volver a armar nunca solo (25-nov).

**Capacidad:**
- Una trama de 8 bytes ocupa ≈ 0,26 ms del bus.
- Una ronda a las seis patas tarda ≈ 3 ms.
- Con un ciclo de 50 Hz el bus queda ocupado ≈ 16 %.

**La velocidad del bus sobra para la locomoción.** El control rápido (corriente a 1 kHz, velocidad a 100 Hz y posición a 50 Hz) corre dentro de cada placa. Por el CAN solo viajan consignas y estados.

---

## 2. Por qué un solo nodo defectuoso bloquea todo el bus

La mayor dificultad del proyecto viene de **cómo funciona la capa física de CAN**, no del software.

1. **El bus es compartido y de tipo AND cableado.** Un bit **dominante (0)** de cualquier nodo se impone sobre el **recesivo (1)** de todos los demás. Eso permite el arbitraje sin colisiones, pero tiene una consecuencia: **si un nodo deja el bus en dominante de forma permanente, todos los nodos ven un 0 continuo.**
2. **Con el bus en dominante nadie puede transmitir.**
   - Cada intento de los demás nodos termina en error de bit o de forma.
   - Sus contadores de error (TEC y REC) suben, en el caso del TEC de 8 en 8.
   - Al pasar de 255 el controlador entra en **BUS_OFF** y deja de participar.
   - La maestra lo vio como `NO_SALIO`, CRX fijo en 0 y BUS_OFF.
3. **Un transceptor puede forzar el dominante por varias causas, todas de hardware:**
   - **CTX (IO21) en 0 V:** un puente de estaño a GND, una pista dañada o un transceptor con fuga en su entrada D. El transceptor transmite dominante continuo.
   - **El transceptor sin alimentación (VCC abierto)** mientras el ESP32 está encendido. El chip se alimenta a través del diodo de protección de su entrada D (*back-powering*), arrastra CTX a 0–0,7 V, pone el bus en dominante y **termina dañado**. Esto se midió en la placa 6 (sección 4).
   - **CANL en corto con GND**, o CANH y CANL cruzados en un conector. En P1/P2, CANL (pin 2) y GND (pin 3) son pines vecinos.
4. **Un nodo con CRX abierto** (la señal del transceptor no llega a IO22) no bloquea el bus por sí solo. Pero el controlador lee un nivel indefinido, acumula errores y el nodo no funciona. Al tocar la placa, la soldadura intermitente cambiaba el estado.
5. **El latido convierte cada corte en una parada segura.** Si el bus se bloquea más de 0,5 s, las patas armadas sueltan sus motores. **Es el comportamiento correcto por seguridad**, pero significa que **cualquier falla de un nodo detiene la marcha del robot entero**.

**Consecuencia para la tesis:** la locomoción coordinada exige que **las seis patas y la maestra** estén en el bus a la vez y sin errores. Cada nodo es, por lo tanto, un **punto único de falla** del sistema de comunicación. Con siete nodos, la confiabilidad del bus es el producto de la de cada nodo. Por eso los defectos de ensamblaje, aunque afecten a una sola placa, impidieron las pruebas de marcha.

---

## 3. Cronología

| Fecha | Qué se hizo | Resultado | Origen |
|---|---|---|---|
| 07-nov | Primeras pruebas CAN en las MOD_DRV ensambladas, con WCMCU-230 a 500 kbit/s | Una placa pasa el autotest; otra pasa **de forma intermitente**; una tercera tiene **CRX en bajo en reposo**. No hubo comunicación entre dos placas | Medido |
| 08-nov | Drivers, fase 1 (solo USB) | Pull-ups de nFAULT y resistencias de IPROPI presentes, sin cortos en IN1/IN2 | Medido |
| 20-nov | Maestra S3 ↔ MOD_DRV (`prueba_can`) | La placa COM17 tenía CRX en 0 con 3,28 V en el pin del módulo: **la unión J4-4 → IO22 (pin 36 del WROOM) estaba abierta**. Al resoldarla pasó 10/10, pero **se volvió a abrir al mover la placa**. Con otra placa (COM6): **1000/1000 pings**, ida y vuelta de 263 µs de media | Medido |
| 20-nov | Fémur y tibia movidos por CAN | 265/265 consignas aceptadas. **La coxa quedó fuera por un nFAULT de su driver (DRVA)** que solo aparece con V_MOT | Medido |
| 21-nov | Protocolo v2 (ángulos absolutos) | 7/7 órdenes llegaron, con error ≤ 0,019 rad | Medido |
| 22-nov | ID de pata en NVS y maestra para 6 patas | Implementado | Repositorio |
| 23-nov | Cinco placas en el bus | **5041 tramas, 0 fallos.** La sexta placa (COM17) **forzaba el bus a dominante** por sí sola (CANH−CANL = 2,1 V, 0/6 placas responden). Las placas 1 y 2 empezaron a funcionar **después de limpiarlas con alcohol, *flux* y estaño** | Medido |
| 24-nov | Cinco y luego seis placas, con los ID reasignados | Antes, el bus se bloqueaba a ratos (CRX en 0, BUS_OFF). Se encontró **flojo el GND entre la maestra y la primera placa**. Después: **18 000/18 000 pings con las seis**, 0 errores. La pata 5 reportó un **nFAULT en el fémur** (DRVB) que luego desapareció sin causa registrada | Medido |
| 25-nov | Pruebas de aislamiento | **La placa 1 bloqueaba el bus**: bastaba conectar **solo su CANL** para dejarlo en dominante. Aislada, su propio módulo veía dominante (REC 128, ~30 000 errores/s, TEC 0). Sin ella, el bus volvía a funcionar (4000/4000). Se creó la herramienta `diag_placa` | Medido |
| 26-nov | Maestra con MCP2551 y bus completo | **Seis patas: 71 928/71 928 respuestas, 0 errores, 0 BUS_OFF**, latencia máxima de 1,68 ms. Antes, la maestra había dado `MODULO_DOMINANTE` y `MODULO_NO_TRANSMITE` por la falta de 5 V y de los 120 Ω en el CJMCU-2551 | Medido (CSV) |
| 26/27-nov | **La placa 6 vuelve a bloquear el bus** | El módulo transceptor quedó **sin VCC** por una soldadura abierta. Al resoldarlo, el chip ya estaba dañado: con CTX en 1 medía CANH 2,7 V y CANL 0,3 V, **dominante en reposo**. Se cambió el chip | Medido |
| 27-nov | Placa 6 sin transceptor y con un puente CTX–CRX | **IO21, IO22 y el TWAI sanos**: 34 400/34 400 tramas y TWAI 200/200. La falla estaba solo en el módulo | Medido |
| 27-nov | Placa 6 con el módulo nuevo | Pasa ctx, eco y reposo. Sin 120 Ω el TWAI falla (subida de 2,99 µs), como se esperaba para un nodo intermedio | Medido |
| 27-nov | Maestra S3 con el transceptor y 120 Ω | TWAI 200/200, aguante de 20 000/20 000 tramas, subida de 0,9 µs | Medido |
| 27-nov | **Maestra ↔ placa 6 reparada** (firmware `pata_MOD_DRV`) | Escaneo 20/20, **2000/2000 pings** (354 µs de media), **985/985 ESTADO a 100 Hz**, 0 errores | Medido |

---

## 4. Catálogo de fallas

| # | Nodo | Síntoma en el bus | Medida clave | Causa | Tipo | Estado |
|---|---|---|---|---|---|---|
| F1 | MOD_DRV COM17 (ID 2) | CRX en 0, autotest 0/10 | 3,28 V en el pin CRX del módulo, pero no en IO22 | **Unión abierta J4-4 → IO22** (pin 36 del WROOM). Se resolvió y **se volvió a abrir al mover la placa** | Soldadura intermitente | Funcionó el 24-nov; no quedó registrado qué se le hizo |
| F2 | MOD_DRV COM17 (23-nov) | **Bus entero en dominante**, 0/6 placas responden | CANH−CANL = 2,1 V con la placa sola | Módulo transceptor o su soldadura (inferido) | Transceptor o soldadura | Pasó el 24-nov |
| F3 | Placas 1 y 2 (23-nov) | No respondían | — | Contactos sucios o soldaduras frías: funcionaron después de limpiar con alcohol, *flux* y estaño | Soldadura | Resuelta |
| F4 | Cableado maestra ↔ placa 1 (24-nov) | Bloqueos intermitentes, BUS_OFF con distintas combinaciones | — | **GND común flojo** | Conector o cable | Resuelta |
| F5 | MOD_DRV ID 1 (25-nov) | **Bus en dominante** al conectar solo su CANL | REC 128, ~30 000 errores/s, TEC 0 | Candidatas sin confirmar: un puente CANL–GND en P1/P2 (pines vecinos), un cable cruzado o el transceptor | Soldadura, conector o transceptor | Funcionó el 26-nov; causa no registrada |
| F6 | MOD_DRV ID 6, COM7 (26/27-nov) | **Bus en dominante** | VCC del módulo abierto. Después: D 1,5 V, IO21 0,78 V, CANH 2,7 V, CANL 0,3 V (diferencia de 2,4 V en reposo) | **El transceptor sin VCC se alimentó por su entrada D y se dañó** | Soldadura → daño del transceptor | **Resuelta** (chip cambiado, 2000/2000 pings) |
| F7 | MOD_DRV ID 6 con el chip de repuesto | CTX en 0 aunque el ESP32 lo manejaba en 1 | Sin el módulo, IO21 sube y baja bien | Defecto del chip de repuesto o de su montaje | Transceptor o soldadura | Resuelta con otro módulo |
| F8 | Maestra con CJMCU-2551 (26-nov) | `MODULO_NO_TRANSMITE`, `MODULO_DOMINANTE` | Recesivo lento: 6,9 µs | Sin 5 V externos y **sin la 120 Ω** (R2 y C2 vacías en ese módulo) | Configuración del módulo | Resuelta |
| F9 | Placas intermedias en prueba sola | TWAI 0/200, aunque ctx, eco y reposo pasan | Subida de 3,0–3,3 µs | **Falta la terminación**: sin 120 Ω el regreso a recesivo es lento. No es un defecto; es lo esperado para un nodo intermedio | Método de prueba | Explicado |
| F10 | MOD_DRV COM6, coxa (DRVA) | No afecta el bus, pero **la coxa no se puede mover** | nFAULT en bajo **solo con V_MOT**, con el motor parado y desconectado | Inferido: la bomba de carga (C3A de 0,1 µF / C4A de 22 nF, ambos **0402**) o VM y el **pad térmico** del DRVA mal soldados | Soldadura de un driver | **Pendiente** de medir VCP |
| F11 | Pata 5, fémur (DRVB) | nFAULT con la pata desarmada (24-nov) | — | No registrada; desapareció al repetir | Driver o conexión | Vigilar |

**Patrón:** de las once fallas, **ninguna se debió al protocolo ni al firmware de comunicación**.
- Ocho son de **ensamblaje**: soldadura, pads, alimentación del módulo y conectores (F1–F7 y F10).
- Una es de **configuración de un módulo comercial** (F8).
- Una es del **método de prueba** (F9).
- Una no tiene causa registrada (F11).

---

## 5. Hallazgos de la revisión por un ingeniero electrónico (informado)

El autor llevó **una de las placas MOD_DRV** a un ingeniero electrónico para revisarla. Según su informe, el ingeniero encontró:

- **pistas con soldaduras deficientes**;
- **pads sin contacto eléctrico interno con algunos de los drivers** DRV8876: la soldadura se veía en la superficie, pero no había unión real con el pad del componente.

**Relación con lo medido:**
- El DRV8876PWPR viene en **HTSSOP-16 con pad térmico expuesto (PowerPAD) debajo del chip**. Ese pad es GND y disipa el calor. **No se ve ni se alcanza con el cautín**, así que una soldadura deficiente ahí solo se detecta con medidas eléctricas o con rayos X.
- Un pad del driver sin unión real explica el **nFAULT de la coxa que aparece solo con V_MOT** (F10). El driver detecta subtensión o una falla de su bomba de carga.
- Pistas y uniones mal soldadas explican las **fallas intermitentes** de la comunicación: la unión de IO22 que se abría al mover la placa (F1), y las placas que funcionaron solo después de repasar las soldaduras (F3).

> [!todo] Completar este registro
> - Fecha de la revisión y nombre o cargo del ingeniero (si se puede citar).
> - **Qué placa** se revisó: MAC o ID de pata.
> - **Qué drivers** (DRVA, DRVB o DRVC) y qué pads o pistas tenían el defecto.
> - Fotos de la inspección en `05-documentacion/adjuntos/` y enlace aquí.
> - Qué se reparó y si se volvió a probar.

---

## 6. Complejidad del ensamblaje: tamaño de los componentes

La MOD_DRV tiene 88 componentes en una placa compacta, con encapsulados muy pequeños. La tabla sale del archivo *Pick Place* de Altium (`Pick Place for MOD_DRV.csv`).

| Componente | Encapsulado | Tamaño o paso | Por qué es difícil a mano | Falla relacionada |
|---|---|---|---|---|
| DRV8876PWPR (DRVA–DRVC) | **HTSSOP-16 con PowerPAD** | Paso de **0,65 mm**, pad térmico inaccesible debajo del chip | Puentes entre pines vecinos. El pad térmico necesita aire caliente o horno; con cautín no se ve si soldó | F10 y el hallazgo del ingeniero |
| C3A–C3C (0,1 µF, bomba de carga), C4A–C4C (22 nF), C23, C25, C30, R14 | **0402** | **1,0 × 0,5 mm** | Se levantan (*tombstoning*), se pierden o quedan con un solo extremo soldado; casi no se distinguen a simple vista | F10 (bomba de carga del DRVA) |
| CP2102N (U2) | **QFN-28** con pad expuesto | Paso de **0,5 mm**, sin patas visibles | Las uniones quedan debajo del chip | — (el USB funcionó en el *bring-up*) |
| ESP32-WROOM-32E (MD1) | Módulo con pads almenados | Paso de 1,27 mm, 38 pads | Uniones pequeñas en el borde. Un pad mal soldado se abre con el esfuerzo mecánico | **F1: pin 36 (IO22) abierto** |
| Zócalo J4 + módulo WCMCU-230 | Tira Samtec de 2,54 mm + módulo con un SOIC-8 | — | El módulo queda soldado sobre el zócalo. Un pin sin estaño (VCC, CRX) **no se ve** y deja el transceptor sin alimentación o sin señal | **F6 (VCC abierto → transceptor dañado)**, F1 |
| AP63205 (U1), TLV75733 (U3) | TSOT-23-6, SOT-23-5 | Paso de 0,95 mm | Moderada | — |
| Resto de pasivos | 0603, 0805, 1206 | — | Manejables a mano | — |

**Conclusión:** el diseño (DRC del 18-oct con 0 violaciones) es correcto. Pero **soldar a mano encapsulados de 0,5–0,65 mm de paso, pads térmicos ocultos y pasivos 0402** deja uniones defectuosas que **no se pueden inspeccionar visualmente**. Esas uniones aparecen como **fallas intermitentes**: funcionan en una prueba y fallan al mover la placa o al calentarse. Es la explicación más consistente de por qué el bus completo funcionó (26-nov: 71 928/71 928) y al día siguiente volvió a bloquearse por un solo nodo (placa 6).

---

## 7. Impacto en la locomoción

- La marcha (trípode, ripple u ondulada) mueve **las seis patas coordinadas**. La maestra les manda consignas a 20–50 Hz y vigila el latido de cada una.
- Si **un nodo** bloquea el bus:
  1. la maestra entra en BUS_OFF;
  2. en 0,5 s **todas las patas armadas sueltan sus motores** (latido);
  3. el robot no puede seguir caminando y, si estaba apoyado, **cae**.
- Por eso no basta con que cinco placas funcionen: **una sola placa con una soldadura intermitente impide la prueba de marcha completa**. Cada vez que fallaba un nodo hubo que:
  1. aislarlo;
  2. diagnosticarlo con `diag_placa`;
  3. repararlo (resoldar o cambiar el transceptor);
  4. volver a probar el bus.

  Ese ciclo consumió los días que se habían planificado para la locomoción.
- La fase 1 del robot (`robot_dashboard.html`: cinemática inversa y marcha **simulada** en la maestra) quedó lista el 25-nov. La marcha **real** (fase 2) depende de tener el bus estable con las seis patas.

---

## 8. Medidas adoptadas

### 8.1 Herramientas de diagnóstico (firmware y dashboards)

| Herramienta | Qué resuelve |
|---|---|
| `diag_placa` (25-nov), para ESP32 y ESP32-S3 | Prueba **una placa sola**, sin otro nodo: `ctx` (CTX libre), `eco` (CTX→CRX por GPIO), `reposo` (CRX no baja), `twai` (200 tramas en NO_ACK) y `motores` (nFAULT y encoders). Dice la causa probable y qué medir |
| `aguante N` | Repite las pruebas N veces para encontrar **fallas intermitentes** (soldaduras) |
| `retardo` | Mide el lazo CTX → transceptor → CRX en ns. Distingue un transceptor dañado de la **falta de terminación** |
| `ctx1`, `ctx0` y `ctxlibre` (27-nov) | Dejan CTX fijo para **medir el transceptor con el multímetro** pata por pata |
| `escanear`, `rafaga` y `salud` en la maestra | Qué patas responden (con su MAC), pérdidas, latencia, TEC/REC, BUS_OFF y carga del bus |
| `diag_dashboard.html` | Interfaz con PASA/FALLA por prueba y **registro de placas por MAC**, exportable a CSV (`robustez_can.csv`, `diagnostico_can_placas.csv`) |
| **Prueba con puente CTX–CRX** (27-nov) | Sin el transceptor, separa una falla del ESP32 o de las pistas de una falla del módulo |

### 8.2 Firmware y protocolo

- **ID de pata en NVS** y **detección de IDs duplicados** por MAC (IDENT): no se arma ni mueve una pata con el ID repetido.
- **Latido de 0,5 s:** una pata sin comunicación suelta sus motores.
- **PARO con la máxima prioridad** (`0x0FF`), que sale aunque la maestra esté en ESCUCHA.
- **Recuperación tras BUS_OFF:** la maestra escucha sin transmitir y vuelve sola a NORMAL si el bus se libera, sin volver a armar.
- **Orden SEGUIR** (27-nov) para la marcha continua sin reiniciar los lazos de control.

### 8.3 Procedimientos

- Medir **≈ 60 Ω entre CANH y CANL** con todo apagado. Solo llevan 120 Ω la maestra y la última pata.
- **GND común firme** entre la maestra, las placas y la fuente.
- **Revisar el VCC del módulo transceptor en cada placa antes de encenderla**: un transceptor sin VCC con el ESP32 encendido se daña y bloquea el bus (lección de F6).
- Probar cada placa sola (`todo` y `aguante`) **antes** de conectarla al bus. Si el bus falla, agregar las placas de una en una.

---

## 9. Recomendaciones para una revisión de la placa (Rev. B)

1. **Transceptor soldado en la placa**, en lugar de un módulo sobre un zócalo. Elimina las uniones de J4, que no se pueden inspeccionar (F1 y F6).
2. **Transceptor con protección contra dominante permanente** (*TXD dominant time-out*), por ejemplo de la familia TCAN104x. Si CTX queda en 0, el transceptor suelta el bus solo y el nodo defectuoso no bloquea a los demás.
3. **Protección ESD en CANH y CANL** y **terminación de 120 Ω seleccionable con un puente** en cada placa.
4. **Puntos de prueba** en CTX, CRX, VCC del transceptor, VCP y VM de cada driver: se mide sin tocar las patas de los chips.
5. Pasivos de **0603 en lugar de 0402** donde el espacio lo permita.
6. **Ensamblaje con esténcil y horno** (o encargarlo al fabricante) para los HTSSOP con PowerPAD y el QFN, con inspección del pad térmico.
7. **Conectores con traba** para el bus (en lugar de tiras de pines rectas) y un GND del bus en cada conector.

---

## 10. Estado al 28-nov-2026

| Elemento | Estado |
|---|---|
| Protocolo, firmware de la maestra y de las patas | Funcionan y están validados con las seis patas (71 928/71 928 respuestas, 26-nov) |
| Placa 6 (ID 6) | Reparada (transceptor cambiado). 2000/2000 pings con la maestra (27-nov). Le faltan sus ganancias y su pose |
| Maestra S3 | Pasa sola: 20 000/20 000 tramas con 120 Ω |
| Bus con las seis patas después de las reparaciones | **Pendiente** de volver a validar (`escanear 20` y `salud 60 100`) |
| nFAULT de la coxa (COM6) | Pendiente de medir VM y VCP en el DRVA |
| Marcha real (fase 2) | Bloqueada hasta tener el bus estable con las seis patas |

---

## 11. Fuentes

- Registro de pruebas y protocolo: `03-control/prueba_can/README.md`
- Pruebas por fecha: [[prueba-modulos-can-esp32-wcmcu230]] (`02-electronica/notas/`)
- Diseño de la placa y lista de componentes: [[Modulo de control de extremidad - Registro del proyecto]]
- Datos de los dashboards:
  - `03-control/prueba_can/robustez_can.csv`
  - `03-control/pata_MOD_DRV/dashboard/diagnostico_can_placas.csv`
- Herramienta de diagnóstico: `03-control/prueba_can/diag_placa/`
- Lista de montaje: `02-electronica/unidad2-MOD_DRV/.../Pick Place for MOD_DRV.csv`
- Commits de la comunicación CAN:
  - `a3d6e82` (20-nov), `52210bd` y `e2bffc5` (21-nov), `6644ac9` (22-nov);
  - `bad1144`, `30bcced` y `5bbd7c8` (23–24-nov);
  - `4a2620b`, `e4e2f46` y `6fa9807` (25-nov), `2113686` (27-nov).
- Latencia con el firmware v1 (antes de bloquear en `twai_receive`): `05-documentacion/informe-unidad2/figuras/fig_can_latencia.png`. Con el firmware actual, la ida y vuelta es de 0,33–0,40 ms de media.
