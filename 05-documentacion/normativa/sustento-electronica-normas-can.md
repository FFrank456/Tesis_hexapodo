---
tipo: marco-normativo
fecha: 2026-11-26
estado: borrador para sustentación
tags:
  - normativa
  - iso
  - ipc
  - pcb
  - can
---

# Sustento de la parte electrónica: normas, PCB y bus CAN

Notas para defender ante el jurado las decisiones de electrónica y comunicación de la tesis "Dos hexápodos bioinspirados cooperativos para mapeo y detección de gases" (UTEC). Resume lo conversado el 2026-11-26.

> [!info] Contexto en una línea
> La **Unidad 1** (líder: 18 MG996R, Mini Maestro 18, Raspberry Pi 5, Orbbec Gemini 2) mapea con RTAB-Map + Nav2. La **Unidad 2** (modular: 18 micromotores N20, 6 placas MOD_DRV con ESP32 + 3× DRV8876) coordina sus patas por **bus CAN** desde una maestra ESP32-S3. Las dos unidades se comunican por micro-ROS sobre Wi-Fi.

---

## 1. Dos familias de normas: ISO e IPC

| | **ISO** | **IPC** |
|---|---|---|
| Qué es | Organización Internacional de Normalización, formada por los organismos nacionales de normas | Asociación de la industria electrónica (antes *Institute for Printed Circuits*, hoy *Association Connecting Electronics Industries*) |
| Alcance | Normas de todo tipo de industrias | Solo electrónica: diseño, fabricación y ensamblado de placas |
| Qué respalda en la tesis | La **arquitectura** del sistema: modularidad, vocabulario, protocolo CAN | Las **decisiones físicas** de la placa: ancho de pistas, separaciones, huellas, calidad de fabricación y soldadura |

Regla práctica: si la pregunta es "¿por qué el sistema está organizado así?", se responde con ISO. Si es "¿por qué esta pista mide eso?", con IPC.

---

## 2. Normas ISO aplicables

### ISO 22166-1:2021 — Modularidad en robots de servicio (parte 1: requisitos generales)
- Define el **módulo** como una pieza con interfaces y un perfil de propiedades bien definidos, que se puede reemplazar o reutilizar, y lo distingue del simple componente.
- Pide que los módulos sean componibles, integrables, interoperables y reutilizables, con seguridad y protección, e incluye una plantilla para describirlos.
- **Uso en la tesis:** respalda las 6 placas MOD_DRV idénticas de la Unidad 2. Cada una es un módulo con dos interfaces definidas: potencia (12 V por J1) y datos (CAN por P1/P2). Si una pata falla, se cambia la placa sin rediseñar el resto.

### ISO 8373:2021 — Vocabulario de robótica
- Sirve para definir y clasificar los hexápodos (robot de servicio, robot móvil, robot con patas).

### ISO 11898 — Controller Area Network (CAN)
| Parte | Qué cubre | Uso en la tesis |
|---|---|---|
| **ISO 11898-1:2024** | Capa de enlace: tramas, arbitraje, detección y confinamiento de errores | Formato de mensaje y prioridades del bus de la U2 |
| **ISO 11898-2:2024** | Capa física de alta velocidad: niveles de voltaje, transceptor, terminación | Transceptor SN65HVD230, resistencias de 120 Ω, topología en cadena |
| **ISO 11898-3** | CAN tolerante a fallas (baja velocidad, hasta 125 kbit/s) | Solo como comparación: la U2 **no** lo usa |
| **ISO 15765-2** | Transporte de mensajes de más de 8 bytes sobre CAN | Solo si algún dato no cabe en una trama |

Origen histórico: especificación **CAN 2.0 de Bosch** (1991).

### Otras normas ya identificadas
IEC 60079 (atmósferas explosivas, declarar como limitación), IEC 60529 (grado IP), ASTM E54.09 (pruebas de robots de respuesta) y DS 024-2016-EM (umbrales de gases en minería). Detalle en [[normas-y-reglamentos]] y [[gases-a-monitorear]].

---

## 3. Normas IPC para las placas PCB

| Norma | Fase | Qué cubre |
|---|---|---|
| **IPC-2221B** (2012) | Diseño | Norma general: ancho de pista según corriente, separación entre conductores según voltaje, vías, márgenes |
| **IPC-2152** (2009) | Diseño | Capacidad de corriente de pistas y planos. Reemplaza las curvas de la IPC-2221, basadas en datos de hace más de 50 años, y considera espesor de cobre, planos cercanos, vías y material |
| **IPC-7351** | Diseño | Huellas (footprints) de los componentes |
| **IPC-6012** | Fabricación | Requisitos de calidad de placas rígidas |
| **IPC-A-600** | Fabricación | Criterios de aceptación visual de la placa desnuda |
| **IPC-A-610** | Ensamblado | Criterios de aceptación de la placa ya soldada |
| **J-STD-001** | Ensamblado | Requisitos del proceso de soldadura |

### Cómo usarlas para sustentar los buses de la PCB
1. **Ancho de pista y planos de potencia (IPC-2152, con IPC-2221B como referencia):** el ancho se elige por la corriente y el aumento de temperatura aceptable.
   - *Ejemplo de la PCBicho v1.3:* la parte más angosta de la zona VBUS mide ~8,7 mm. Con +10 °C de aumento, aguanta ~19 A con cobre de 2 oz y ~11,5 A con 1 oz (cálculo IPC-2221). Por eso se recomienda 2 oz para el objetivo de 10–15 A.
   - Conviene **recalcular con IPC-2152**: la IPC-2221 sobredimensiona cuando hay planos de cobre cerca, así que el resultado queda del lado seguro.
2. **Separación entre conductores (IPC-2221B):** con 12–16 V, la separación mínima es pequeña. Basta con citar la tabla de separaciones y mostrar que el DRC de KiCad la cumple.
3. **Huellas (IPC-7351):** justifica las huellas de librería y la del módulo MP1584, verificada imprimiendo a 1:1.
4. **Fabricación y soldadura:** JLCPCB y PCBWay fabrican por defecto a IPC-6012 / IPC-A-600 clase 2. La soldadura THT y SMD se inspecciona con los criterios de IPC-A-610.
5. **Par CAN en la placa:** pistas CANH y CANL paralelas, juntas y de igual largo (par diferencial), con un plano de tierra continuo debajo. No hace falta una impedancia controlada exacta con trazos tan cortos, pero sí mantener el par unido.

---

## 4. Bus CAN

### Definición para la tesis
**Versión corta:**
> El bus CAN, definido en la ISO 11898, es un protocolo serial multimaestro con señalización diferencial sobre un par trenzado y arbitraje por prioridad de mensaje. Eso le da inmunidad al ruido y una comunicación determinista, adecuada para el control distribuido de las patas de la Unidad 2.

**Versión completa:**
> El bus CAN (Controller Area Network) es un protocolo de comunicación serial multimaestro, estandarizado en la ISO 11898, en el que todos los nodos comparten un par trenzado y transmiten mediante señalización diferencial entre las líneas CANH y CANL, lo que le da alta inmunidad al ruido electromagnético. El acceso al medio se resuelve por arbitraje bit a bit según el identificador de cada mensaje, de modo que el mensaje de mayor prioridad se transmite sin pérdida ni retransmisión, lo que vuelve la comunicación determinista. Además, incorpora detección y confinamiento de errores por hardware, lo que lo hace adecuado para sistemas de control distribuido en tiempo real, como la coordinación de las seis patas de la Unidad 2.

### Cómo funciona
- **Dos cables comunes a todos los nodos:** CANH y CANL. El receptor no lee cada cable por separado, sino la **diferencia** de voltaje entre ellos. El ruido de los motores afecta a los dos cables por igual y se cancela.
- **Bit 1 = recesivo:** el transceptor no empuja; los dos cables quedan a ~2,5 V y la diferencia es ~0 V.
- **Bit 0 = dominante:** el transceptor empuja; CANH sube a ~3,5 V y CANL baja a ~1,5 V; la diferencia es ~2 V.
- **Arbitraje:** si dos nodos transmiten a la vez, el 0 se impone sobre el 1. El nodo que mandó un 1 y lee un 0 sabe que perdió y se calla, sin dañar el mensaje del otro. Gana el **identificador más bajo**, que es el de mayor prioridad.
- **Terminación:** una resistencia de **120 Ω en cada extremo** de la cadena (60 Ω vistos entre CANH y CANL) para que la señal no rebote.
- **Errores:** cada nodo cuenta sus errores. Si acumula demasiados, pasa a *error pasivo* y luego a *bus-off*: se desconecta solo para no bloquear a los demás.

### Qué pasa si falla un cable
| Falla | Efecto |
|---|---|
| CANL a tierra (o CANH a alimentación) | La diferencia queda fija en un valor "dominante". Todos ven un 0 permanente, nadie gana el arbitraje y los nodos terminan en *bus-off*. Como el cable es común, cae todo el bus |
| Cable cortado | Se pierde una terminación, la señal rebota y los nodos al otro lado del corte quedan aislados |

El CAN de alta velocidad (ISO 11898-2), que usa el SN65HVD230, **no tolera** la falla de un solo cable. El CAN tolerante a fallas (ISO 11898-3) sí, pero llega solo a 125 kbit/s. **Mitigación en la U2:** conectores con traba, mazo fijado al chasis y el respaldo propuesto por ESP-NOW para que cada pata siga operando si pierde el bus.

### Tamaño de mensaje y velocidad
| Versión | Datos por trama | ¿Lo soporta el ESP32? |
|---|---|---|
| CAN clásico (CAN 2.0) | hasta **8 bytes** (64 bits) | **Sí** (controlador TWAI) |
| CAN FD | hasta 64 bytes, fase de datos más rápida | No |
| CAN XL (ISO 11898-1:2024) | hasta 2048 bytes | No |

- Identificador de **11 bits** (estándar) o **29 bits** (extendido). Una trama estándar completa ocupa ~110–130 bits con CRC, acuse y relleno de bits.
- La velocidad depende del largo del bus: a **1 Mbit/s** llega a unos **40 m**. Dentro del robot, con cables cortos, 1 Mbit/s es viable: del orden de **7000 a 8000 tramas por segundo** en todo el bus.
- Si algo no cabe en 8 bytes, se parte en varias tramas (ISO 15765-2 o numeración propia).
- **Para la U2, 8 bytes alcanzan:** tres consignas de posición de 16 bits por pata ocupan 6 bytes.

### Implementación en la Unidad 2
| Elemento | Detalle |
|---|---|
| Controlador | TWAI interno del ESP32-WROOM-32E (cada MOD_DRV) y del ESP32-S3 (maestra) |
| Pines en la MOD_DRV | IO21 = CTX, IO22 = CRX |
| Transceptor | SN65HVD230 (módulo WCMCU-230 en J4), 3,3 V |
| Conexión | En cadena por P1/P2 (CANH, CANL, GND) |
| Topología | LR → LM → LF → maestra → RF → RM → RR, con 120 Ω solo en LR y RR |
| Lazos | Cada ESP32 cierra corriente, velocidad y posición de sus 3 motores; por el CAN solo viajan consignas y telemetría |

**Por qué CAN y no Wi-Fi entre patas:** es determinista, resiste el ruido de los motores por ser diferencial, no depende del alcance de la radio dentro del chasis o de una galería, y libera al ESP32 de la pila Wi-Fi para el lazo de control.

---

## 5. Preguntas probables del jurado

| Pregunta | Respuesta corta |
|---|---|
| ¿Por qué un ESP32 por pata? | Una pata necesita 20–22 señales y el ESP32 tiene 26 GPIO; dos patas no caben. Además, es modular según la ISO 22166-1 |
| ¿Por qué CAN y no I²C, SPI o UART? | I²C y SPI son para distancias de centímetros y sin señal diferencial; UART es punto a punto. CAN es multinodo, diferencial, con prioridades y detección de errores |
| ¿Qué pasa si se corta el bus? | Se aíslan las patas al otro lado del corte; cada pata conserva su lazo local y se propone ESP-NOW como respaldo |
| ¿Cómo dimensionaste las pistas de potencia? | Con la corriente máxima y +10 °C de aumento según IPC-2221 / IPC-2152, con cobre de 2 oz |
| ¿Por qué 120 Ω? | Es la impedancia característica del par trenzado definida en la ISO 11898-2; terminar en los dos extremos evita reflexiones |
| ¿Cabe la información en 8 bytes? | Sí: 3 consignas de 16 bits = 6 bytes por pata |

---

## 6. Pendientes
- [ ] Recalcular el ancho de la zona VBUS de la PCBicho con IPC-2152.
- [ ] Confirmar que la cadena CAN lleva 120 Ω solo en LR y RR, y medir 60 Ω entre CANH y CANL con el bus apagado.
- [ ] Definir la velocidad del bus (500 kbit/s o 1 Mbit/s) y la tabla de identificadores por pata.
- [ ] Definir si el respaldo por ESP-NOW entra en el alcance o queda como trabajo futuro.
- [ ] Revisar si la edición vigente de la ISO 11898-2 es la de 2024 o la de 2026 (ANSI ya lista una edición 2026) antes de citarla.

## Referencias
- ISO 22166-1:2021. *Robotics — Modularity for service robots — Part 1: General requirements.*
- ISO 8373:2021. *Robotics — Vocabulary.*
- ISO 11898-1:2024. *Road vehicles — Controller area network (CAN) — Part 1: Data link layer and physical coding sublayer.* https://www.iso.org/standard/86384.html
- ISO 11898-2:2024. *Road vehicles — Controller area network (CAN) — Part 2: High-speed physical medium attachment (PMA) sublayer.*
- Robert Bosch GmbH (1991). *CAN Specification Version 2.0.*
- IPC-2221B (2012). *Generic Standard on Printed Board Design.*
- IPC-2152 (2009). *Standard for Determining Current Carrying Capacity in Printed Board Design.*
- IPC-7351, IPC-6012, IPC-A-600, IPC-A-610, J-STD-001.
- Texas Instruments. *SN65HVD230 datasheet*; Espressif. *ESP-IDF Programming Guide: TWAI.*

Relacionado: [[normas-y-reglamentos]] · [[justificacion-esp32-por-pata]] · [[arquitectura-electronica-hexapodos]]
