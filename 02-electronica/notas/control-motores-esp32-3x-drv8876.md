---
tipo: componente-tecnico
estado: diseñado
tags:
  - hardware
---

# Interfaz de control de motores: ESP32-WROOM-32 + 3× DRV8876

> [!warning] Contexto actualizado 2026-11-09
> Esta nota supone Wi-Fi/micro-ROS en cada pata. Ahora el **enlace principal entre patas es el bus CAN** (Wi-Fi/ESP-NOW queda como respaldo). La regla de usar ADC1 para IPROPI sigue vigente. Ver [[justificacion-esp32-por-pata]].

## ⚙️ Especificaciones

### 1. Por qué el ESP32 tiene dos ADC (ADC1 y ADC2)

El ESP32 (original, no S2/S3/C3) trae **dos bloques SAR-ADC independientes de 12 bits**, cada uno conectado a un grupo distinto de pines:

| Unidad | Nº de canales | Pines GPIO | Convive con WiFi/BT |
|---|---|---|---|
| **ADC1** | 8 | 32, 33, 34, 35, 36, 37, 38, 39 | ✅ Sí, siempre disponible |
| **ADC2** | 10 | 0, 2, 4, 12, 13, 14, 15, 25, 26, 27 | ❌ No confiable |

No es redundancia: son dos periféricos físicos separados porque **ADC2 comparte hardware con el radio WiFi/Bluetooth**. El driver de WiFi tiene prioridad sobre ADC2, así que en cuanto arrancas `esp_wifi_start()` (lo cual vas a necesitar para ROS2/micro-ROS por WiFi), las lecturas de ADC2 pueden fallar o bloquearse. ADC1 no tiene ese conflicto porque no comparte silicio con el radio.

**Regla práctica para este proyecto:** como el sistema va a llevar WiFi/micro-ROS activo, **toda señal analógica que necesites leer en producción debe ir en un pin de ADC1 (GPIO 32-39)**. Los pines de ADC2 se pueden seguir usando sin problema, pero solo como **entradas/salidas digitales** (ellos no pierden su función digital, solo su función analógica cuando WiFi está activo).

Otras características a tener en cuenta al leer las tres señales IPROPI (corriente de motor):
- Resolución 12 bits (0-4095), rango por defecto ~0-3.3 V con atenuación de 11 dB (`ADC_ATTEN_DB_11` / `analogSetAttenuation(ADC_11db)` en Arduino).
- El ADC del ESP32 tiene no-linealidad conocida, más marcada cerca de 0 V y cerca del fondo de escala — para lecturas de corriente que alimenten el lazo de control en cascada (ver [[24-wang-servo-motor-position-control-dsp]]) conviene calibrar con `esp_adc_cal` (o su equivalente en el framework que uses) en vez de asumir una conversión lineal ideal.
- GPIO 34-39 son **solo entrada** (no tienen resistencias de pull-up/down internas ni pueden usarse como salida) — es una limitación irrelevante para IPROPI porque esa señal siempre es una entrada hacia el ESP32.

### 2. Cómo digitalizar los 3 IPROPI con un solo ADC (no hace falta "un ADC por driver")

"2 ADCs" no significa "solo puedo leer 2 señales analógicas". Cada ADC del ESP32 es **un único convertidor físico con un multiplexor analógico** que rota entre varios pines: ADC1 tiene 8 pines (32-39) compartiendo un solo conversor. Las 3 señales IPROPI (GPIO32, 33, 34) van **todas en ADC1**, leídas una tras otra dentro del mismo ciclo de control — no en paralelo, pero sí en microsegundos.

Tiempos de referencia: la conversión de hardware pura tarda ~1-2 µs; `analogRead()` de Arduino le añade overhead y queda en ~9-12 µs por lectura (~90-100 kS/s medidos). Leer las 3 en secuencia cuesta entonces ~30 µs con Arduino o ~3-6 µs con el driver nativo de ESP-IDF — despreciable frente a un periodo de lazo de corriente típico (cientos de µs a pocos ms, ver [[24-wang-servo-motor-position-control-dsp]]).

**Opción A — Arduino, suficiente para la mayoría de lazos de este proyecto:**
```cpp
const int PIN_IPROPI[3] = {32, 33, 34}; // driver 1, 2, 3 — todos en ADC1

void setup() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // rango ~0-3.3 V
}

void loop() {
  int raw_d1 = analogRead(PIN_IPROPI[0]);
  int raw_d2 = analogRead(PIN_IPROPI[1]);
  int raw_d3 = analogRead(PIN_IPROPI[2]);
  // raw -> voltios -> corriente con tu RIPROPI / AIPROPI
}
```

**Opción B — driver nativo ESP-IDF (`adc_oneshot`), con calibración real (recomendado si esta lectura alimenta el lazo de control, no solo telemetría):**
```c
#include "esp_adc/adc_oneshot.h"

adc_oneshot_unit_handle_t adc1;
adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
adc_oneshot_new_unit(&init_cfg, &adc1);

adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_11 };
adc_oneshot_config_channel(adc1, ADC_CHANNEL_4, &chan_cfg); // GPIO32 → driver 1
adc_oneshot_config_channel(adc1, ADC_CHANNEL_5, &chan_cfg); // GPIO33 → driver 2
adc_oneshot_config_channel(adc1, ADC_CHANNEL_6, &chan_cfg); // GPIO34 → driver 3

int raw;
adc_oneshot_read(adc1, ADC_CHANNEL_4, &raw);
adc_oneshot_read(adc1, ADC_CHANNEL_5, &raw);
adc_oneshot_read(adc1, ADC_CHANNEL_6, &raw);
```
Combínalo con `adc_cali_create_scheme_curve_fitting` (ESP-IDF ≥5.0) para obtener milivoltios calibrados en vez de un crudo de 12 bits — el ADC del ESP32 no es perfectamente lineal, y esta señal retroalimenta el lazo de control.

**Si algún día necesitas las 3 corrientes muestreadas en la misma ventana temporal:** existe el modo continuo/DMA de ADC1 (`adc_continuous.h`, ESP-IDF ≥5.0), que muestrea varios canales por round-robin hasta ~270 kS/s combinados. Para un lazo de cascada por motor como este normalmente no hace falta — la opción A/B ya deja los 3 valores dentro de una ventana de microsegundos. Motivo extra para no usar ADC2 aquí: **ADC2 no soporta el modo DMA**.

### 3. Conexión de 3× DRV8876 al ESP32-WROOM-32

Cada DRV8876 (según el pinout que ya usaste: EN/IN1, PH/IN2, SLEEP, nFAULT, IPROPI) necesita, en modo **PH/EN**:

- **EN/IN1**: entrada digital — habilita el puente H; puedes meterle PWM aquí para controlar velocidad.
- **PH/IN2**: entrada digital — fija el sentido de giro.
- **nSLEEP**: activa el chip en alto (lo que ya hiciste atándolo a 3.3 V).
- **nFAULT**: salida **open-drain**, activa en bajo — según el datasheet/carrier necesita **resistencia pull-up externa** (p. ej. 10 kΩ a 3.3 V) para leerse bien; verifica si tu breakout ya la trae integrada, porque en varios carriers de este chip *no* viene puesta de fábrica.
- **IPROPI**: salida de corriente proporcional a la corriente del motor. En el IC "pelado" se convierte a voltaje con una resistencia externa a GND (RIPROPI); en algunos carriers ya viene esa resistencia puesta. Revisa el esquemático de tu breakout específico antes de asumir un valor — no lo doy por sentado acá para no darte un dato posiblemente incorrecto para tu placa exacta.
- **PMODE** (si tu breakout lo expone como pin aparte): debe quedar en **bajo** para que EN/IN1 y PH/IN2 se comporten como enable/dirección (modo PH/EN). Si el pin queda flotando, el chip arranca en modo "medio puente independiente" y tus conexiones actuales no funcionarían como esperas — confírmalo en el esquemático de tu breakout (en varios carriers viene ya fijado en la placa y no se expone como pin).

**Restricciones del ESP32-WROOM-32 a respetar al elegir pines:**
- GPIO 6-11 → **nunca**, están cableados a la flash SPI interna (el chip se cuelga).
- GPIO 0, 2, 12, 15 → pines de *strapping* (modo de arranque); evitarlos para señales que un driver pueda forzar a un nivel inesperado durante el reset.
- GPIO 1, 3 → UART0 (USB-serial); evítalos si quieres conservar el monitor serie.
- GPIO 34-39 → solo entrada, ideales para nFAULT/IPROPI, no sirven para EN/IN1/PH/IN2 (que son salidas).
- GPIO 21/22 → los dejo libres a propósito: son los pines I2C por defecto en el core Arduino de ESP32, y tu carpeta de electrónica ya prevé un IMU — te conviene reservarlos para el bus I2C en vez de gastarlos en los drivers.

**Asignación propuesta** (driver 1 = la tuya, ya conectada; 2 y 3 nuevas):

| Señal  | Driver 1 (ya conectado)                             | Driver 2                   | Driver 3                   |
| ------ | --------------------------------------------------- | -------------------------- | -------------------------- |
| EN/IN1 | GPIO27                                              | GPIO14                     | GPIO18                     |
| PH/IN2 | GPIO26                                              | GPIO13                     | GPIO19                     |
| nSLEEP | 3.3V (fijo)                                         | GPIO17*                    | GPIO17*                    |
| nFAULT | GPIO4 (+ pull-up externo si tu breakout no lo trae) | GPIO35 (+ pull-up externo) | GPIO39 (+ pull-up externo) |
| IPROPI | GPIO32 — ADC1_CH4                                   | GPIO33 — ADC1_CH5          | GPIO34 — ADC1_CH6          |

\* **Sugerencia, no obligatoria:** en vez de atar cada nSLEEP a 3.3 V como hiciste con el driver 1 (válido, pero deja el chip siempre despierto), considera unir las 3 líneas nSLEEP en un solo pin GPIO17 controlado por software. Así puedes dormir los tres drivers a la vez (ahorro de energía, o como parada de emergencia) sin gastar 3 GPIOs distintos. Si prefieres mantener el driver 1 tal como está, los drivers 2 y 3 pueden ir igual atados directo a 3.3 V — no es obligatorio homogeneizar.

Por qué esta asignación:
- Los 3 IPROPI quedan **exclusivamente en ADC1** (32/33/34) para que las lecturas de corriente sigan siendo confiables con WiFi/micro-ROS activo (ver sección 1).
- EN/IN1, PH/IN2 y nFAULT del driver 1 ya están en pines de ADC2 (27, 26, 4) — está bien porque los usas como digitales puros, no como analógicos; por eso repito el mismo criterio en drivers 2 y 3, mezclando pines ADC2 y no-ADC libremente para las señales digitales.
- Quedan libres GPIO16 y GPIO25 por si necesitas una señal extra (p. ej. otro nFAULT en modo bus, o un pin para el sensor de gases).

### 4. ¿Se pueden unir los 3 nFAULT en un solo pin del ESP32?

**Sí, es válido eléctricamente.** nFAULT es una salida *open-drain* activa en bajo — el chip solo tira la línea a GND, nunca la empuja a alto — por lo que varias salidas open-drain se pueden unir en un mismo nodo sin riesgo de cortocircuito entre drivers (mismo principio que un bus I2C). Basta **una sola resistencia pull-up compartida** (p. ej. 10 kΩ a 3.3 V) en el nodo común, no una por driver:

```
nFAULT driver 1 ──┐
nFAULT driver 2 ──┼── GPIO ESP32 ── pull-up 10 kΩ a 3.3V
nFAULT driver 3 ──┘
```

**Trade-off:** el ESP32 solo sabe "algo falló en alguno de los tres", no cuál — el DRV8876 no tiene registro de estado leíble para diferenciar la causa después del hecho. Para un robot que debe decidir autónomamente qué hacer ante una falla (¿bajar velocidad de una pata? ¿abortar la misión?), identificar el motor culpable suele valer más que ahorrar 2 GPIOs — y en la asignación de esta nota ya sobran GPIO35/GPIO39 para mantenerlos separados.

**Recomendación:** mantener las 3 líneas individuales (como en la tabla de la sección 3). Reservar la unión en un solo pin como opción si en algún momento faltan GPIOs, o si solo necesitas una interrupción global tipo "algo se rompió, detén todo ya" además de (no en vez de) las líneas individuales para diagnóstico.

### 5. Pinout completo del DRV8876 — función de cada pin

El IC viene en un empaque de 16 pines (HTSSOP); en tu breakout probablemente no ves todos estos pines en el header porque el fabricante ya resuelve algunos en la placa (ver nota al final de la tabla). Los nombres son los que usa el datasheet de TI:

| Pin | Tipo | Función |
|---|---|---|
| VM | Alimentación | Alimentación del motor (4.5-37 V). Lleva un capacitor de desacople a GND cerca del pin. |
| GND | Tierra | Tierra de referencia lógica. |
| PGND | Tierra | Tierra de potencia (retorno de corriente del puente H) — internamente separada de GND para bajar ruido, pero en el PCB del breakout ya está unida a la misma tierra. |
| OUT1 / OUT2 | Salida de potencia | Van directo a las dos terminales del motor. |
| EN/IN1 | Entrada de control | En modo PH/EN (el que usas): habilita el puente H, admite PWM para velocidad. En modo PWM/PWM: es IN1 directo. |
| PH/IN2 | Entrada de control | En modo PH/EN: fija el sentido de giro. En modo PWM/PWM: es IN2 directo. |
| PMODE | Entrada de configuración | Selecciona el modo de control **una sola vez, al encender** (queda "latcheado" hasta el siguiente power-on/nSLEEP): bajo = PH/EN, alto = PWM/PWM, flotante = medio puente independiente. Es el pin que te mencioné antes: si tu breakout no lo fija internamente en bajo, tus EN/IN1 y PH/IN2 no se comportan como esperas. |
| nSLEEP | Entrada | Alto = chip activo; bajo = duerme (consumo de µA). Es lo que ya atas a 3.3V. |
| nFAULT | Salida open-drain, activa en bajo | Avisa sobretemperatura, sobrecorriente, subvoltaje, etc. Necesita pull-up externo (ver sección 4 de más arriba). |
| VREF | Entrada analógica | Junto con IMODE y la resistencia externa en IPROPI, define el límite de corriente (ITRIP) al que el chip empieza a recortar (chopping). |
| IMODE | Entrada de configuración | Selecciona el modo de regulación de corriente (varios niveles según el voltaje/resistencia que le pongas). |
| IPROPI | Salida analógica | Corriente proporcional a la corriente real del motor; se convierte en voltaje con una resistencia externa a GND (RIPROPI) — es la señal que llevas a tus GPIO32/33/34. |
| VCP, CPH, CPL | Nodos de la bomba de carga | Generan internamente un voltaje por encima de VM para manejar los MOSFET del puente H. Llevan un capacitor cerámico externo (típico 22 nF) entre CPH y CPL. |

**¿Tiene el DRV8876 pines NC?** No. Las 16 patas tienen función — lo que en un breakout parece "falta un pin" (VCP/CPH/CPL, PGND) normalmente es porque el fabricante de la placa ya puso el capacitor de la bomba de carga y unió las tierras por ti, y por eso no los expone en el header. No es que sobren pines, es que ya están resueltos en la placa.

### 6. Pinout del ESP32-WROOM-32 — función de cada pin y el pin NC

El módulo tiene 38 pines físicos. Agrupados por función:

| Grupo | Pines | Notas |
|---|---|---|
| Alimentación/control del módulo | 3V3, GND (varios), EN | EN es el reset/enable del chip completo, activo en alto — no confundir con el EN/IN1 del driver. |
| Solo entrada, sin pull-up/down interno | GPIO34, GPIO35, GPIO36 (SENSOR_VP), GPIO39 (SENSOR_VN) | No sirven como salida. Los usamos para IPROPI y nFAULT en esta nota porque esas señales siempre son entradas hacia el ESP32. |
| Flash SPI interna — **nunca usar** | GPIO6-11 (en el datasheet: SD2, SD3, CMD, CLK, SD0, SD1) | Conectarles algo cuelga el chip al arrancar. |
| Strapping (modo de arranque) | GPIO0, GPIO2, GPIO5, GPIO12, GPIO15 | Un nivel externo forzado aquí durante el reset puede meter al ESP32 en modo descarga de firmware o falla de arranque. |
| UART0 (programación/consola serie) | GPIO1 (TXD0), GPIO3 (RXD0) | Evitarlos si quieres seguir viendo el monitor serie. |
| Uso general (los que usamos para los drivers) | GPIO4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 | Sin restricciones especiales. |
| **NC** | 1 pin físico (entre IO19 e IO21 en la numeración del módulo) | Ver abajo. |

**¿Por qué hay un pin NC?** El propio datasheet de Espressif no da una razón explícita — solo lo lista con "Tipo: -" y "Función: -", es decir, no está conectado a nada dentro del módulo. La explicación habitual de ingeniería (no textual del datasheet, pero es la práctica estándar del sector) es que Espressif mantiene el mismo footprint de 38 pines entre varias variantes del módulo (WROOM, WROVER, etc.); un pin que en una variante tiene señal, en otra puede quedar sin usar para no cambiar la huella del PCB. **Un pin NC no es un GPIO gratis**: no está internamente conectado a ningún pad del chip, así que conectarle algo no hace nada (o solo capta ruido) — se deja sin soldar/flotando.

### 7. Los "sensor BP y BN" son en realidad VP y VN (SENSOR_VP / SENSOR_VN)

Son GPIO36 (`SENSOR_VP`, ADC1_CH0) y GPIO39 (`SENSOR_VN`, ADC1_CH3). El nombre viene de que originalmente el ESP32 los usa para leer, de forma diferencial, su **sensor Hall interno** (V positivo y V negativo de esa medición). Por eso:
- Ambos tienen capacitores internos de 270 pF hacia otros dos pines (S_CAPP/S_CAPN, GPIO37/38) para esa medición diferencial de precisión.
- Si no usas el sensor Hall interno ni necesitas esa medición diferencial (no es el caso de este proyecto), se pueden usar como GPIO de entrada normales — es justo lo que hacemos con GPIO39 como nFAULT del driver 3 en la tabla de la sección 3. El capacitor parásito de 270 pF es irrelevante para una señal digital lenta como nFAULT; no hace falta cambiar nada de lo ya propuesto.
- Lo único que no deberías hacer es usarlos como salida (son solo-entrada, como el resto del grupo 34-39) ni asumir que tienen pull-up interno (no lo tienen).

## 🔧 Estado
Diseñado — driver 1 implementado y cableado; drivers 2 y 3 pendientes de armar con la asignación de pines de esta nota.

## 📊 Resultados de prueba
_(pendiente — completar cuando se pruebe la lectura de IPROPI y la respuesta de EN/IN1 con PWM)_

## 🔗 Relacionado
- [[24-wang-servo-motor-position-control-dsp]] — arquitectura de control en cascada que consume estas 3 señales IPROPI como realimentación de corriente
- [[TESIS]]
