/*
 * Diagnostico CAN de UNA placa sola: no hace falta otra placa en el bus.
 *
 * Misma fuente para las dos placas (se elige al compilar con el target):
 *   esp32   -> MOD_DRV: IO21 -> CTX (J4-3), IO22 <- CRX (J4-4), consola UART 115200.
 *   esp32s3 -> maestra: GPIO4 -> CTX, GPIO5 <- CRX, consola por el USB nativo.
 *
 * Pruebas (comando 'todo', y se corre sola al arrancar):
 *   1 ctx     IO de CTX libre con pull-up: un 0 indica corto a GND en la linea CTX.
 *   2 eco     CTX manejado por GPIO: 1 -> CRX debe dar 1; 0 -> CRX debe dar 0.
 *             Comprueba CTX, el modulo y CRX sin el controlador CAN.
 *   3 reposo  CTX en 1 durante 100 ms: CRX no debe bajar nunca.
 *   4 twai    200 tramas a 500 kbit/s en modo NO_ACK con autorrecepcion:
 *             el controlador TWAI, el modulo y su eco interno, con datos verificados.
 *   5 motores (solo MOD_DRV) nFAULT de los 3 DRV8876 y nivel de los encoders.
 * Ademas: 'escucha' (5 s en solo escucha, para una placa conectada al bus) y
 * 'm' (monitor cada segundo: CRX, encoders, nFAULT).
 *
 * Seguridad: los 6 IN de los DRV8876 quedan en 0 todo el tiempo (motores sueltos).
 * La NVS solo se lee (ID de pata): este firmware no borra ni escribe nada en ella.
 * Las pruebas 1-4 ponen el bus en dominante unos microsegundos: correrlas con la
 * placa DESCONECTADA del bus (CANH y CANL sueltos) para que el resultado sea solo suyo.
 */
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3
#define PLACA      "MAESTRA ESP32-S3"
#define PIN_CTX     GPIO_NUM_4
#define PIN_CRX     GPIO_NUM_5
#define TXT_CTX     "GPIO4 (CTX del modulo)"
#define TXT_CRX     "GPIO5 (CRX del modulo)"
#define HAY_MOTORES 0
#else
#define PLACA       "MOD_DRV (ESP32)"
#define PIN_CTX     GPIO_NUM_21
#define PIN_CRX     GPIO_NUM_22
#define TXT_CTX     "IO21 / J4-3 (CTX)"
#define TXT_CRX     "IO22 / J4-4 (CRX)"
#define HAY_MOTORES 1
#endif

#define ESPERA(ms) (pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1)

typedef enum { PASA, FALLA, DUDA, NO_CORRIDA } res_t;
static const char *txt_res(res_t r)
{
    switch (r) {
    case PASA:  return "PASA";
    case FALLA: return "FALLA";
    case DUDA:  return "DUDOSA";
    default:    return "-";
    }
}

/* Resultados de la ultima corrida, para el diagnostico final. */
static struct {
    res_t ctx, eco, reposo, twai, motores;
    bool ctx_no_sube, ctx_no_baja;   /* CTX no sigue al GPIO (corto) */
    int alto_crx0, bajo_crx1, n_eco; /* fallos del eco: con CTX=1 CRX=0 / con CTX=0 CRX=1 */
    int reposo_pct;
} R;

static bool s_monitor;

/* ---------------------------------------------------------------- motores */
#if HAY_MOTORES
static const char *const k_art[3] = {"coxa ", "femur", "tibia"};
static const gpio_num_t k_in[6]     = {26, 27, 18, 19, 16, 2};
static const gpio_num_t k_nfault[3] = {5, 14, 15};
static const gpio_num_t k_enc_a[3]  = {35, 23, 4};
static const gpio_num_t k_enc_b[3]  = {25, 17, 13};
#define CUENTAS_VUELTA 2800.0f   /* 7 PPR x 4 x 100, igual que pata_MOD_DRV */

static volatile int32_t s_cnt[3];
static volatile uint32_t s_saltos[3];   /* cambian A y B a la vez: ruido o flanco perdido */
static volatile uint8_t s_prev[3];
static const int8_t k_qdec[16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};

static void isr_encoder(void *arg)
{
    int i = (int)(intptr_t)arg;
    uint8_t s = (uint8_t)((gpio_get_level(k_enc_a[i]) << 1) | gpio_get_level(k_enc_b[i]));
    int8_t d = k_qdec[(s_prev[i] << 2) | s];
    if (d == 2) s_saltos[i]++;
    else s_cnt[i] += d;
    s_prev[i] = s;
}

static void motores_iniciar(void)
{
    /* Los 6 IN a 0 antes que nada: con PMODE en PWM, IN1 = IN2 = 0 deja el motor suelto. */
    for (int i = 0; i < 6; i++) {
        gpio_reset_pin(k_in[i]);
        gpio_set_level(k_in[i], 0);
        gpio_set_direction(k_in[i], GPIO_MODE_OUTPUT);
    }
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(k_nfault[i]);
        gpio_set_direction(k_nfault[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(k_nfault[i], GPIO_PULLUP_ONLY);   /* ademas de R4 10 k */
    }
    gpio_install_isr_service(0);
    for (int i = 0; i < 3; i++) {
        gpio_num_t p[2] = {k_enc_a[i], k_enc_b[i]};
        for (int k = 0; k < 2; k++) {
            if (GPIO_IS_VALID_OUTPUT_GPIO(p[k])) gpio_reset_pin(p[k]);   /* IO35: solo entrada */
            gpio_set_direction(p[k], GPIO_MODE_INPUT);
            /* IO35 no tiene pull-up interno */
            gpio_set_pull_mode(p[k], p[k] == GPIO_NUM_35 ? GPIO_FLOATING : GPIO_PULLUP_ONLY);
            gpio_set_intr_type(p[k], GPIO_INTR_ANYEDGE);
            gpio_isr_handler_add(p[k], isr_encoder, (void *)(intptr_t)i);
        }
        s_prev[i] = (uint8_t)((gpio_get_level(k_enc_a[i]) << 1) | gpio_get_level(k_enc_b[i]));
    }
}

static void prueba_motores(void)
{
    puts("\n[5] MOTORES (solo lectura: los IN de los DRV8876 estan en 0)");
    bool falla = false;
    for (int i = 0; i < 3; i++) {
        int nf = gpio_get_level(k_nfault[i]);
        printf("  %s  nFAULT=%d %s  encoder A=%d B=%d  cuentas=%" PRId32 " (%.3f rad)  saltos=%" PRIu32 "\n",
               k_art[i], nf, nf ? "(OK)   " : "(FALLA)", gpio_get_level(k_enc_a[i]),
               gpio_get_level(k_enc_b[i]), s_cnt[i], s_cnt[i] * 6.2831853f / CUENTAS_VUELTA,
               s_saltos[i]);
        if (!nf) falla = true;
    }
    if (falla) {
        puts("  -> nFAULT en 0 con los motores quietos: ese DRV8876 senala falla.\n"
             "     Sin V_MOT es normal en algunos casos; con V_MOT revisa el motor, su cable\n"
             "     y un posible corto en la salida (J3 pines 1-2).");
    } else {
        puts("  -> nFAULT bien en los tres.");
    }
    puts("  Para probar los encoders: escribe 'm' y gira a mano cada articulacion.\n"
         "  Las cuentas deben cambiar de forma suave; 'saltos' debe quedar en 0 o casi.");
    R.motores = falla ? FALLA : PASA;
}
#endif

/* ---------------------------------------------------------------- pines CAN */

/* Deja CTX en 1 manejado por GPIO (bus recesivo) y CRX como entrada. */
static void pines_gpio(void)
{
    gpio_reset_pin(PIN_CTX);
    gpio_set_level(PIN_CTX, 1);
    gpio_set_direction(PIN_CTX, GPIO_MODE_INPUT_OUTPUT);
    gpio_reset_pin(PIN_CRX);
    gpio_set_direction(PIN_CRX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_CRX, GPIO_FLOATING);
}

static int contar_unos(gpio_num_t p, int n, int paso_us)
{
    int u = 0;
    for (int i = 0; i < n; i++) {
        u += gpio_get_level(p);
        if (paso_us) esp_rom_delay_us(paso_us);
    }
    return u;
}

/* [1] CTX solo con el pull-up interno (~45 k): cualquier cosa que lo tire a 0 es un corto. */
static void prueba_ctx(void)
{
    puts("\n[1] LINEA CTX LIBRE (" TXT_CTX ", solo pull-up interno)");
    gpio_reset_pin(PIN_CTX);
    gpio_set_direction(PIN_CTX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_CTX, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(2000);
    int unos = contar_unos(PIN_CTX, 100, 10);
    printf("  CTX leido en 1: %d de 100\n", unos);
    if (unos == 100) {
        puts("  -> PASA: nada tira de CTX hacia 0 V.");
        R.ctx = PASA;
    } else if (unos == 0) {
        puts("  -> FALLA: CTX esta en 0 V aun sin manejarlo. Hay un corto a GND en la linea\n"
             "     CTX (puente de estano en J4-3 o cerca del ESP32) o el modulo la fuerza.\n"
             "     Con la placa APAGADA mide continuidad entre J4-3 y GND.");
        R.ctx = FALLA;
    } else {
        puts("  -> DUDOSA: CTX cambia solo. Contacto flojo o ruido en J4-3.");
        R.ctx = DUDA;
    }
    pines_gpio();
}

/* [2] El transceptor repite en CRX lo que se pone en CTX (eco local). */
static void prueba_eco(void)
{
    puts("\n[2] ECO CTX -> CRX por GPIO (sin el controlador CAN)");
    pines_gpio();
    const int n = 50;
    int ctx_alto_mal = 0, ctx_bajo_mal = 0, alto_crx0 = 0, bajo_crx1 = 0;
    for (int i = 0; i < n; i++) {
        gpio_set_level(PIN_CTX, 1);
        esp_rom_delay_us(200);
        if (gpio_get_level(PIN_CTX) != 1) ctx_alto_mal++;
        if (gpio_get_level(PIN_CRX) != 1) alto_crx0++;

        /* Dominante solo 10 us (5 bits a 500 kbit/s): si la placa sigue en un bus, los demas
         * lo ven como una trama erronea y nada mas. */
        gpio_set_level(PIN_CTX, 0);
        esp_rom_delay_us(5);
        if (gpio_get_level(PIN_CTX) != 0) ctx_bajo_mal++;
        if (gpio_get_level(PIN_CRX) != 0) bajo_crx1++;
        gpio_set_level(PIN_CTX, 1);
    }
    esp_rom_delay_us(200);
    printf("  CTX=1: CTX releido mal %d/%d, CRX en 0 %d/%d   (debe ser 0 y 0)\n",
           ctx_alto_mal, n, alto_crx0, n);
    printf("  CTX=0: CTX releido mal %d/%d, CRX en 1 %d/%d   (debe ser 0 y 0)\n",
           ctx_bajo_mal, n, bajo_crx1, n);

    R.ctx_no_sube = ctx_alto_mal > n / 2;
    R.ctx_no_baja = ctx_bajo_mal > n / 2;
    R.alto_crx0 = alto_crx0;
    R.bajo_crx1 = bajo_crx1;
    R.n_eco = n;
    if (R.ctx_no_sube) puts("  -> CTX no sube a 1 aunque el ESP32 lo pone en 1: corto de CTX a GND.");
    if (R.ctx_no_baja) puts("  -> CTX no baja a 0 aunque el ESP32 lo pone en 0: corto de CTX a 3V3.");
    if (alto_crx0 == n)
        puts("  -> Con CTX=1 el modulo entrega CRX=0 (dominante) siempre: ver diagnostico.");
    else if (alto_crx0)
        puts("  -> Con CTX=1, CRX baja a veces: contacto flojo, ruido u otro nodo transmitiendo.");
    if (bajo_crx1 == n)
        puts("  -> Con CTX=0 el modulo NO pone el bus en dominante: ver diagnostico.");
    else if (bajo_crx1)
        puts("  -> Con CTX=0, CRX no siempre baja: contacto flojo en CTX/CRX o en la alimentacion.");

    if (!ctx_alto_mal && !ctx_bajo_mal && !alto_crx0 && !bajo_crx1) {
        puts("  -> PASA: el modulo sigue a CTX en los dos niveles.");
        R.eco = PASA;
    } else {
        R.eco = (alto_crx0 == n || bajo_crx1 == n || R.ctx_no_sube || R.ctx_no_baja) ? FALLA : DUDA;
    }
}

/* [3] Bus en reposo: con CTX=1 y nadie mas transmitiendo, CRX no debe bajar. */
static void prueba_reposo(void)
{
    puts("\n[3] REPOSO: CTX=1 durante 100 ms, CRX no debe bajar");
    pines_gpio();
    esp_rom_delay_us(500);
    const int n = 2000;
    int ceros = n - contar_unos(PIN_CRX, n, 50);
    R.reposo_pct = ceros * 100 / n;
    printf("  CRX en 0: %d de %d muestras (%d %%)\n", ceros, n, R.reposo_pct);
    if (ceros == 0) {
        puts("  -> PASA: reposo limpio.");
        R.reposo = PASA;
    } else if (ceros == n) {
        puts("  -> FALLA: CRX pegado en 0. Si la placa esta desconectada del bus, el problema\n"
             "     es de esta placa (ver diagnostico). Si esta conectada, puede ser otra.");
        R.reposo = FALLA;
    } else {
        puts("  -> DUDOSA: CRX baja a ratos. Desconectada del bus: ruido o contacto flojo.\n"
             "     Conectada: otra placa esta transmitiendo (normal si hay trafico).");
        R.reposo = DUDA;
    }
}

/* [4] Controlador TWAI completo a 500 kbit/s. En NO_ACK la trama no necesita ACK de nadie;
 * con autorrecepcion el controlador la lee de vuelta por CRX (pasa por el modulo). */
static void prueba_twai(void)
{
    puts("\n[4] CONTROLADOR TWAI: 200 tramas a 500 kbit/s con autorrecepcion (NO_ACK)");
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_NO_ACK);
    g.tx_queue_len = 4;
    g.rx_queue_len = 8;
    g.alerts_enabled = TWAI_ALERT_NONE;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e = twai_driver_install(&g, &t, &f);
    if (e != ESP_OK) {
        printf("  -> FALLA: no se pudo instalar el driver TWAI (%s)\n", esp_err_to_name(e));
        R.twai = FALLA;
        pines_gpio();
        return;
    }
    twai_start();

    const int n = 200;
    int ok = 0, no_sale = 0, no_vuelve = 0, distinta = 0;
    bool bus_off = false;
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < n && !bus_off; i++) {
        /* IDs 0x7E0..0x7EF: fuera del protocolo de las patas (0x0FF, 0x1xx, 0x2xx). */
        twai_message_t m = {.identifier = 0x7E0 + (i & 0xF), .self = 1, .ss = 1,
                            .data_length_code = (uint8_t)(i % 9)};
        for (int k = 0; k < m.data_length_code; k++)
            m.data[k] = (uint8_t)((i * 7 + k * 31) ^ ((k & 1) ? 0xFF : 0x00));
        if (twai_transmit(&m, ESPERA(20)) != ESP_OK) {
            no_sale++;
        } else {
            twai_message_t r;
            if (twai_receive(&r, ESPERA(20)) != ESP_OK) {
                no_vuelve++;
            } else if (r.identifier != m.identifier || r.data_length_code != m.data_length_code ||
                       memcmp(r.data, m.data, m.data_length_code)) {
                distinta++;
            } else {
                ok++;
            }
        }
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) bus_off = true;
    }
    int64_t dt = esp_timer_get_time() - t0;
    twai_status_info_t st = {0};
    twai_get_status_info(&st);
    printf("  correctas %d/%d   no salieron %d   no volvieron %d   distintas %d\n",
           ok, n, no_sale, no_vuelve, distinta);
    printf("  TEC=%" PRIu32 " REC=%" PRIu32 " errores de bus=%" PRIu32 "%s   tiempo %.1f ms\n",
           st.tx_error_counter, st.rx_error_counter, st.bus_error_count,
           bus_off ? "  BUS_OFF" : "", dt / 1000.0);
    if (st.state == TWAI_STATE_RUNNING) twai_stop();
    twai_driver_uninstall();
    pines_gpio();

    if (ok == n && st.bus_error_count == 0) {
        puts("  -> PASA: el controlador, el modulo y el eco funcionan a 500 kbit/s.");
        R.twai = PASA;
    } else if (ok == 0) {
        puts("  -> FALLA: ninguna trama volvio bien. El controlador ve errores de bit en CRX.");
        R.twai = FALLA;
    } else {
        puts("  -> DUDOSA: algunas tramas fallan. Contacto flojo, ruido o placa conectada a un\n"
             "     bus con trafico (repite desconectada).");
        R.twai = DUDA;
    }
}

/* ---------------------------------------------------------------- diagnostico */

static void diagnostico(void)
{
    puts("\n================ DIAGNOSTICO ================");
    printf("  [1] ctx %s   [2] eco %s   [3] reposo %s   [4] twai %s", txt_res(R.ctx),
           txt_res(R.eco), txt_res(R.reposo), txt_res(R.twai));
#if HAY_MOTORES
    printf("   [5] motores %s", txt_res(R.motores));
#endif
    putchar('\n');

    bool can_ok = R.ctx == PASA && R.eco == PASA && R.reposo == PASA && R.twai == PASA;
    if (can_ok) {
        puts("RESULTADO,CAN,PASA");
        puts("  El CAN de esta placa funciona sola. Si el bus falla al conectarla, la causa\n"
             "  esta fuera del modulo: cable, conector P1/P2, CANH/CANL invertidos o GND.");
    } else if (R.ctx == FALLA || R.ctx_no_sube) {
        puts("RESULTADO,CAN,FALLA,LINEA_CTX_A_GND");
        puts("  La linea CTX (" TXT_CTX ") esta tirada a 0 V. Con CTX en 0 el modulo\n"
             "  transmite dominante sin parar y BLOQUEA EL BUS DE TODAS LAS PLACAS.\n"
             "  Con la placa apagada: continuidad J4-3 <-> GND. Repasa soldaduras de J4\n"
             "  y del pin del ESP32. No hace falta quitar el modulo.");
    } else if (R.ctx_no_baja) {
        puts("RESULTADO,CAN,FALLA,LINEA_CTX_A_3V3");
        puts("  CTX no baja: corto de la linea CTX con 3V3. Esta placa no puede transmitir.");
    } else if (R.alto_crx0 == R.n_eco && R.n_eco) {
        puts("RESULTADO,CAN,FALLA,MODULO_DOMINANTE");
        puts("  CTX esta bien (en 1), pero el modulo entrega dominante. Con la placa\n"
             "  DESCONECTADA del bus las causas posibles son:\n"
             "   a) modulo sin 3,3 V: mide J4-1 respecto a J4-2 (debe dar ~3,3 V);\n"
             "   b) CANH en corto a 3V3 o CANL en corto a GND: con la placa encendida\n"
             "      mide CANH y CANL respecto a GND (en reposo ~2,3-2,5 V cada uno);\n"
             "   c) CRX (" TXT_CRX ") en corto a GND: con la placa apagada, continuidad J4-4 <-> GND;\n"
             "   d) el transceptor del modulo esta danado.\n"
             "  Si estaba CONECTADA al bus, repite desconectada: puede ser otra placa.");
    } else if (R.bajo_crx1 == R.n_eco && R.n_eco) {
        puts("RESULTADO,CAN,FALLA,MODULO_NO_TRANSMITE");
        puts("  Con CTX=0 el modulo no pone el bus en dominante. Causas:\n"
             "   a) J4-3 sin contacto entre el ESP32 y el modulo (soldadura fria);\n"
             "   b) modulo sin 3,3 V (mide J4-1);\n"
             "   c) CANH y CANL en corto entre si (mide la resistencia: ~60 ohm en el bus\n"
             "      completo, ~120 ohm en una placa sola; 0 ohm es corto).");
    } else if (R.twai == FALLA && R.eco == PASA) {
        puts("RESULTADO,CAN,FALLA,CONTROLADOR");
        puts("  El modulo responde por GPIO pero el TWAI no: revisa que el pin de CRX\n"
             "  sea el correcto y repite. Si persiste, el ESP32 puede estar danado.");
    } else {
        puts("RESULTADO,CAN,DUDOSA");
        puts("  Hay fallos intermitentes: contacto flojo, soldadura fria o ruido.\n"
             "  Repite 'todo' varias veces moviendo con cuidado el modulo y el cable:\n"
             "  si el resultado cambia al tocar algo, ahi esta el contacto malo.");
    }
    puts("=============================================");
}

static void todo(void)
{
    memset(&R, 0, sizeof R);
    R.ctx = R.eco = R.reposo = R.twai = R.motores = NO_CORRIDA;
    puts("\n### DIAGNOSTICO DE UNA PLACA (" PLACA ") ###");
    puts("Para que el resultado sea solo de esta placa: CANH y CANL DESCONECTADOS.");
    prueba_ctx();
    prueba_eco();
    prueba_reposo();
    prueba_twai();
#if HAY_MOTORES
    prueba_motores();
#endif
    diagnostico();
}

/* ---------------------------------------------------------------- escucha y monitor */

/* Solo escucha 5 s: no transmite ni da ACK. Sirve con la placa conectada al bus. */
static void escucha(void)
{
    puts("\nESCUCHA 5 s (no transmite ni da ACK; sirve con la placa conectada al bus)");
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_LISTEN_ONLY);
    g.rx_queue_len = 32;
    g.alerts_enabled = TWAI_ALERT_NONE;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
        puts("  no se pudo arrancar el TWAI");
        twai_driver_uninstall();
        pines_gpio();
        return;
    }
    uint32_t tramas = 0, muestras = 0, dom = 0;
    uint32_t ids[8] = {0}, n_ids = 0;
    int64_t fin = esp_timer_get_time() + 5000000;
    while (esp_timer_get_time() < fin) {
        twai_message_t r;
        while (twai_receive(&r, 0) == ESP_OK) {
            tramas++;
            bool visto = false;
            for (uint32_t k = 0; k < n_ids; k++) visto |= ids[k] == r.identifier;
            if (!visto && n_ids < 8) ids[n_ids++] = r.identifier;
        }
        muestras++;
        dom += gpio_get_level(PIN_CRX) == 0;
        vTaskDelay(ESPERA(1));
    }
    twai_status_info_t st = {0};
    twai_get_status_info(&st);
    twai_stop();
    twai_driver_uninstall();
    pines_gpio();
    printf("  tramas recibidas %" PRIu32 "   errores de bus %" PRIu32 "   CRX en 0 %" PRIu32 " %%\n",
           tramas, st.bus_error_count, muestras ? dom * 100 / muestras : 0);
    printf("  IDs vistos:");
    for (uint32_t k = 0; k < n_ids; k++) printf(" 0x%03" PRIX32, ids[k]);
    puts(n_ids ? "" : " ninguno");
    if (muestras && dom * 100 / muestras > 90)
        puts("  -> El bus esta pegado en dominante. Desconecta esta placa y corre 'todo'.");
    else if (!tramas && !st.bus_error_count)
        puts("  -> Bus en silencio y sin errores (nadie transmite, o placa desconectada).");
    else if (st.bus_error_count)
        puts("  -> Hay errores de bus: cableado, terminacion o un nodo que molesta.");
    else
        puts("  -> Hay trafico sano en el bus.");
}

static void monitor_linea(void)
{
    int dom = 2000 - contar_unos(PIN_CRX, 2000, 5);
    printf("MON,t=%.1f s,CTX=%d,CRX_en_0=%d%%", esp_timer_get_time() / 1e6, gpio_get_level(PIN_CTX),
           dom * 100 / 2000);
#if HAY_MOTORES
    for (int i = 0; i < 3; i++)
        printf(",%s=%" PRId32 "(%.2f rad,saltos %" PRIu32 ",nF=%d)", k_art[i], s_cnt[i],
               s_cnt[i] * 6.2831853f / CUENTAS_VUELTA, s_saltos[i], gpio_get_level(k_nfault[i]));
#endif
    putchar('\n');
}

/* ---------------------------------------------------------------- bus con las patas */
/* Solo ordenes que no mueven nada: IDENT [06] (responde [06 MAC]) y PING (dlc 0, responde
 * PONG [00]), del protocolo de pata_MOD_DRV/main/can_protocol.h. Nunca arma.
 * La placa queda fuera del bus (driver desinstalado) al terminar cada prueba. */
#define ID_ORDEN(n)  (0x100 + (n))
#define ID_RESP(n)   (0x200 + (n))
typedef enum { B_OK, B_SIN_ACK, B_NO_SALIO, B_NO_ENCOLADA } btx_t;
static const char *txt_btx(btx_t r)
{
    return r == B_OK ? "OK" : r == B_SIN_ACK ? "SIN_ACK" : r == B_NO_SALIO ? "NO_SALIO" : "NO_ENCOLADA";
}

static int crx_dominante_pct(int ms)
{
    int n = ms * 20;
    return (n - contar_unos(PIN_CRX, n, 50)) * 100 / n;
}

static bool s_bus;   /* driver TWAI instalado en NORMAL por bus_iniciar() */
static bool s_instalado_bus(void) { return s_bus; }

static bool bus_iniciar(void)
{
    int dom = crx_dominante_pct(20);
    if (dom > 90) {
        printf("BUS,PEGADO,%d%%\n", dom);
        puts("  El bus esta en dominante antes de transmitir: alguna placa lo fuerza.\n"
             "  Quita placas de una en una hasta que se libere.");
        return false;
    }
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CTX, PIN_CRX, TWAI_MODE_NORMAL);
    g.tx_queue_len = 4;
    g.rx_queue_len = 32;
    g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) { puts("BUS,ERROR,instalar"); pines_gpio(); return false; }
    if (twai_start() != ESP_OK) { puts("BUS,ERROR,arrancar"); twai_driver_uninstall(); pines_gpio(); return false; }
    s_bus = true;
    return true;
}

static void bus_parar(void)
{
    twai_status_info_t st = {0};
    twai_get_status_info(&st);
    printf("BUS,TEC=%" PRIu32 ",REC=%" PRIu32 ",errores=%" PRIu32 ",estado=%s\n", st.tx_error_counter,
           st.rx_error_counter, st.bus_error_count,
           st.state == TWAI_STATE_BUS_OFF ? "BUS_OFF" : st.state == TWAI_STATE_RUNNING ? "NORMAL" : "OTRO");
    if (st.state == TWAI_STATE_RUNNING) twai_stop();
    twai_driver_uninstall();
    s_bus = false;
    pines_gpio();
}

static bool bus_off(void)
{
    twai_status_info_t st;
    return twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF;
}

/* Un solo intento (ss = 1). Espera a saber si la trama salio con ACK. */
static btx_t bus_enviar(uint32_t id, const uint8_t *d, uint8_t len)
{
    uint32_t a;
    while (twai_read_alerts(&a, 0) == ESP_OK) {}   /* alertas viejas */
    twai_message_t m = {.identifier = id, .ss = 1, .data_length_code = len};
    if (len) memcpy(m.data, d, len);
    if (twai_transmit(&m, ESPERA(20)) != ESP_OK) return B_NO_ENCOLADA;
    int64_t fin = esp_timer_get_time() + 100000;
    while (esp_timer_get_time() < fin) {
        if (twai_read_alerts(&a, ESPERA(10)) != ESP_OK) continue;
        if (a & TWAI_ALERT_TX_SUCCESS) return B_OK;
        if (a & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) return B_SIN_ACK;
    }
    return B_NO_SALIO;
}

/* Espera la respuesta de la pata n con el codigo dado. Descarta las demas tramas. */
static bool bus_respuesta(int n, uint8_t codigo, twai_message_t *r, int64_t espera_us)
{
    int64_t fin = esp_timer_get_time() + espera_us;
    while (esp_timer_get_time() < fin) {
        if (twai_receive(r, ESPERA(5)) != ESP_OK) continue;
        if (!r->extd && r->identifier == (uint32_t)ID_RESP(n) && r->data_length_code >= 1 &&
            r->data[0] == codigo)
            return true;
    }
    return false;
}

static void vaciar_rx(void)
{
    twai_message_t r;
    while (twai_receive(&r, 0) == ESP_OK) {}
}

/* Pregunta IDENT a las patas 1..6, 'veces' rondas. Con una ronda imprime cada pata. */
static void escanear(int veces)
{
    if (veces < 1) veces = 1;
    if (veces > 200) veces = 200;
    printf("\nESCANEO del bus: patas 1..6, %d ronda(s), solo IDENT (no mueve nada)\n", veces);
    if (!bus_iniciar()) return;
    int ok[7] = {0};
    uint8_t mac[7][6] = {{0}};
    int64_t us_max[7] = {0};
    for (int v = 0; v < veces && !bus_off(); v++) {
        for (int n = 1; n <= 6 && !bus_off(); n++) {
            vaciar_rx();
            uint8_t ord = 6;   /* IDENT */
            int64_t t0 = esp_timer_get_time();
            btx_t tx = bus_enviar(ID_ORDEN(n), &ord, 1);
            twai_message_t r;
            bool resp = tx == B_OK && bus_respuesta(n, 6, &r, 30000);
            int64_t us = esp_timer_get_time() - t0;
            if (resp) {
                ok[n]++;
                memcpy(mac[n], &r.data[1], 6);
                if (us > us_max[n]) us_max[n] = us;
            }
            if (veces == 1) {
                if (resp)
                    printf("SCAN,%d,OK,%02X%02X%02X%02X%02X%02X,us=%" PRId64 "\n", n, r.data[1], r.data[2],
                           r.data[3], r.data[4], r.data[5], r.data[6], us);
                else
                    printf("SCAN,%d,NO,%s\n", n, tx == B_OK ? "NO_RESPONDE" : txt_btx(tx));
            }
        }
    }
    if (bus_off()) puts("BUS,BUS_OFF: demasiados errores; escaneo cortado");
    int vistas = 0;
    for (int n = 1; n <= 6; n++) {
        if (ok[n]) vistas++;
        if (veces > 1)
            printf("SCANS,%d,ok=%d,de=%d,MAC=%02X%02X%02X%02X%02X%02X,us_max=%" PRId64 "\n", n, ok[n], veces,
                   mac[n][0], mac[n][1], mac[n][2], mac[n][3], mac[n][4], mac[n][5], us_max[n]);
    }
    printf("SCAN,fin,%d patas respondieron\n", vistas);
    bus_parar();
}

/* 'cuantos' PINGs a la pata n; corta con 20 fallos seguidos o BUS_OFF. */
static void rafaga(int n, int cuantos)
{
    if (n < 1 || n > 6) { puts("AVISO,pata 1..6"); return; }
    if (cuantos < 1) cuantos = 1000;
    if (cuantos > 20000) cuantos = 20000;
    printf("\nRAFAGA a la pata %d: %d pings (solo PING, no mueve nada)\n", n, cuantos);
    if (!bus_iniciar()) return;
    int env = 0, ok = 0, sin_ack = 0, no_salio = 0, sin_resp = 0, seguidos = 0;
    int64_t mn = INT64_MAX, mx = 0, suma = 0;
    for (int i = 0; i < cuantos; i++) {
        vaciar_rx();
        int64_t t0 = esp_timer_get_time();
        btx_t tx = bus_enviar(ID_ORDEN(n), NULL, 0);
        env++;
        twai_message_t r;
        if (tx == B_OK && bus_respuesta(n, 0, &r, 20000)) {
            int64_t us = esp_timer_get_time() - t0;
            ok++; seguidos = 0; suma += us;
            if (us < mn) mn = us;
            if (us > mx) mx = us;
        } else {
            if (tx == B_SIN_ACK) sin_ack++;
            else if (tx == B_OK) sin_resp++;
            else no_salio++;
            if (++seguidos >= 20 || bus_off()) { puts("RAFAGA,cortada: 20 fallos seguidos o BUS_OFF"); break; }
        }
        if ((i + 1) % 250 == 0) printf("RAFAGA,%d,progreso,%d/%d,ok=%d\n", n, i + 1, cuantos, ok);
    }
    printf("RAFAGA,%d,fin,enviados=%d,ok=%d,sin_ack=%d,no_salio=%d,sin_resp=%d,us_min=%" PRId64
           ",us_prom=%" PRId64 ",us_max=%" PRId64 "\n", n, env, ok, sin_ack, no_salio, sin_resp,
           ok ? mn : 0, ok ? suma / ok : 0, mx);
    bus_parar();
}

/* Salud del bus en vivo: la maestra pide ESTADO [03] a cada pata presente, 'hz' ciclos por
 * segundo, como lo hara el control del robot. Cada segundo imprime por pata las peticiones,
 * respuestas, fallos y latencia, y del bus TEC/REC, errores nuevos, BUS_OFF y carga estimada.
 * ESTADO solo lee la posicion: no arma ni mueve. Cualquier tecla lo detiene. */
static bool hay_tecla(void)
{
    char c;
    bool h = false;
    while (read(STDIN_FILENO, &c, 1) == 1) h = true;
    return h;
}

static bool pedir_ident(int n, int64_t espera_us)
{
    vaciar_rx();
    uint8_t ord = 6;
    twai_message_t r;
    return bus_enviar(ID_ORDEN(n), &ord, 1) == B_OK && bus_respuesta(n, 6, &r, espera_us);
}

typedef struct {
    uint32_t env, ok, sin_ack, sin_resp, no_salio;
    int64_t lat_suma, lat_max;
    uint32_t t_env, t_ok;
    int64_t t_lat_max;
} salud_t;

static void salud(int seg, int hz)
{
    if (seg <= 0 || seg > 3600) seg = 3600;
    if (hz < 1) hz = 1;
    if (hz > 200) hz = 200;
    printf("\nSALUD del bus: %d s, %d ciclos/s, ESTADO a cada pata (no mueve nada). Cualquier tecla detiene.\n",
           seg, hz);
    if (!bus_iniciar()) return;

    bool pres[7] = {false};
    printf("SALUD,patas,");
    for (int n = 1; n <= 6; n++) {
        pres[n] = pedir_ident(n, 20000);
        if (pres[n]) printf("%d ", n);
    }
    putchar('\n');

    salud_t s[7];
    memset(s, 0, sizeof s);
    uint32_t bus_offs = 0;
    twai_status_info_t st = {0};
    twai_get_status_info(&st);
    uint32_t err_prev = st.bus_error_count, err_tot = 0;
    const int64_t periodo = 1000000 / hz;
    int64_t ahora = esp_timer_get_time(), fin = ahora + (int64_t)seg * 1000000;
    int64_t prox = ahora, t_seg = ahora + 1000000;
    int segundo = 0;
    const uint8_t ord = 3;   /* ESTADO */

    while (esp_timer_get_time() < fin && !hay_tecla()) {
        for (int n = 1; n <= 6; n++) {
            if (!pres[n]) continue;
            salud_t *p = &s[n];
            vaciar_rx();
            int64_t t0 = esp_timer_get_time();
            btx_t tx = bus_enviar(ID_ORDEN(n), &ord, 1);
            twai_message_t r;
            p->env++;
            if (tx == B_OK && bus_respuesta(n, 5, &r, 10000)) {
                int64_t us = esp_timer_get_time() - t0;
                p->ok++;
                p->lat_suma += us;
                if (us > p->lat_max) p->lat_max = us;
            } else if (tx == B_OK) p->sin_resp++;
            else if (tx == B_SIN_ACK) p->sin_ack++;
            else p->no_salio++;
            if (bus_off()) {
                bus_offs++;
                printf("SALUD,BUS_OFF,%" PRIu32 ": reinicio el controlador\n", bus_offs);
                twai_driver_uninstall();
                s_bus = false;
                pines_gpio();
                vTaskDelay(ESPERA(100));
                if (!bus_iniciar()) { vTaskDelay(ESPERA(500)); if (!bus_iniciar()) goto salir; }
                twai_get_status_info(&st);
                err_prev = st.bus_error_count;
            }
        }

        ahora = esp_timer_get_time();
        if (ahora >= t_seg) {
            segundo++;
            uint32_t bits = 0;
            for (int n = 1; n <= 6; n++) {
                salud_t *p = &s[n];
                if (!pres[n]) {
                    /* Una vez por segundo busca las que faltan: una placa puede aparecer. */
                    if (pedir_ident(n, 3000)) { pres[n] = true; printf("SALUD,aparece,%d\n", n); }
                    continue;
                }
                printf("SALUD,t=%d,pata=%d,env=%" PRIu32 ",ok=%" PRIu32 ",sin_resp=%" PRIu32 ",sin_ack=%" PRIu32
                       ",no_salio=%" PRIu32 ",lat_prom=%" PRId64 ",lat_max=%" PRId64 "\n", segundo, n, p->env,
                       p->ok, p->sin_resp, p->sin_ack, p->no_salio, p->ok ? p->lat_suma / p->ok : 0, p->lat_max);
                /* Bits aproximados con relleno: orden de 1 byte ~64, respuesta de 8 bytes ~130. */
                bits += p->env * 64 + p->ok * 130;
                p->t_env += p->env;
                p->t_ok += p->ok;
                if (p->lat_max > p->t_lat_max) p->t_lat_max = p->lat_max;
                uint32_t te = p->t_env, tok = p->t_ok;
                int64_t tl = p->t_lat_max;
                memset(p, 0, sizeof *p);
                p->t_env = te; p->t_ok = tok; p->t_lat_max = tl;
            }
            twai_get_status_info(&st);
            uint32_t err_nuevos = st.bus_error_count - err_prev;
            err_prev = st.bus_error_count;
            err_tot += err_nuevos;
            printf("SALUD,t=%d,bus,TEC=%" PRIu32 ",REC=%" PRIu32 ",errores=%" PRIu32 ",bus_off=%" PRIu32
                   ",carga=%" PRIu32 "\n", segundo, st.tx_error_counter, st.rx_error_counter, err_nuevos,
                   bus_offs, bits / 5000);   /* % de 500 000 bit/s */
            t_seg += 1000000;
        }

        prox += periodo;
        int64_t falta = prox - esp_timer_get_time();
        if (falta > 0) vTaskDelay(ESPERA((falta + 999) / 1000));
        else prox = esp_timer_get_time();   /* no alcanza el ritmo pedido: sigue lo mas rapido posible */
    }
salir:
    for (int n = 1; n <= 6; n++) {
        if (!pres[n]) continue;
        salud_t *p = &s[n];
        uint32_t env = p->t_env + p->env, ok = p->t_ok + p->ok;
        printf("SALUD,fin,pata=%d,env=%" PRIu32 ",ok=%" PRIu32 ",perdidas=%" PRIu32 ",lat_max=%" PRId64 "\n", n,
               env, ok, env - ok, p->t_lat_max > p->lat_max ? p->t_lat_max : p->lat_max);
    }
    printf("SALUD,fin,bus,errores=%" PRIu32 ",bus_off=%" PRIu32 "\n", err_tot, bus_offs);
    if (s_instalado_bus()) bus_parar();
}

/* ---------------------------------------------------------------- info y consola */

static void info(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    esp_chip_info_t c;
    esp_chip_info(&c);
    printf("\nINFO,placa=%s,MAC=%02X%02X%02X%02X%02X%02X,nucleos=%d,rev=%d\n", PLACA, mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5], c.cores, c.revision);
    printf("INFO,CTX=GPIO%d,CRX=GPIO%d,500000 bit/s\n", PIN_CTX, PIN_CRX);

    /* Solo lectura: si la NVS no abre, no se borra (guarda la configuracion de la pata). */
    uint8_t id = 0;
    nvs_handle_t h;
    if (nvs_open("pata_can", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u8(h, "leg_id", &id) != ESP_OK) id = 0;
        nvs_close(h);
    }
    if (id) printf("INFO,ID de pata guardado en NVS = %u\n", id);
    else    puts("INFO,sin ID de pata en NVS (normal en la maestra)");
}

static void ayuda(void)
{
    puts("\nCOMANDOS (escribe y pulsa Enter):");
    puts("  todo     las pruebas 1-4 (y 5 en la MOD_DRV) con diagnostico final");
    puts("  ctx | eco | reposo | twai   una prueba suelta");
#if HAY_MOTORES
    puts("  motores  nFAULT y encoders (no mueve nada)");
    puts("  cero     encoders a 0");
#endif
    puts("  escucha  5 s en solo escucha (placa conectada al bus)");
    puts("BUS CON LAS PATAS (firmware pata_MOD_DRV; solo IDENT y PING, nunca arma):");
    puts("  escanear      que patas 1..6 responden, con su MAC");
    puts("  escanear 20   20 rondas seguidas: cuantas veces respondio cada una");
    puts("  rafaga 5      1000 pings a la pata 5 ('rafaga 5 5000' = 5000)");
    puts("  salud 60 100  60 s pidiendo ESTADO a todas, 100 ciclos/s; informe cada 1 s");
    puts("  m        monitor cada 1 s on/off");
    puts("  i        informacion de la placa (MAC, ID de pata)");
    puts("  h        esta ayuda");
}

static void procesar(const char *l)
{
    int a = 0, b = 0;
    if      (sscanf(l, "escanear %d", &a) == 1) escanear(a);
    else if (!strcmp(l, "escanear"))           escanear(1);
    else if (sscanf(l, "rafaga %d %d", &a, &b) == 2) rafaga(a, b);
    else if (sscanf(l, "rafaga %d", &a) == 1)  rafaga(a, 1000);
    else if (sscanf(l, "salud %d %d", &a, &b) == 2) salud(a, b);
    else if (sscanf(l, "salud %d", &a) == 1)   salud(a, 100);
    else if (!strcmp(l, "salud"))              salud(60, 100);
    else if (!strcmp(l, "todo") || !strcmp(l, "t")) todo();
    else if (!strcmp(l, "ctx"))     { prueba_ctx(); }
    else if (!strcmp(l, "eco"))     { prueba_eco(); }
    else if (!strcmp(l, "reposo"))  { prueba_reposo(); }
    else if (!strcmp(l, "twai"))    { prueba_twai(); }
    else if (!strcmp(l, "escucha") || !strcmp(l, "e")) escucha();
    else if (!strcmp(l, "m"))       { s_monitor = !s_monitor; printf("MONITOR,%d\n", s_monitor); }
    else if (!strcmp(l, "i"))       info();
    else if (!strcmp(l, "h") || !strcmp(l, "?")) ayuda();
#if HAY_MOTORES
    else if (!strcmp(l, "motores")) prueba_motores();
    else if (!strcmp(l, "cero"))    { for (int i = 0; i < 3; i++) s_cnt[i] = 0, s_saltos[i] = 0; puts("ENCODERS,0"); }
#endif
    else printf("AVISO,orden '%s' desconocida ('h' = ayuda)\n", l);
}

void app_main(void)
{
#if HAY_MOTORES
    motores_iniciar();   /* primero: los IN de los DRV8876 a 0 */
#endif
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t usb = {.rx_buffer_size = 256, .tx_buffer_size = 4096};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    usb_serial_jtag_vfs_use_driver();
#else
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(UART_NUM_0);
#endif
    int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (fl >= 0) fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);

    pines_gpio();
    if (nvs_flash_init() != ESP_OK) puts("AVISO,NVS no disponible (no se toca)");

    vTaskDelay(ESPERA(1500));   /* tiempo para abrir el monitor */
    puts("\n=== DIAGNOSTICO CAN DE UNA PLACA: " PLACA " ===");
    info();
    todo();
    ayuda();

    char linea[32];
    int n = 0;
    int64_t ultimo_mon = 0;
    for (;;) {
        char c;
        while (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\r' || c == '\n') {
                if (n) { linea[n] = '\0'; procesar(linea); }
                n = 0;
            } else if (n < (int)sizeof linea - 1) {
                linea[n++] = c;
            }
        }
        if (s_monitor && esp_timer_get_time() - ultimo_mon > 1000000) {
            ultimo_mon = esp_timer_get_time();
            monitor_linea();
        }
        vTaskDelay(ESPERA(10));
    }
}
