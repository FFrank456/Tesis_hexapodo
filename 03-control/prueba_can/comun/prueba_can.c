/*
 * Prueba CAN (TWAI) paso a paso: maestra ESP32-S3 <-> pata MOD_DRV.
 *
 * Protocolo: el mismo de pata_MOD_DRV/main/can_protocol.h, para que el ultimo
 * paso funcione igual contra el firmware real de la pata.
 *   0x0FF          maestra -> todas   PARO (dlc 0)          responde DETENIDA
 *   0x100 + pata   maestra -> pata    ping (dlc 0)          responde PONG
 *   0x200 + pata   pata -> maestra    respuesta (dlc 1: codigo)
 * Solo para esta prueba, la pata tambien puede enviar 0x200+pata con 4 bytes
 * (un contador) cuando se pulsa 'p' en su monitor.
 *
 * Las placas arrancan en modo ESCUCHA: no tocan el bus ni dan ACK hasta que
 * se pulse 'n'. Asi una placa con un problema no tumba el bus de la otra.
 */
#include "prueba_can.h"

#include <ctype.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "can_protocol.h"

#include "driver/twai.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ID_PARO          0x0FF
#define ID_ORDEN(n)      (0x100 + (n))
#define ID_RESPUESTA(n)  (0x200 + (n))
#define ID_AUTOTEST      0x7F0

#define N_RAFAGA         1000
#define ESPERA_TX_US     100000   /* tiempo maximo para que la trama salga */
#define ESPERA_RESP_US   50000    /* tiempo maximo para la respuesta de la pata */
#define PRINT_MIN_US     100000   /* como mucho una linea RX cada 100 ms */

/* Con FreeRTOS a 100 Hz, pdMS_TO_TICKS(5) da 0 y el bucle nunca cede la CPU
 * (salta el task watchdog). Toda espera dura al menos 1 tick. */
#define ESPERA(ms) (pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1)

/* Mismos codigos que pata_MOD_DRV/main/can_protocol.h */
enum { RESP_PONG = 0, RESP_ACEPTADA, RESP_DESARMADA, RESP_RECHAZADA, RESP_DETENIDA };

#define ALERTAS (TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF | \
                 TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL)

typedef enum { TX_OK, TX_SIN_ACK, TX_TIMEOUT, TX_NO_ENCOLADA } tx_res_t;

static const prueba_can_cfg_t *s_cfg;
static bool s_instalado;
static twai_mode_t s_modo = TWAI_MODE_LISTEN_ONLY;
static uint32_t s_seq;
static uint32_t s_tx_ok, s_tx_fallo, s_rx;
static int64_t s_ultimo_print;
static uint32_t s_omitidos;

/* Maestra: estado de la pata segun su ultimo ESTADO. */
#define LATIDO_US        100000    /* ESTADO cada 100 ms mientras la pata este armada */
#define POS_PRINT_US     500000    /* posicion cada 0.5 s mientras algo se mueve */
#define LLEGADA_MAX_US   15000000  /* aviso si no llega en 15 s */
static const char *const k_art[3] = {"coxa", "femur", "tibia"};
static bool s_armada;
static int64_t s_ultimo_latido, s_orden_t, s_ultimo_pos_print;
static uint8_t s_pendientes, s_flags, s_latidos_fallidos;
static int16_t s_pos_mrad[3];

/* ---------- textos ---------- */

static const char *nombre_modo(twai_mode_t m)
{
    switch (m) {
    case TWAI_MODE_NORMAL: return "NORMAL";
    case TWAI_MODE_NO_ACK: return "NO_ACK";
    default:               return "ESCUCHA";
    }
}

static const char *nombre_estado(twai_state_t e)
{
    switch (e) {
    case TWAI_STATE_STOPPED:    return "DETENIDO";
    case TWAI_STATE_RUNNING:    return "CORRIENDO";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECUPERANDO";
    default:                    return "?";
    }
}

static const char *nombre_tx(tx_res_t r)
{
    switch (r) {
    case TX_OK:          return "ACK_OK";
    case TX_SIN_ACK:     return "SIN_ACK";
    case TX_TIMEOUT:     return "NO_SALIO";
    default:             return "NO_ENCOLADA";
    }
}

static const char *nombre_respuesta(uint8_t c)
{
    switch (c) {
    case RESP_PONG:      return "PONG";
    case RESP_ACEPTADA:  return "ACEPTADA";
    case RESP_DESARMADA: return "DESARMADA";
    case RESP_RECHAZADA: return "RECHAZADA";
    case RESP_DETENIDA:  return "DETENIDA";
    case LEG_CAN_STATUS: return "ESTADO";
    default:             return "DESCONOCIDA";
    }
}

static const char *nombre_motivo(uint8_t m)
{
    switch (m) {
    case LEG_CAN_REASON_FORMAT:      return "formato de orden invalido";
    case LEG_CAN_REASON_RANGE:       return "angulo fuera del rango de la articulacion";
    case LEG_CAN_REASON_NFAULT:      return "el driver de esa articulacion esta en falla (nFAULT)";
    case LEG_CAN_REASON_NOT_STOPPED: return "la articulacion no esta parada";
    case LEG_CAN_REASON_ARMED:       return "la pata ya esta armada (desarma con 'd')";
    case LEG_CAN_REASON_CONTROL:     return "el control no arranco o cambio el cero del encoder";
    default:                         return "?";
    }
}

static void explicar_fallo_tx(tx_res_t r)
{
    switch (r) {
    case TX_SIN_ACK:
        puts("  Nadie confirmo la trama (ACK). Revisa: la otra placa en NORMAL ('n'),\n"
             "  CANH con CANH y CANL con CANL, GND comun, ~60 ohm entre CANH y CANL.");
        break;
    case TX_TIMEOUT:
        puts("  La trama no pudo salir en 100 ms: el bus parece pegado en dominante.\n"
             "  Mide CRX con 'r' y revisa que CANH y CANL no esten en corto o invertidos.");
        break;
    case TX_NO_ENCOLADA:
        puts("  El driver no acepto la trama: la placa esta en ESCUCHA o en BUS_OFF.");
        break;
    default: break;
    }
}

static void ayuda(void)
{
    puts("COMANDOS (escribe y pulsa Enter):");
    puts("  h  esta ayuda");
    puts("  s  estado del controlador y contadores");
    puts("  r  medir CRX en reposo durante 100 ms (debe quedar en 1)");
    puts("  t  autotest local: 10 tramas con autorrecepcion (NO_ACK)");
    puts("  n  modo NORMAL: transmite y da ACK");
    puts("  l  modo ESCUCHA: no toca el bus");
    puts("  z  contadores a cero");
    if (s_cfg->es_maestra) {
        puts("  p  ping a la pata (0x10x) y espera PONG (0x20x)");
        puts("  b  rafaga de 1000 pings: perdidas y tiempo de ida y vuelta");
        puts("  k  PARO general 0x0FF y espera DETENIDA");
        puts("MOVIMIENTO (con el firmware real pata_MOD_DRV; primero 'n'):");
        puts("  a            armar la pata (todas las articulaciones paradas)");
        puts("  f 0.5        femur a 0.5 rad      t -0.3   tibia a -0.3 rad");
        puts("  c 0.2        coxa a 0.2 rad       ft 0.3 -0.2   femur y tibia a la vez");
        puts("  e            estado: posiciones, nFAULT, armada");
        puts("  d            desarmar (suelta las articulaciones)");
        puts("  cero f|t|c|todo   el encoder vale 0 en la pose actual (desarmada)");
        puts("  Rangos: coxa -0.5..1.5, femur -1..1, tibia -1..0.7 rad (respecto del cero)");
    } else {
        puts("  p  envia una trama 0x20x con un contador hacia la maestra");
    }
}

/* ---------- controlador TWAI ---------- */

static bool iniciar(twai_mode_t modo)
{
    if (s_instalado) {
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_RUNNING) twai_stop();
        twai_driver_uninstall();
        s_instalado = false;
    }
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(s_cfg->pin_tx, s_cfg->pin_rx, modo);
    g.rx_queue_len = 32;
    g.alerts_enabled = ALERTAS;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t e = twai_driver_install(&g, &t, &f);
    if (e != ESP_OK) {
        printf("ERROR,instalar_twai,%s\n", esp_err_to_name(e));
        return false;
    }
    s_instalado = true;
    e = twai_start();
    if (e != ESP_OK) {
        printf("ERROR,arrancar_twai,%s\n", esp_err_to_name(e));
        return false;
    }
    s_modo = modo;
    printf("MODO,%s\n", nombre_modo(modo));
    return true;
}

static void estado(void)
{
    twai_status_info_t st;
    if (!s_instalado || twai_get_status_info(&st) != ESP_OK) {
        puts("ESTADO,driver_no_instalado");
        return;
    }
    puts("ESTADO");
    printf("  modo          %s\n", nombre_modo(s_modo));
    printf("  controlador   %s\n", nombre_estado(st.state));
    printf("  CRX ahora     %d   (en reposo debe ser 1)\n", gpio_get_level(s_cfg->pin_rx));
    printf("  TEC / REC     %" PRIu32 " / %" PRIu32 "   (errores de TX / RX; 0 es lo normal)\n",
           st.tx_error_counter, st.rx_error_counter);
    printf("  errores bus   %" PRIu32 "\n", st.bus_error_count);
    printf("  rx perdidas   %" PRIu32 "\n", st.rx_missed_count);
    printf("  tx ok/fallo   %" PRIu32 " / %" PRIu32 "\n", s_tx_ok, s_tx_fallo);
    printf("  rx            %" PRIu32 "\n", s_rx);
}

/* Con el bus en reposo el transceptor entrega CRX = 1 (recesivo). */
static void medir_crx(void)
{
    const int n = 2000;
    int dominantes = 0;
    for (int i = 0; i < n; i++) {
        if (gpio_get_level(s_cfg->pin_rx) == 0) dominantes++;
        esp_rom_delay_us(50);
    }
    printf("CRX,muestras=%d,en_0=%d -> ", n, dominantes);
    if (dominantes == 0)      puts("OK, reposo en 1");
    else if (dominantes == n) puts("FALLA, CRX pegado en 0 (modulo sin 3,3 V, CTX/CRX cruzados\n"
                                   "  u otro nodo forzando el bus a dominante)");
    else                      puts("hay actividad o ruido en el bus (repite con la otra placa callada)");
}

static void procesar_alertas(uint32_t a)
{
    if (a & TWAI_ALERT_ERR_PASS) {
        /* El driver puede dar esta alerta al reiniciarse con TEC = REC = 0: se confirma. */
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK &&
            (st.tx_error_counter > 127 || st.rx_error_counter > 127))
            printf("AVISO,error_pasivo: TEC=%" PRIu32 " REC=%" PRIu32 " (pasaron de 127)\n",
                   st.tx_error_counter, st.rx_error_counter);
    }
    if (a & TWAI_ALERT_RX_QUEUE_FULL) puts("AVISO,cola_rx_llena");
    if (a & TWAI_ALERT_BUS_OFF) {
        puts("ERROR,BUS_OFF: demasiadas transmisiones fallidas. Vuelvo a ESCUCHA.");
        iniciar(TWAI_MODE_LISTEN_ONLY);
    }
}

/* Envia una trama en un solo intento (sin reintentos) y espera el resultado. */
static tx_res_t enviar(uint32_t id, const uint8_t *datos, uint8_t len, bool autorrecepcion)
{
    uint32_t a;
    if (twai_read_alerts(&a, 0) == ESP_OK) procesar_alertas(a);   /* descarta alertas viejas */

    twai_message_t m = {.identifier = id, .ss = 1, .self = autorrecepcion ? 1 : 0,
                        .data_length_code = len};
    if (len) memcpy(m.data, datos, len);
    esp_err_t e = twai_transmit(&m, ESPERA(20));
    if (e != ESP_OK) {
        s_tx_fallo++;
        return TX_NO_ENCOLADA;
    }
    int64_t limite = esp_timer_get_time() + ESPERA_TX_US;
    while (esp_timer_get_time() < limite) {
        if (twai_read_alerts(&a, ESPERA(10)) != ESP_OK) continue;
        bool exito = a & TWAI_ALERT_TX_SUCCESS;
        bool fallo = (a & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) != 0;
        procesar_alertas(a);
        if (exito) { s_tx_ok++; return TX_OK; }
        if (fallo) { s_tx_fallo++; return TX_SIN_ACK; }
    }
    s_tx_fallo++;
    return TX_TIMEOUT;
}

/* ---------- recepcion ---------- */

/* Limita la impresion para que una rafaga no bloquee la consola. */
static bool puede_imprimir(void)
{
    int64_t ahora = esp_timer_get_time();
    if (ahora - s_ultimo_print < PRINT_MIN_US) {
        s_omitidos++;
        return false;
    }
    s_ultimo_print = ahora;
    if (s_omitidos) {
        printf("RX,(%" PRIu32 " tramas mas sin imprimir)\n", s_omitidos);
        s_omitidos = 0;
    }
    return true;
}

static void imprimir_trama(const char *prefijo, const twai_message_t *f)
{
    printf("%s,id=0x%03" PRIX32 ",dlc=%d,datos=", prefijo, f->identifier, f->data_length_code);
    for (int i = 0; i < f->data_length_code && i < 8; i++) printf("%02X ", f->data[i]);
    putchar('\n');
}

static void responder(uint8_t codigo)
{
    if (s_modo != TWAI_MODE_NORMAL) return;
    twai_message_t r = {.identifier = ID_RESPUESTA(s_cfg->pata_id), .ss = 1,
                        .data_length_code = 1};
    r.data[0] = codigo;
    if (twai_transmit(&r, 0) == ESP_OK) s_tx_ok++;
    else s_tx_fallo++;
}

static void procesar_rx(const twai_message_t *f)
{
    s_rx++;
    bool normal = s_modo == TWAI_MODE_NORMAL;
    if (!f->extd && !f->rtr && !s_cfg->es_maestra) {
        int codigo = -1;
        if (f->identifier == ID_ORDEN(s_cfg->pata_id))
            codigo = f->data_length_code == 0 ? RESP_PONG : RESP_RECHAZADA;
        else if (f->identifier == ID_PARO && f->data_length_code == 0)
            codigo = RESP_DETENIDA;
        if (codigo >= 0) {
            responder((uint8_t)codigo);
            if (puede_imprimir()) {
                imprimir_trama("RX", f);
                if (normal) printf("  -> respondo %s en 0x%03X\n", nombre_respuesta(codigo),
                                   ID_RESPUESTA(s_cfg->pata_id));
                else puts("  -> en ESCUCHA no respondo (pulsa 'n')");
            }
            return;
        }
    }
    if (!puede_imprimir()) return;
    imprimir_trama("RX", f);
    if (s_cfg->es_maestra && f->identifier == ID_RESPUESTA(s_cfg->pata_id) && f->data_length_code == 1)
        printf("  -> respuesta %s\n", nombre_respuesta(f->data[0]));
}

/* La maestra espera la respuesta de su pata (trama completa); las demas tramas
 * se procesan normal. */
static bool esperar_trama(twai_message_t *out, int64_t *t_llegada)
{
    int64_t limite = esp_timer_get_time() + ESPERA_RESP_US;
    twai_message_t f;
    while (esp_timer_get_time() < limite) {
        if (twai_receive(&f, ESPERA(5)) != ESP_OK) continue;
        if (!f.extd && !f.rtr && f.identifier == ID_RESPUESTA(s_cfg->pata_id) &&
            f.data_length_code >= 1) {
            *t_llegada = esp_timer_get_time();
            *out = f;
            s_rx++;
            return true;
        }
        procesar_rx(&f);
    }
    return false;
}

static bool esperar_respuesta(uint8_t *codigo, int64_t *t_llegada)
{
    twai_message_t f;
    if (!esperar_trama(&f, t_llegada)) return false;
    *codigo = f.data[0];
    return true;
}

/* ---------- pruebas ---------- */

static bool requiere_normal(void)
{
    if (s_modo == TWAI_MODE_NORMAL) return true;
    puts("AVISO,primero pon esta placa en NORMAL con 'n'");
    return false;
}

static void autotest(void)
{
    twai_mode_t previo = s_modo;
    puts("AUTOTEST,10 tramas en NO_ACK con autorrecepcion (sale por CTX y vuelve por CRX)");
    if (!iniciar(TWAI_MODE_NO_ACK)) return;
    int bien = 0;
    for (int i = 0; i < 10; i++) {
        uint32_t valor = 0xCA000000u | (uint32_t)i;
        uint8_t d[4];
        memcpy(d, &valor, 4);
        tx_res_t r = enviar(ID_AUTOTEST, d, 4, true);
        bool eco = false;
        twai_message_t rx;
        for (int k = 0; r == TX_OK && k < 4 && !eco; k++) {
            if (twai_receive(&rx, ESPERA(50)) != ESP_OK) break;
            eco = !rx.extd && rx.identifier == ID_AUTOTEST && rx.data_length_code == 4 &&
                  memcmp(rx.data, d, 4) == 0;
        }
        if (eco) bien++;
        printf("AUTOTEST,trama=%d,tx=%s,eco=%s\n", i + 1, nombre_tx(r), eco ? "OK" : "NO");
    }
    printf("AUTOTEST,%s,%d/10\n", bien == 10 ? "PASA" : "FALLA", bien);
    estado();
    iniciar(previo == TWAI_MODE_NO_ACK ? TWAI_MODE_LISTEN_ONLY : previo);
}

static void ping(void)
{
    if (!requiere_normal()) return;
    uint32_t id = ID_ORDEN(s_cfg->pata_id);
    int64_t t0 = esp_timer_get_time();
    tx_res_t r = enviar(id, NULL, 0, false);
    printf("TX,id=0x%03" PRIX32 ",dlc=0 (ping) -> %s\n", id, nombre_tx(r));
    if (r != TX_OK) {
        explicar_fallo_tx(r);
        puts("FALLA,ping");
        return;
    }
    uint8_t codigo;
    int64_t t1;
    if (esperar_respuesta(&codigo, &t1)) {
        printf("RESPUESTA,id=0x%03X,%s,ida_y_vuelta_us=%lld\n", ID_RESPUESTA(s_cfg->pata_id),
               nombre_respuesta(codigo), (long long)(t1 - t0));
        puts(codigo == RESP_PONG ? "PASA,ping" : "FALLA,ping (respuesta distinta de PONG)");
    } else {
        puts("FALLA,ping: hubo ACK pero la pata no respondio en 50 ms\n"
             "  (esta en ESCUCHA? tiene otro PATA_ID?)");
    }
}

static void paro(void)
{
    s_armada = false;
    s_pendientes = 0;
    if (!requiere_normal()) return;
    tx_res_t r = enviar(ID_PARO, NULL, 0, false);
    printf("TX,id=0x%03X,dlc=0 (PARO) -> %s\n", ID_PARO, nombre_tx(r));
    if (r != TX_OK) { explicar_fallo_tx(r); return; }
    uint8_t codigo;
    int64_t t1;
    if (esperar_respuesta(&codigo, &t1))
        printf("RESPUESTA,id=0x%03X,%s\n", ID_RESPUESTA(s_cfg->pata_id), nombre_respuesta(codigo));
    else
        puts("FALLA,PARO sin respuesta de la pata");
}

static void rafaga(void)
{
    if (!requiere_normal()) return;
    int ok = 0, sin_ack = 0, sin_resp = 0, enviados = 0;
    int64_t suma = 0, minimo = INT64_MAX, maximo = 0;
    twai_clear_receive_queue();
    printf("RAFAGA,inicio,%d pings uno tras otro\n", N_RAFAGA);
    for (int i = 0; i < N_RAFAGA; i++) {
        enviados++;
        int64_t t0 = esp_timer_get_time();
        tx_res_t r = enviar(ID_ORDEN(s_cfg->pata_id), NULL, 0, false);
        uint8_t codigo;
        int64_t t1;
        if (r != TX_OK) sin_ack++;
        else if (esperar_respuesta(&codigo, &t1) && codigo == RESP_PONG) {
            ok++;
            int64_t d = t1 - t0;
            suma += d;
            if (d < minimo) minimo = d;
            if (d > maximo) maximo = d;
        } else sin_resp++;
        if ((i + 1) % 100 == 0) printf("RAFAGA,%d/%d,ok=%d\n", i + 1, N_RAFAGA, ok);
        if (s_modo != TWAI_MODE_NORMAL || sin_ack + sin_resp > 20) {
            puts("RAFAGA,abortada: mas de 20 fallos o BUS_OFF");
            break;
        }
    }
    printf("RAFAGA,fin,enviados=%d,ok=%d,sin_ack=%d,sin_respuesta=%d", enviados, ok, sin_ack, sin_resp);
    if (ok) printf(",ida_y_vuelta_us_min=%lld,prom=%lld,max=%lld", (long long)minimo,
                   (long long)(suma / ok), (long long)maximo);
    putchar('\n');
    puts(ok == N_RAFAGA ? "PASA,rafaga" : "FALLA,rafaga");
    estado();
}

/* ---------- maestra: ordenes de movimiento (protocolo v2, can_protocol.h) ---------- */

/* Envia una orden a la pata y devuelve su respuesta completa. */
static bool orden(const uint8_t *d, uint8_t len, twai_message_t *resp)
{
    if (!requiere_normal()) return false;
    tx_res_t r = enviar(ID_ORDEN(s_cfg->pata_id), d, len, false);
    if (r != TX_OK) {
        printf("TX -> %s\n", nombre_tx(r));
        explicar_fallo_tx(r);
        return false;
    }
    int64_t t1;
    if (!esperar_trama(resp, &t1)) {
        puts("FALLA,la pata no respondio en 50 ms");
        return false;
    }
    return true;
}

static void imprimir_rechazo(const twai_message_t *r)
{
    if (r->data[0] == LEG_CAN_REJECTED)
        printf("  -> RECHAZADA: %s\n", r->data_length_code >= 2 ? nombre_motivo(r->data[1]) : "?");
    else
        printf("  -> %s\n", nombre_respuesta(r->data[0]));
}

static bool leer_estado(void)
{
    uint8_t d[1] = {LEG_CAN_CMD_STATUS};
    twai_message_t r;
    if (enviar(ID_ORDEN(s_cfg->pata_id), d, 1, false) != TX_OK) return false;
    int64_t t1;
    if (!esperar_trama(&r, &t1) || r.data[0] != LEG_CAN_STATUS || r.data_length_code != LEG_CAN_STATUS_DLC)
        return false;
    s_flags = r.data[1];
    for (int i = 0; i < 3; i++) s_pos_mrad[i] = (int16_t)(r.data[2 + 2*i] | (r.data[3 + 2*i] << 8));
    return true;
}

static void imprimir_posiciones(const char *prefijo)
{
    printf("%s,coxa=%+.3f,femur=%+.3f,tibia=%+.3f rad", prefijo,
           s_pos_mrad[0] / 1000.0, s_pos_mrad[1] / 1000.0, s_pos_mrad[2] / 1000.0);
    for (int i = 0; i < 3; i++)
        if (s_flags & (1U << (LEG_CAN_STATUS_NFAULT_SHIFT + i))) printf(",nFAULT_%s", k_art[i]);
    printf(",%s\n", (s_flags & LEG_CAN_STATUS_ARMED_BIT) ? "armada" : "desarmada");
}

static void estado_pata(void)
{
    if (!requiere_normal()) return;
    if (leer_estado()) imprimir_posiciones("ESTADO");
    else puts("FALLA,la pata no respondio a ESTADO");
}

static void armar(void)
{
    uint8_t d[1] = {LEG_CAN_CMD_ARM};
    twai_message_t r;
    if (!orden(d, 1, &r)) return;
    if (r.data[0] == LEG_CAN_ACCEPTED) {
        s_armada = true;
        s_pendientes = 0;
        s_latidos_fallidos = 0;
        s_ultimo_latido = esp_timer_get_time();
        puts("ARMADA: la pata acepta movimientos. Mando ESTADO cada 100 ms como latido;");
        puts("  si la maestra se calla 0,5 s, la pata suelta las articulaciones.");
        if (leer_estado()) imprimir_posiciones("ESTADO");
    } else {
        imprimir_rechazo(&r);
    }
}

static void desarmar(void)
{
    uint8_t d[1] = {LEG_CAN_CMD_DISARM};
    twai_message_t r;
    s_armada = false;
    s_pendientes = 0;
    if (orden(d, 1, &r)) printf("DESARMAR -> %s (articulaciones sueltas)\n", nombre_respuesta(r.data[0]));
}

/* mask: bit0 coxa, bit1 femur, bit2 tibia. rad[i] solo cuenta si su bit esta en mask. */
static void ir_a(uint8_t mask, const float rad[3])
{
    uint8_t d[LEG_CAN_GOTO_DLC] = {LEG_CAN_CMD_GOTO, mask};
    printf("IR_A");
    for (int i = 0; i < 3; i++) {
        if (!(mask & (1U << i))) continue;
        long mrad = lroundf(rad[i] * 1000.0f);
        if (mrad > 32767) mrad = 32767;
        if (mrad < -32768) mrad = -32768;
        d[2 + 2*i] = (uint8_t)(mrad & 0xFF);
        d[3 + 2*i] = (uint8_t)(((uint16_t)(int16_t)mrad) >> 8);
        printf(",%s=%+.3f", k_art[i], rad[i]);
    }
    puts(" rad");
    twai_message_t r;
    if (!orden(d, sizeof d, &r)) return;
    if (r.data[0] == LEG_CAN_ACCEPTED) {
        puts("  -> ACEPTADA, en camino");
        s_pendientes |= mask;
        s_orden_t = s_ultimo_pos_print = esp_timer_get_time();
    } else {
        imprimir_rechazo(&r);
        if (r.data[0] == LEG_CAN_DISARMED) puts("  (primero arma la pata con 'a')");
    }
}

static void cero(uint8_t mask)
{
    uint8_t d[2] = {LEG_CAN_CMD_ZERO, mask};
    twai_message_t r;
    if (!orden(d, 2, &r)) return;
    if (r.data[0] == LEG_CAN_ACCEPTED) {
        printf("CERO -> ACEPTADO:");
        for (int i = 0; i < 3; i++) if (mask & (1U << i)) printf(" %s", k_art[i]);
        puts(" valen 0 en la pose actual");
    } else imprimir_rechazo(&r);
}

/* Latido: con la pata armada, ESTADO cada 100 ms. Avisa llegadas y fallas. */
static void latido(void)
{
    if (!s_armada || s_modo != TWAI_MODE_NORMAL) return;
    int64_t ahora = esp_timer_get_time();
    if (ahora - s_ultimo_latido < LATIDO_US) return;
    s_ultimo_latido = ahora;
    if (!leer_estado()) {
        if (++s_latidos_fallidos == 3) puts("AVISO,la pata no responde al latido (3 seguidos)");
        return;
    }
    s_latidos_fallidos = 0;
    if (!(s_flags & LEG_CAN_STATUS_ARMED_BIT)) {
        s_armada = false;
        s_pendientes = 0;
        imprimir_posiciones("AVISO,la pata se desarmo (PARO, timeout, STOP por serie o falla)");
        return;
    }
    for (int i = 0; i < 3; i++) {
        uint8_t b = (uint8_t)(1U << i);
        if (!(s_pendientes & b)) continue;
        if (s_flags & (1U << (LEG_CAN_STATUS_NFAULT_SHIFT + i))) {
            printf("AVISO,nFAULT en %s\n", k_art[i]);
            s_pendientes &= (uint8_t)~b;
        } else if (ahora - s_orden_t > 200000 && (s_flags & (1U << (LEG_CAN_STATUS_ARRIVED_SHIFT + i)))) {
            printf("LLEGO,%s,%+.3f rad,%.1f s\n", k_art[i], s_pos_mrad[i] / 1000.0,
                   (ahora - s_orden_t) / 1e6);
            s_pendientes &= (uint8_t)~b;
        } else if (ahora - s_orden_t > LLEGADA_MAX_US) {
            printf("AVISO,%s no llego en 15 s (esta en %+.3f rad)\n", k_art[i], s_pos_mrad[i] / 1000.0);
            s_pendientes &= (uint8_t)~b;
        }
    }
    if (s_pendientes && ahora - s_ultimo_pos_print >= POS_PRINT_US) {
        s_ultimo_pos_print = ahora;
        imprimir_posiciones("POS");
    }
}

static void enviar_contador(void)
{
    if (!requiere_normal()) return;
    uint32_t valor = ++s_seq;
    uint8_t d[4];
    memcpy(d, &valor, 4);
    uint32_t id = ID_RESPUESTA(s_cfg->pata_id);
    tx_res_t r = enviar(id, d, 4, false);
    printf("TX,id=0x%03" PRIX32 ",dlc=4,contador=%" PRIu32 " -> %s\n", id, valor, nombre_tx(r));
    if (r != TX_OK) explicar_fallo_tx(r);
    else puts("PASA,la maestra dio ACK (en su monitor debe aparecer la linea RX)");
}

static void comando(char c)
{
    switch (tolower((unsigned char)c)) {
    case 'h': ayuda(); break;
    case 's': estado(); break;
    case 'r': medir_crx(); break;
    case 't': autotest(); break;
    case 'n': iniciar(TWAI_MODE_NORMAL); break;
    case 'l': iniciar(TWAI_MODE_LISTEN_ONLY); break;
    case 'z': s_tx_ok = s_tx_fallo = s_rx = 0; puts("CONTADORES,0"); break;
    case 'p': if (s_cfg->es_maestra) ping(); else enviar_contador(); break;
    case 'b': if (s_cfg->es_maestra) rafaga(); else puts("AVISO,'b' es solo de la maestra"); break;
    case 'k': if (s_cfg->es_maestra) paro(); else puts("AVISO,'k' es solo de la maestra"); break;
    case 'a': if (s_cfg->es_maestra) armar(); else puts("AVISO,'a' es solo de la maestra"); break;
    case 'd': if (s_cfg->es_maestra) desarmar(); else puts("AVISO,'d' es solo de la maestra"); break;
    case 'e': if (s_cfg->es_maestra) estado_pata(); else puts("AVISO,'e' es solo de la maestra"); break;
    case '\r': case '\n': case ' ': break;
    default: printf("AVISO,comando '%c' desconocido; 'h' para ayuda\n", c); break;
    }
}

/* "f 0.5" femur, "t -0.3" tibia, "c 0.2" coxa, "ft 0.3 -0.2" femur y tibia,
 * "cero f|t|c|todo"; una sola letra = comando de prueba. */
static int articulacion(const char *w)
{
    if (!strcmp(w, "c") || !strcmp(w, "coxa")) return 0;
    if (!strcmp(w, "f") || !strcmp(w, "femur")) return 1;
    if (!strcmp(w, "t") || !strcmp(w, "tibia")) return 2;
    return -1;
}

static bool leer_rad(const char *s, float *v)
{
    char *fin = NULL;
    if (!s) return false;
    *v = strtof(s, &fin);
    return fin != s && *fin == '\0' && isfinite(*v);
}

static void procesar_linea(char *l)
{
    for (char *p = l; *p; p++) *p = (char)tolower((unsigned char)*p);
    printf(">> %s\n", l);
    char *w = strtok(l, " ,"), *x = strtok(NULL, " ,"), *y = strtok(NULL, " ,");
    if (!w) return;
    if (!x && strlen(w) == 1) { comando(w[0]); return; }
    if (!s_cfg->es_maestra) { puts("AVISO,la pata solo acepta comandos de una letra"); return; }

    float rad[3] = {0, 0, 0};
    int j = articulacion(w);
    if (!strcmp(w, "armar")) armar();
    else if (!strcmp(w, "desarmar")) desarmar();
    else if (!strcmp(w, "estado")) estado_pata();
    else if (!strcmp(w, "cero")) {
        int k = x ? articulacion(x) : -1;
        if (!x || !strcmp(x, "todo")) cero(LEG_CAN_AXIS_MASK);
        else if (k >= 0) cero((uint8_t)(1U << k));
        else puts("AVISO,uso: cero f | cero t | cero c | cero todo");
    } else if (!strcmp(w, "ft")) {
        if (leer_rad(x, &rad[1]) && leer_rad(y, &rad[2])) ir_a(0x06, rad);
        else puts("AVISO,uso: ft <femur_rad> <tibia_rad>");
    } else if (j >= 0) {
        if (leer_rad(x, &rad[j]) && !y) ir_a((uint8_t)(1U << j), rad);
        else printf("AVISO,uso: %s <angulo_rad>, por ejemplo %s 0.5\n", w, w);
    } else printf("AVISO,orden '%s' desconocida; 'h' para ayuda\n", w);
}

void prueba_can_ejecutar(const prueba_can_cfg_t *cfg)
{
    s_cfg = cfg;
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    printf("\n=== PRUEBA CAN: %s ===\n", cfg->nombre);
    printf("PLACA,TX=GPIO%d->CTX,RX=GPIO%d<-CRX,500000 bit/s,pata_id=%d,rol=%s\n",
           cfg->pin_tx, cfg->pin_rx, cfg->pata_id, cfg->es_maestra ? "MAESTRA" : "PATA");
    iniciar(TWAI_MODE_LISTEN_ONLY);
    medir_crx();
    estado();
    ayuda();

    char linea[48];
    size_t n = 0;
    while (true) {
        twai_message_t f;
        if (s_instalado && twai_receive(&f, ESPERA(5)) == ESP_OK) procesar_rx(&f);
        else if (!s_instalado) vTaskDelay(ESPERA(5));

        uint32_t a;
        if (s_instalado && twai_read_alerts(&a, 0) == ESP_OK) procesar_alertas(a);
        if (s_cfg->es_maestra) latido();

        /* Las ordenes se escriben y se confirman con Enter. */
        char c;
        for (int i = 0; i < 16 && read(STDIN_FILENO, &c, 1) == 1; i++) {
            if (c == '\r' || c == '\n') {
                if (n) { linea[n] = '\0'; putchar('\n'); procesar_linea(linea); n = 0; }
            } else if ((c == '\b' || c == 0x7F) && n) {
                n--; fputs("\b \b", stdout);
            } else if (isprint((unsigned char)c) && n < sizeof linea - 1) {
                linea[n++] = c; putchar(c);   /* eco: el monitor no muestra lo que se escribe */
            }
            fflush(stdout);
        }
    }
}
