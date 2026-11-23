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
static bool s_stream;   /* 'v 1': linea EST cada 100 ms para la interfaz web */
static int64_t s_ultimo_latido;
typedef struct {
    bool armada;       /* la maestra la armo: le manda latido */
    bool vista;        /* respondio hace poco */
    bool duplicada;    /* dos placas con este ID en el ultimo escaneo */
    uint8_t pendientes, flags, fallos;
    int64_t orden_t[3], ultimo_pos;
    int16_t pos_mrad[3];
} pata_t;
static pata_t s_patas[LEG_CAN_MAX_LEGS + 1];   /* indice = ID de pata, 1..6 */
static uint8_t s_sel = 1;                      /* pata elegida en la maestra */

/* Pata a la que habla esta placa: la elegida (maestra) o ella misma (pata). */
static int pata_actual(void) { return s_cfg->es_maestra ? s_sel : s_cfg->pata_id; }
static bool alguna_armada(void)
{
    for (int p = 1; p <= LEG_CAN_MAX_LEGS; p++) if (s_patas[p].armada) return true;
    return false;
}

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
        puts("  escanear     que patas (1..6) responden y si hay IDs repetidos");
        puts("  p3           elegir la pata 3;  'p3 f 0.5' la elige y manda la orden");
        puts("  p  ping a la pata elegida (0x10x) y espera PONG (0x20x)");
        puts("  b  rafaga de 1000 pings: perdidas y tiempo de ida y vuelta");
        puts("  k  PARO general 0x0FF y espera DETENIDA");
        puts("MOVIMIENTO (con el firmware real pata_MOD_DRV; primero 'n'):");
        puts("  a            armar la pata (todas las articulaciones paradas)");
        puts("  f 0.5        femur a 0.5 rad      t -0.3   tibia a -0.3 rad");
        puts("  c 0.2        coxa a 0.2 rad       ft 0.3 -0.2   femur y tibia a la vez");
        puts("  e            estado: posiciones, nFAULT, armada");
        puts("  v 1 / v 0    linea EST cada 100 ms (la usa la interfaz web)");
        puts("  d            desarmar (suelta las articulaciones);  'd todas' = las 6");
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
    twai_message_t r = {.identifier = ID_RESPUESTA(pata_actual()), .ss = 1,
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
        if (f->identifier == ID_ORDEN(pata_actual()))
            codigo = f->data_length_code == 0 ? RESP_PONG : RESP_RECHAZADA;
        else if (f->identifier == ID_PARO && f->data_length_code == 0)
            codigo = RESP_DETENIDA;
        if (codigo >= 0) {
            responder((uint8_t)codigo);
            if (puede_imprimir()) {
                imprimir_trama("RX", f);
                if (normal) printf("  -> respondo %s en 0x%03X\n", nombre_respuesta(codigo),
                                   ID_RESPUESTA(pata_actual()));
                else puts("  -> en ESCUCHA no respondo (pulsa 'n')");
            }
            return;
        }
    }
    if (!puede_imprimir()) return;
    imprimir_trama("RX", f);
    if (s_cfg->es_maestra && f->identifier == ID_RESPUESTA(pata_actual()) && f->data_length_code == 1)
        printf("  -> respuesta %s\n", nombre_respuesta(f->data[0]));
}

/* Tira las tramas viejas: una respuesta tardia no debe pasar por la siguiente. */
static void vaciar_rx(void)
{
    twai_message_t f;
    while (twai_receive(&f, 0) == ESP_OK) procesar_rx(&f);
}

/* La maestra espera la respuesta de su pata (trama completa); las demas tramas
 * se procesan normal. */
static bool esperar_trama(int pata, twai_message_t *out, int64_t *t_llegada)
{
    int64_t limite = esp_timer_get_time() + ESPERA_RESP_US;
    twai_message_t f;
    while (esp_timer_get_time() < limite) {
        if (twai_receive(&f, ESPERA(5)) != ESP_OK) continue;
        if (!f.extd && !f.rtr && f.identifier == (uint32_t)ID_RESPUESTA(pata) &&
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
    if (!esperar_trama(pata_actual(), &f, t_llegada)) return false;
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
    uint32_t id = ID_ORDEN(pata_actual());
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
        printf("RESPUESTA,id=0x%03X,%s,ida_y_vuelta_us=%lld\n", ID_RESPUESTA(pata_actual()),
               nombre_respuesta(codigo), (long long)(t1 - t0));
        puts(codigo == RESP_PONG ? "PASA,ping" : "FALLA,ping (respuesta distinta de PONG)");
    } else {
        puts("FALLA,ping: hubo ACK pero la pata no respondio en 50 ms\n"
             "  (esta en ESCUCHA? tiene otro PATA_ID?)");
    }
}

static void paro(void)
{
    for (int p = 1; p <= LEG_CAN_MAX_LEGS; p++) { s_patas[p].armada = false; s_patas[p].pendientes = 0; }
    if (!requiere_normal()) return;
    vaciar_rx();
    tx_res_t r = TX_SIN_ACK;
    for (int i = 0; i < 3 && r != TX_OK; i++) r = enviar(ID_PARO, NULL, 0, false);
    printf("TX,id=0x%03X,dlc=0 (PARO) -> %s\n", ID_PARO, nombre_tx(r));
    if (r != TX_OK) { explicar_fallo_tx(r); return; }
    uint8_t detenidas = 0;
    twai_message_t f;
    int64_t limite = esp_timer_get_time() + ESPERA_RESP_US;
    while (esp_timer_get_time() < limite) {
        if (twai_receive(&f, ESPERA(5)) != ESP_OK) continue;
        int p = (int)f.identifier - ID_RESPUESTA(0);
        if (!f.extd && !f.rtr && p >= 1 && p <= LEG_CAN_MAX_LEGS && f.data_length_code >= 1 &&
            f.data[0] == RESP_DETENIDA) {
            detenidas |= (uint8_t)(1U << p);
            s_rx++;
        }
    }
    for (int p = 1; p <= LEG_CAN_MAX_LEGS; p++) {
        if (detenidas & (1U << p)) printf("RESPUESTA,id=0x%03X,pata=%d,DETENIDA\n", ID_RESPUESTA(p), p);
        else if (s_patas[p].vista) printf("AVISO,la pata %d no confirmo el PARO\n", p);
    }
    if (!detenidas) puts("FALLA,PARO sin respuesta de ninguna pata");
}

static void rafaga(void)
{
    if (!requiere_normal()) return;
    if (alguna_armada()) { puts("AVISO,desarma las patas antes de la rafaga ('d todas')"); return; }
    int ok = 0, sin_ack = 0, sin_resp = 0, enviados = 0;
    int64_t suma = 0, minimo = INT64_MAX, maximo = 0;
    twai_clear_receive_queue();
    printf("RAFAGA,inicio,%d pings uno tras otro\n", N_RAFAGA);
    for (int i = 0; i < N_RAFAGA; i++) {
        enviados++;
        int64_t t0 = esp_timer_get_time();
        tx_res_t r = enviar(ID_ORDEN(pata_actual()), NULL, 0, false);
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

/* Envia una orden a la pata elegida y devuelve su respuesta completa. */
static bool orden(const uint8_t *d, uint8_t len, twai_message_t *resp)
{
    if (!requiere_normal()) return false;
    vaciar_rx();
    tx_res_t r = enviar(ID_ORDEN(s_sel), d, len, false);
    if (r != TX_OK) {
        printf("FALLA,pata=%d,TX -> %s\n", s_sel, nombre_tx(r));
        explicar_fallo_tx(r);
        return false;
    }
    int64_t t1;
    if (!esperar_trama(s_sel, resp, &t1)) {
        printf("FALLA,la pata %d no respondio en 50 ms (revisa su ID con CAN,ID por serie)\n", s_sel);
        return false;
    }
    s_patas[s_sel].vista = true;
    return true;
}

static void imprimir_rechazo(const twai_message_t *r)
{
    if (r->data[0] == LEG_CAN_REJECTED)
        printf("  -> pata %d RECHAZADA: %s\n", s_sel,
               r->data_length_code >= 2 ? nombre_motivo(r->data[1]) : "?");
    else
        printf("  -> pata %d %s\n", s_sel, nombre_respuesta(r->data[0]));
}

static bool leer_estado(int p)
{
    uint8_t d[1] = {LEG_CAN_CMD_STATUS};
    twai_message_t r;
    pata_t *q = &s_patas[p];
    vaciar_rx();
    if (enviar(ID_ORDEN(p), d, 1, false) != TX_OK) return false;
    int64_t t1;
    if (!esperar_trama(p, &r, &t1) || r.data[0] != LEG_CAN_STATUS || r.data_length_code != LEG_CAN_STATUS_DLC)
        return false;
    q->vista = true;
    q->flags = r.data[1];
    for (int i = 0; i < 3; i++) q->pos_mrad[i] = (int16_t)(r.data[2 + 2*i] | (r.data[3 + 2*i] << 8));
    /* Formato para maquinas: EST,ms,pata,coxa_mrad,femur_mrad,tibia_mrad,flags */
    if (s_stream)
        printf("EST,%lld,%d,%d,%d,%d,%u\n", (long long)(esp_timer_get_time() / 1000), p,
               q->pos_mrad[0], q->pos_mrad[1], q->pos_mrad[2], q->flags);
    return true;
}

static void imprimir_posiciones(const char *prefijo, int p)
{
    const pata_t *q = &s_patas[p];
    printf("%s,pata=%d,coxa=%+.3f,femur=%+.3f,tibia=%+.3f rad", prefijo, p,
           q->pos_mrad[0] / 1000.0, q->pos_mrad[1] / 1000.0, q->pos_mrad[2] / 1000.0);
    for (int i = 0; i < 3; i++)
        if (q->flags & (1U << (LEG_CAN_STATUS_NFAULT_SHIFT + i))) printf(",nFAULT_%s", k_art[i]);
    printf(",%s\n", (q->flags & LEG_CAN_STATUS_ARMED_BIT) ? "armada" : "desarmada");
}

static void estado_pata(void)
{
    if (!requiere_normal()) return;
    if (leer_estado(s_sel)) imprimir_posiciones("ESTADO", s_sel);
    else printf("FALLA,la pata %d no respondio a ESTADO\n", s_sel);
}

static void elegir(int p)
{
    s_sel = (uint8_t)p;
    printf("PATA,%d\n", p);
}

/* Pregunta IDENT a cada ID. Cada placa contesta con su MAC, asi que dos placas
 * con el mismo ID se notan: llegan dos respuestas distintas. */
static void escanear(void)
{
    if (!requiere_normal()) return;
    int total = 0;
    for (int p = 1; p <= LEG_CAN_MAX_LEGS; p++) {
        pata_t *q = &s_patas[p];
        uint8_t d[1] = {LEG_CAN_CMD_IDENT};
        int n = 0;
        bool distintas = false;
        twai_message_t f, primera = {0};
        vaciar_rx();
        tx_res_t r = enviar(ID_ORDEN(p), d, 1, false);
        if (r != TX_OK) {
            printf("SCAN,%d,SIN_BUS,%s\n", p, nombre_tx(r));
            q->vista = q->duplicada = false;
            continue;
        }
        int64_t limite = esp_timer_get_time() + 30000;
        while (esp_timer_get_time() < limite) {
            if (twai_receive(&f, ESPERA(5)) != ESP_OK) continue;
            if (f.extd || f.rtr || f.identifier != (uint32_t)ID_RESPUESTA(p)) continue;
            s_rx++;
            if (n++ == 0) primera = f;
            else if (f.data_length_code != primera.data_length_code ||
                     memcmp(f.data, primera.data, f.data_length_code)) distintas = true;
        }
        q->vista = n > 0;
        q->duplicada = distintas;
        if (!n) printf("SCAN,%d,NO\n", p);
        else if (distintas) printf("SCAN,%d,DUPLICADA\n", p);
        else if (primera.data[0] == LEG_CAN_IDENT && primera.data_length_code == LEG_CAN_IDENT_DLC)
            printf("SCAN,%d,OK,%02X%02X%02X%02X%02X%02X\n", p, primera.data[1], primera.data[2],
                   primera.data[3], primera.data[4], primera.data[5], primera.data[6]);
        else printf("SCAN,%d,OK,firmware_sin_IDENT\n", p);
        if (n) total++;
    }
    printf("SCAN,fin,%d\n", total);
}

static void armar(void)
{
    uint8_t d[1] = {LEG_CAN_CMD_ARM};
    twai_message_t r;
    pata_t *q = &s_patas[s_sel];
    if (q->duplicada) {
        printf("FALLA,hay dos placas con el ID %d: no la armo. Cambia un ID y vuelve a escanear\n", s_sel);
        return;
    }
    if (!orden(d, 1, &r)) return;
    if (r.data[0] == LEG_CAN_ACCEPTED) {
        q->armada = true;
        q->pendientes = 0;
        q->fallos = 0;
        printf("ARMADA,pata=%d: acepta movimientos. Le mando ESTADO cada 100 ms como latido;\n", s_sel);
        puts("  si la maestra se calla 0,5 s, la pata suelta las articulaciones.");
        if (leer_estado(s_sel)) imprimir_posiciones("ESTADO", s_sel);
    } else {
        imprimir_rechazo(&r);
    }
}

static void desarmar(void)
{
    uint8_t d[1] = {LEG_CAN_CMD_DISARM};
    twai_message_t r;
    s_patas[s_sel].armada = false;
    s_patas[s_sel].pendientes = 0;
    if (orden(d, 1, &r))
        printf("DESARMAR,pata=%d -> %s (articulaciones sueltas)\n", s_sel, nombre_respuesta(r.data[0]));
}

static void desarmar_todas(void)
{
    uint8_t previa = s_sel;
    for (int p = 1; p <= LEG_CAN_MAX_LEGS; p++)
        if (s_patas[p].armada || s_patas[p].vista) { s_sel = (uint8_t)p; desarmar(); }
    s_sel = previa;
}

/* mask: bit0 coxa, bit1 femur, bit2 tibia. rad[i] solo cuenta si su bit esta en mask. */
static void ir_a(uint8_t mask, const float rad[3])
{
    uint8_t d[LEG_CAN_GOTO_DLC] = {LEG_CAN_CMD_GOTO, mask};
    pata_t *q = &s_patas[s_sel];
    printf("IR_A,pata=%d", s_sel);
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
    if (q->duplicada) {
        printf("FALLA,hay dos placas con el ID %d: no la muevo\n", s_sel);
        return;
    }
    twai_message_t r;
    if (!orden(d, sizeof d, &r)) return;
    if (r.data[0] == LEG_CAN_ACCEPTED) {
        printf("  -> pata %d ACEPTADA, en camino\n", s_sel);
        q->pendientes |= mask;
        q->ultimo_pos = esp_timer_get_time();
        for (int i = 0; i < 3; i++) if (mask & (1U << i)) q->orden_t[i] = q->ultimo_pos;
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
        printf("CERO,pata=%d -> ACEPTADO:", s_sel);
        for (int i = 0; i < 3; i++) if (mask & (1U << i)) printf(" %s", k_art[i]);
        puts(" valen 0 en la pose actual");
    } else imprimir_rechazo(&r);
}

/* Latido de una pata: lee su ESTADO y avisa llegadas y fallas. */
static void latido_pata(int p)
{
    pata_t *q = &s_patas[p];
    if (!leer_estado(p)) {
        if (q->fallos < 255) q->fallos++;
        if (q->fallos == 3 && (q->armada || q->vista)) printf("AVISO,la pata %d no responde (3 seguidos)\n", p);
        if (q->fallos >= 3) q->vista = false;
        if (q->fallos == 5 && q->armada) {
            /* Ya paso su limite de 0,5 s: se solto sola. No retrasar a las demas. */
            q->armada = false;
            q->pendientes = 0;
            printf("AVISO,pata=%d se da por desarmada: no responde\n", p);
        }
        return;
    }
    q->fallos = 0;
    if (!q->armada) return;
    if (!(q->flags & LEG_CAN_STATUS_ARMED_BIT)) {
        q->armada = false;
        q->pendientes = 0;
        imprimir_posiciones("AVISO,la pata se desarmo (PARO, timeout, STOP por serie o falla)", p);
        return;
    }
    int64_t ahora = esp_timer_get_time();
    for (int i = 0; i < 3; i++) {
        uint8_t b = (uint8_t)(1U << i);
        if (!(q->pendientes & b)) continue;
        if (q->flags & (1U << (LEG_CAN_STATUS_NFAULT_SHIFT + i))) {
            printf("AVISO,pata=%d,nFAULT en %s\n", p, k_art[i]);
            q->pendientes &= (uint8_t)~b;
        } else if (ahora - q->orden_t[i] > 200000 && (q->flags & (1U << (LEG_CAN_STATUS_ARRIVED_SHIFT + i)))) {
            printf("LLEGO,pata=%d,%s,%+.3f rad,%.1f s\n", p, k_art[i], q->pos_mrad[i] / 1000.0,
                   (ahora - q->orden_t[i]) / 1e6);
            q->pendientes &= (uint8_t)~b;
        } else if (ahora - q->orden_t[i] > LLEGADA_MAX_US) {
            printf("AVISO,pata=%d,%s no llego en 15 s (esta en %+.3f rad)\n", p, k_art[i], q->pos_mrad[i] / 1000.0);
            q->pendientes &= (uint8_t)~b;
        }
    }
    if (q->pendientes && ahora - q->ultimo_pos >= POS_PRINT_US) {
        q->ultimo_pos = ahora;
        imprimir_posiciones("POS", p);
    }
}

/* Cada 100 ms: ESTADO a todas las patas armadas (latido). Con 'v 1', tambien a
 * las que responden y a la elegida, para la interfaz web. */
static void latido(void)
{
    if ((!alguna_armada() && !s_stream) || s_modo != TWAI_MODE_NORMAL) return;
    int64_t ahora = esp_timer_get_time();
    if (ahora - s_ultimo_latido < LATIDO_US) return;
    s_ultimo_latido = ahora;
    for (int p = 1; p <= LEG_CAN_MAX_LEGS && s_modo == TWAI_MODE_NORMAL; p++)
        if (s_patas[p].armada) latido_pata(p);
    if (!s_stream) return;
    for (int p = 1; p <= LEG_CAN_MAX_LEGS && s_modo == TWAI_MODE_NORMAL; p++)
        if (!s_patas[p].armada && (s_patas[p].vista || p == s_sel)) latido_pata(p);
}

static void enviar_contador(void)
{
    if (!requiere_normal()) return;
    uint32_t valor = ++s_seq;
    uint8_t d[4];
    memcpy(d, &valor, 4);
    uint32_t id = ID_RESPUESTA(pata_actual());
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
    char *w = strtok(l, " ,");
    if (!w) return;
    /* "p3": elige la pata 3. "p3 f 0.5": la elige y sigue con la orden. */
    if (s_cfg->es_maestra && w[0] == 'p' && w[1] >= '1' && w[1] <= '0' + LEG_CAN_MAX_LEGS && !w[2]) {
        elegir(w[1] - '0');
        w = strtok(NULL, " ,");
        if (!w) return;
    }
    char *x = strtok(NULL, " ,"), *y = strtok(NULL, " ,");
    if (!x && strlen(w) == 1) { comando(w[0]); return; }
    if (!s_cfg->es_maestra) { puts("AVISO,la pata solo acepta comandos de una letra"); return; }

    float rad[3] = {0, 0, 0};
    int j = articulacion(w);
    if (!strcmp(w, "v")) {
        s_stream = x && x[0] == '1';
        printf("STREAM,%d\n", s_stream ? 1 : 0);
    }
    else if (!strcmp(w, "escanear")) escanear();
    else if (!strcmp(w, "armar")) armar();
    else if (!strcmp(w, "d") || !strcmp(w, "desarmar")) {
        if (x && !strcmp(x, "todas")) desarmar_todas();
        else if (!x) desarmar();
        else puts("AVISO,uso: d | d todas");
    }
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
    if (cfg->es_maestra) s_sel = cfg->pata_id;
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    printf("\n=== PRUEBA CAN: %s ===\n", cfg->nombre);
    printf("PLACA,TX=GPIO%d->CTX,RX=GPIO%d<-CRX,500000 bit/s,pata_id=%d,rol=%s\n",
           cfg->pin_tx, cfg->pin_rx, pata_actual(), cfg->es_maestra ? "MAESTRA" : "PATA");
    iniciar(TWAI_MODE_LISTEN_ONLY);
    medir_crx();
    estado();
    ayuda();

    char linea[64];
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
