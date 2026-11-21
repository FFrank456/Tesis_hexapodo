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
#include <stdio.h>
#include <string.h>
#include <unistd.h>

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
    default:             return "DESCONOCIDA";
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
    puts("COMANDOS (una tecla, sin Enter):");
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
        puts("  m  mueve femur y tibia por CAN +-50 mrad (antes CAN,ARM en la pata)");
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

/* La maestra espera la respuesta de su pata; las demas tramas se procesan normal. */
static bool esperar_respuesta(uint8_t *codigo, int64_t *t_llegada)
{
    int64_t limite = esp_timer_get_time() + ESPERA_RESP_US;
    twai_message_t f;
    while (esp_timer_get_time() < limite) {
        if (twai_receive(&f, ESPERA(5)) != ESP_OK) continue;
        if (!f.extd && !f.rtr && f.identifier == ID_RESPUESTA(s_cfg->pata_id) &&
            f.data_length_code == 1) {
            *t_llegada = esp_timer_get_time();
            *codigo = f.data[0];
            s_rx++;
            return true;
        }
        procesar_rx(&f);
    }
    return false;
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

/* Mueve fémur y tibia por CAN (máscara 0x06) con consignas relativas al origen
 * que la pata tomó en CAN,ARM. Límite del protocolo: ±50 mrad. Manda una trama
 * cada 50 ms (la pata se desarma si pasan 300 ms sin consigna). Termina en el
 * origen y con PARO. Cualquier tecla durante el movimiento manda PARO. */
#define MOVER_MASCARA    0x06
#define MOVER_PERIODO_US 50000

static bool mover_tramo(int16_t off_m2, int16_t off_m3, int ms, int *aceptadas, uint8_t *fallo)
{
    uint8_t d[7] = {MOVER_MASCARA, 0, 0,
                    (uint8_t)(off_m2 & 0xFF), (uint8_t)((uint16_t)off_m2 >> 8),
                    (uint8_t)(off_m3 & 0xFF), (uint8_t)((uint16_t)off_m3 >> 8)};
    int64_t fin = esp_timer_get_time() + (int64_t)ms * 1000;
    int64_t siguiente = esp_timer_get_time();
    while (esp_timer_get_time() < fin) {
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) { *fallo = 0xFE; return false; }
        tx_res_t r = enviar(ID_ORDEN(s_cfg->pata_id), d, sizeof d, false);
        if (r != TX_OK) { explicar_fallo_tx(r); *fallo = 0xFD; return false; }
        uint8_t codigo;
        int64_t t1;
        if (!esperar_respuesta(&codigo, &t1)) { *fallo = 0xFC; return false; }
        if (codigo != RESP_ACEPTADA) { *fallo = codigo; return false; }
        (*aceptadas)++;
        siguiente += MOVER_PERIODO_US;
        int64_t espera = siguiente - esp_timer_get_time();
        if (espera > 0) vTaskDelay(ESPERA((uint32_t)(espera / 1000)));
    }
    return true;
}

static void mover(void)
{
    if (!requiere_normal()) return;
    static const struct { int16_t m2, m3; int ms; } tramos[] = {
        {0, 0, 1000}, {50, 50, 3000}, {0, 0, 3000}, {-50, -50, 3000}, {0, 0, 3000},
    };
    puts("MOVER,femur y tibia por CAN: 0 -> +50 -> 0 -> -50 -> 0 mrad (unos 13 s)");
    puts("MOVER,la pata debe estar armada con CAN,ARM; cualquier tecla = PARO");
    twai_clear_receive_queue();
    int total = 0;
    bool ok = true;
    for (size_t i = 0; i < sizeof tramos / sizeof tramos[0] && ok; i++) {
        int aceptadas = 0;
        uint8_t fallo = 0;
        ok = mover_tramo(tramos[i].m2, tramos[i].m3, tramos[i].ms, &aceptadas, &fallo);
        total += aceptadas;
        printf("MOVER,tramo=%u,femur_mrad=%d,tibia_mrad=%d,aceptadas=%d", (unsigned)(i + 1),
               tramos[i].m2, tramos[i].m3, aceptadas);
        if (ok) putchar('\n');
        else if (fallo == 0xFE) puts(",cortado por tecla");
        else if (fallo == 0xFD) puts(",fallo de TX");
        else if (fallo == 0xFC) puts(",la pata no respondio en 50 ms");
        else printf(",la pata respondio %s\n", nombre_respuesta(fallo));
    }
    printf("MOVER,fin,consignas_aceptadas=%d\n", total);
    puts(ok ? "PASA,mover" : "FALLA,mover");
    paro();
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
    /* Separa la salida de cada comando y muestra que tecla llego. */
    if (c != '\r' && c != '\n' && c != ' ') printf("\n>> %c\n", c);
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
    case 'm': if (s_cfg->es_maestra) mover(); else puts("AVISO,'m' es solo de la maestra"); break;
    case '\r': case '\n': case ' ': break;
    default: printf("AVISO,comando '%c' desconocido; 'h' para ayuda\n", c); break;
    }
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

    while (true) {
        twai_message_t f;
        if (s_instalado && twai_receive(&f, ESPERA(5)) == ESP_OK) procesar_rx(&f);
        else if (!s_instalado) vTaskDelay(ESPERA(5));

        uint32_t a;
        if (s_instalado && twai_read_alerts(&a, 0) == ESP_OK) procesar_alertas(a);

        char c;
        for (int i = 0; i < 8 && read(STDIN_FILENO, &c, 1) == 1; i++) comando(c);
    }
}
