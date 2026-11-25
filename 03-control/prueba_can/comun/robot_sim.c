/*
 * Fase 1 de la maestra: cinematica inversa y marcha SIMULADAS. Este archivo no incluye
 * twai.h ni llama a nada que transmita: no puede mover ningun motor.
 *
 * Salida para la interfaz (robot_dashboard.html):
 *   ROB,marcha,radio,altura,paso_x,paso_y,giro_grados,elevacion,ciclo_s,m1..m6   (m = ID CAN de cada posicion)
 *   SIM,estado,fase,x_mm,y_mm,rumbo_grados,margen_mm        10 por segundo
 *   SIMP,pos,apoyo,avance_pct,q0,q1,q2 (mrad),x,y,z (decimas de mm, marco del cuerpo)
 *   SIM,RECHAZADA,motivo,pos,fase | SIM,VALIDA,margen_mm,vel_rad_s,separacion_mm
 *   CI,OK,q0,q1,q2 | CI,motivo
 */
#include "robot_sim.h"
#include "robot.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#define SIM_US 100000
#define GRADO 0.01745329f
#define VEL_MAX 2.0f        /* rad/s: PLIMIT de las patas */
#define CFG_VERSION 1U

typedef struct {
    uint32_t version;
    robot_cfg_t cfg;
    float ciclo_s;
    int32_t mapa[ROBOT_PATAS];
} guardado_t;
static bool s_nvs;

typedef enum { SIM_PARADA = 0, SIM_CORRE, SIM_PAUSA } sim_t;

static robot_cfg_t s_cfg;
static bool s_listo;
static int s_mapa[ROBOT_PATAS] = {1, 2, 3, 4, 5, 6};
static sim_t s_sim;
static float s_ciclo_s = 12.0f, s_fase, s_x, s_y, s_rumbo;
static int64_t s_ultimo;
static robot_pata_t s_pata[ROBOT_PATAS];
static const char *const k_marcha[MARCHA_N] = {"ondulada", "ripple", "tripode"};
static const char *const k_motivo[] = {"OK", "inalcanzable", "limite_coxa", "limite_femur", "limite_tibia", "inestable",
                                       "parametros", "choque_entre_patas"};

static bool mapa_valido(const int32_t m[ROBOT_PATAS])
{
    unsigned vistos = 0;
    for (int i = 0; i < ROBOT_PATAS; i++) {
        if (m[i] < 1 || m[i] > ROBOT_PATAS) return false;
        vistos |= 1U << m[i];
    }
    return vistos == 0x7E;
}

static bool ciclo_valido(const robot_val_t *v, float ciclo)
{
    return isfinite(ciclo) && ciclo >= 4 && ciclo <= 60 && v->dq / ciclo <= VEL_MAX;
}

/* Lo guardado solo se usa si sigue siendo valido con este firmware. */
void robot_iniciar(void)
{
    if (s_listo) return;
    s_listo = true;
    robot_cfg_defecto(&s_cfg);
    esp_err_t e = nvs_flash_init();
    s_nvs = e == ESP_OK;      /* si la NVS falla no se borra nada: se usan los valores iniciales */
    if (!s_nvs) { printf("NVS,ERROR,%s\n", esp_err_to_name(e)); return; }
    nvs_handle_t h;
    guardado_t g;
    size_t n = sizeof g;
    robot_val_t v;
    if (nvs_open("robot", NVS_READONLY, &h) != ESP_OK) { puts("NVS,VACIA"); return; }
    e = nvs_get_blob(h, "cfg", &g, &n);
    nvs_close(h);
    if (e == ESP_OK && n == sizeof g && g.version == CFG_VERSION && mapa_valido(g.mapa) &&
        robot_validar(&g.cfg, &v) == ROBOT_OK && ciclo_valido(&v, g.ciclo_s)) {
        s_cfg = g.cfg;
        s_ciclo_s = g.ciclo_s;
        for (int i = 0; i < ROBOT_PATAS; i++) s_mapa[i] = (int)g.mapa[i];
        puts("NVS,CARGADA");
    } else puts("NVS,IGNORADA,no valida: valores iniciales");
}

static void guardar(void)
{
    if (!s_nvs) { puts("ERROR,NVS no disponible"); return; }
    guardado_t g;
    memset(&g, 0, sizeof g);
    g.version = CFG_VERSION;
    g.cfg = s_cfg;
    g.ciclo_s = s_ciclo_s;
    for (int i = 0; i < ROBOT_PATAS; i++) g.mapa[i] = s_mapa[i];
    nvs_handle_t h;
    esp_err_t e = nvs_open("robot", NVS_READWRITE, &h);
    if (e == ESP_OK) {
        e = nvs_set_blob(h, "cfg", &g, sizeof g);
        if (e == ESP_OK) e = nvs_commit(h);
        nvs_close(h);
    }
    if (e == ESP_OK) puts("NVS,GUARDADA");
    else printf("ERROR,NVS,%s\n", esp_err_to_name(e));
}

static bool num(float *v)
{
    char *t = strtok(NULL, " ,"), *fin = NULL;
    if (!t) return false;
    *v = strtof(t, &fin);
    return fin != t && *fin == '\0' && isfinite(*v);
}

static void imprimir_cfg(void)
{
    printf("ROB,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", k_marcha[s_cfg.marcha], s_cfg.radio, s_cfg.altura,
           s_cfg.paso_x, s_cfg.paso_y, s_cfg.paso_giro / GRADO, s_cfg.elevacion, s_ciclo_s);
    for (int i = 0; i < ROBOT_PATAS; i++) printf(",%d", s_mapa[i]);
    putchar('\n');
}

static void imprimir_sim(void)
{
    static const char *const est[] = {"PARADA", "CORRE", "PAUSA"};
    float margen;
    robot_marcha(&s_cfg, s_fase, s_pata, &margen);
    printf("SIM,%s,%.4f,%.1f,%.1f,%.1f,%.1f\n", est[s_sim], s_fase, s_x, s_y, s_rumbo / GRADO, margen);
    for (int i = 0; i < ROBOT_PATAS; i++) {
        const robot_pata_t *p = &s_pata[i];
        printf("SIMP,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld\n", i + 1, p->apoyo, (int)lroundf(p->avance * 100),
               lroundf(p->q[0] * 1000), lroundf(p->q[1] * 1000), lroundf(p->q[2] * 1000),
               lroundf(p->pie[0] * 10), lroundf(p->pie[1] * 10), lroundf(p->pie[2] * 10));
    }
}

static bool validar(const robot_cfg_t *c, float ciclo)
{
    robot_val_t v;
    if (robot_validar(c, &v) != ROBOT_OK) {
        printf("SIM,RECHAZADA,%s,%d,%.3f\n", k_motivo[v.motivo], v.pos + 1, v.fase);
        return false;
    }
    if (!ciclo_valido(&v, ciclo)) {
        printf("SIM,RECHAZADA,velocidad_o_ciclo,0,0.000\n");
        return false;
    }
    printf("SIM,VALIDA,%.1f,%.2f,%.1f\n", v.margen, v.dq / ciclo, v.sep);
    return true;
}
/* Cambia la configuracion solo si TODO el ciclo es posible. */
static void aplicar(const robot_cfg_t *c)
{
    if (validar(c, s_ciclo_s)) s_cfg = *c;
    imprimir_cfg();
}

/* Avance estimado del cuerpo: supone pies que no patinan. En un ciclo el cuerpo avanza
 * paso/apoyo, porque el pie recorre "paso" solo durante la fraccion de apoyo. */
static void avanzar(float dfase)
{
    float k = dfase / robot_factor_apoyo(s_cfg.marcha);
    float dr = k * s_cfg.paso_giro;
    float dx = k * s_cfg.paso_x, dy = k * s_cfg.paso_y;
    float m = s_rumbo + dr / 2.0f;
    s_x += cosf(m) * dx - sinf(m) * dy;
    s_y += sinf(m) * dx + cosf(m) * dy;
    s_rumbo += dr;
    s_fase += dfase;
    s_fase -= floorf(s_fase);
}

/* Ida y vuelta: angulos al azar -> pie -> angulos, dentro de los limites. */
static void autotest_ci(void)
{
    const int n = 2000;
    static const float lo[3] = {-0.5f, -1.0f, -0.34f}, hi[3] = {0.6f, 1.0f, 0.7f};
    float peor = 0;
    int malas = 0;
    for (int k = 0; k < n; k++) {
        float q[3], pie[3], q2[3] = {0}, pie2[3];
        for (int i = 0; i < 3; i++) q[i] = lo[i] + (hi[i] - lo[i]) * ((float)(esp_random() >> 8) / 16777216.0f);
        robot_directa(q, pie);
        /* Se compara el pie: cerca de la pierna recta el angulo pierde precision. */
        if (robot_inversa(pie, q2) == ROBOT_INALCANZABLE) { malas++; continue; }
        robot_directa(q2, pie2);
        float e = 0;
        for (int i = 0; i < 3; i++) e += (pie[i] - pie2[i]) * (pie[i] - pie2[i]);
        e = sqrtf(e);
        if (e > peor) peor = e;
    }
    printf("CI,AUTOTEST,%s,poses=%d,fallas=%d,error_max_mm=%.4f\n",
           malas == 0 && peor < 0.1f ? "PASA" : "FALLA", n, malas, peor);
}

void robot_ayuda(void)
{
    puts("ROBOT (fase 1: solo calculo y simulacion, no mueve motores):");
    puts("  robot                      configuracion;  robot defecto = valores iniciales");
    puts("  marcha ondulada|ripple|tripode");
    puts("  postura <radio> <altura>   mm       paso <x> <y> <giro_grados> <elevacion>");
    puts("  config <radio> <altura> <x> <y> <giro> <elevacion> <ciclo_s> <marcha>   todo a la vez");
    puts("  ciclo <s>                  duracion de un ciclo (4..60)");
    puts("  sim 1 | sim 0 | sim pausa | sim sig    (sig = avanza 1/24 de ciclo)");
    puts("  mapa <6 IDs>               ID CAN de cada posicion (DD MD TD TI MI DI)");
    puts("  guardar                    configuracion y mapa en la flash de la maestra");
    puts("  ci <x> <y> <z>             inversa de una pata (mm, marco de J0);  ci test");
}

bool robot_comando(const char *w)
{
    robot_iniciar();
    robot_cfg_t c = s_cfg;
    if (!strcmp(w, "robot")) {
        char *t = strtok(NULL, " ,");
        if (t && !strcmp(t, "defecto")) { s_sim = SIM_PARADA; robot_cfg_defecto(&s_cfg); s_ciclo_s = 12.0f; s_fase = s_x = s_y = s_rumbo = 0; }
        imprimir_cfg();
        imprimir_sim();
    } else if (!strcmp(w, "marcha")) {
        char *t = strtok(NULL, " ,");
        int k = -1;
        for (int i = 0; t && i < MARCHA_N; i++) if (!strcmp(t, k_marcha[i])) k = i;
        if (k < 0) puts("AVISO,uso: marcha ondulada|ripple|tripode");
        else { c.marcha = k; aplicar(&c); }
    } else if (!strcmp(w, "postura")) {
        if (num(&c.radio) && num(&c.altura)) aplicar(&c);
        else puts("AVISO,uso: postura <radio_mm> <altura_mm>");
    } else if (!strcmp(w, "paso")) {
        float g;
        if (num(&c.paso_x) && num(&c.paso_y) && num(&g) && num(&c.elevacion) && c.elevacion >= 0) {
            c.paso_giro = g * GRADO;
            aplicar(&c);
        } else puts("AVISO,uso: paso <x_mm> <y_mm> <giro_grados> <elevacion_mm>");
    } else if (!strcmp(w, "config")) {   /* todo de una vez: se valida junto */
        float g, ciclo;
        int k = -1;
        char *tipo;
        if (num(&c.radio) && num(&c.altura) && num(&c.paso_x) && num(&c.paso_y) && num(&g) && num(&c.elevacion) &&
            num(&ciclo) && (tipo = strtok(NULL, " ,")) != NULL) {
            for (int i = 0; i < MARCHA_N; i++) if (!strcmp(tipo, k_marcha[i])) k = i;
        }
        if (k < 0) { puts("AVISO,uso: config <radio> <altura> <x> <y> <giro_grados> <elevacion> <ciclo_s> <marcha>"); return true; }
        c.paso_giro = g * GRADO;
        c.marcha = k;
        if (validar(&c, ciclo)) { s_cfg = c; s_ciclo_s = ciclo; }
        imprimir_cfg();
    } else if (!strcmp(w, "ciclo")) {
        float s;
        if (num(&s) && s >= 4 && s <= 60) { if (validar(&s_cfg, s)) s_ciclo_s = s; imprimir_cfg(); }
        else puts("AVISO,uso: ciclo <segundos, 4..60>");
    } else if (!strcmp(w, "mapa")) {
        int32_t m[ROBOT_PATAS] = {0};
        bool ok = true;
        for (int i = 0; i < ROBOT_PATAS && ok; i++) {
            float v = 0;
            ok = num(&v) && v >= 1 && v <= 6 && v == floorf(v);
            if (ok) m[i] = (int32_t)v;
        }
        if (ok && mapa_valido(m)) { for (int i = 0; i < ROBOT_PATAS; i++) s_mapa[i] = (int)m[i]; imprimir_cfg(); }
        else puts("AVISO,uso: mapa <6 IDs distintos, 1..6> en orden DD MD TD TI MI DI");
    } else if (!strcmp(w, "guardar")) {
        guardar();
    } else if (!strcmp(w, "sim")) {
        char *t = strtok(NULL, " ,");
        if (t && !strcmp(t, "1")) {
            if (validar(&s_cfg, s_ciclo_s)) { s_sim = SIM_CORRE; s_ultimo = esp_timer_get_time(); }
        } else if (t && !strcmp(t, "0")) { s_sim = SIM_PARADA; s_fase = s_x = s_y = s_rumbo = 0; }
        else if (t && !strcmp(t, "pausa")) { if (s_sim == SIM_CORRE) s_sim = SIM_PAUSA; }
        else if (t && !strcmp(t, "sig")) { s_sim = SIM_PAUSA; avanzar(1.0f / 24.0f); }
        else { puts("AVISO,uso: sim 1 | sim 0 | sim pausa | sim sig"); return true; }
        imprimir_sim();
    } else if (!strcmp(w, "ci")) {
        char *t = strtok(NULL, " ,"), *fin = NULL;
        float p[3], q[3] = {0};
        if (t && !strcmp(t, "test")) { autotest_ci(); return true; }
        if (t) p[0] = strtof(t, &fin);
        if (!t || fin == t || *fin || !isfinite(p[0]) || !num(&p[1]) || !num(&p[2])) {
            puts("AVISO,uso: ci <x> <y> <z> | ci test");
            return true;
        }
        int r = robot_inversa(p, q);
        if (r == ROBOT_OK) printf("CI,OK,%.4f,%.4f,%.4f\n", q[0], q[1], q[2]);
        else printf("CI,%s\n", k_motivo[r]);
    } else return false;
    return true;
}

void robot_sim_tick(void)
{
    if (s_sim != SIM_CORRE) return;
    int64_t ahora = esp_timer_get_time();
    if (ahora - s_ultimo < SIM_US) return;
    float dt = (float)(ahora - s_ultimo) / 1e6f;
    s_ultimo = ahora;
    if (dt > 0.5f) dt = 0.5f;
    avanzar(dt / s_ciclo_s);
    imprimir_sim();
}
