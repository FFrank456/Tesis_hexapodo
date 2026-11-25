/* Cinematica y marcha del hexapodo. Ver robot.h. Calculo puro, sin ESP-IDF.
 * Las ecuaciones de la pata son las de 03-control/cinematica (cd_pata.m, ci_pata.m). */
#include "robot.h"

#include <math.h>

#define PI_F 3.14159265f

/* Pata (geometria_pata.m, 19-nov). */
#define L1 50.0f
#define H1 6.05f
#define DLAT (-0.071f)
#define L2 60.0f
#define L3 104.4199f
#define GAMMA 0.355623f

/* Montaje de J0: x, y en mm respecto del centro y direccion de la pata centrada.
 * Agujeros de fijacion de fusion-ensamble/1_estructura.py mas los 23.03 mm radiales hasta
 * J0 (pie de la pose cero menos 207.9 mm a 45 grados). Numeracion y frente: vista superior
 * del usuario (25-nov), patas 1..6 en sentido antihorario desde arriba. */
static const float k_x[6] = {61.51f, 5.0f, -61.51f, -61.51f, 5.0f, 61.51f};
static const float k_y[6] = {46.66f, 62.94f, 46.66f, -46.66f, -62.94f, -46.66f};
static const float k_dir[6] = {0.785398f, 1.570796f, 2.356194f, -2.356194f, -1.570796f, -0.785398f};

/* Encoder: th = signo*q. Limites = interseccion del rango CAN y del provisional de MATLAB. */
static const float k_signo[3] = {1.0f, -1.0f, 1.0f};
static const float k_min[3] = {-0.5f, -1.0f, -1.0f};
static const float k_max[3] = {0.6f, 1.0f, 0.7f};

/* Inicio del vuelo de cada posicion dentro del ciclo. */
static const float k_desfase[3][6] = {
    {0.333333f, 0.166667f, 0.0f, 0.5f, 0.666667f, 0.833333f},
    {0.666667f, 0.333333f, 0.0f, 0.5f, 0.833333f, 0.166667f},
    {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f},
};
/* Un poco mas de 5/6, 2/3 y 1/2: la pata que baja apoya antes de que suba la siguiente. */
static const float k_apoyo[3] = {0.84f, 0.68f, 0.54f};

void robot_cfg_defecto(robot_cfg_t *c)
{
    c->marcha = MARCHA_ONDULADA;
    c->radio = 182.0f;
    c->altura = 75.0f;
    c->paso_x = 20.0f;
    c->paso_y = 0.0f;
    c->paso_giro = 0.0f;
    c->elevacion = 15.0f;
}

void robot_directa(const float q[3], float pie[3])
{
    float th0 = k_signo[0] * q[0];
    float th1 = k_signo[1] * q[1];
    float th2 = k_signo[2] * q[2];
    float phi = th1 - th2 - GAMMA;
    float r = L1 + L2 * cosf(th1) + L3 * cosf(phi);
    pie[0] = r * cosf(th0) - DLAT * sinf(th0);
    pie[1] = r * sinf(th0) + DLAT * cosf(th0);
    pie[2] = H1 + L2 * sinf(th1) + L3 * sinf(phi);
}

int robot_inversa(const float pie[3], float q[3])
{
    float x = pie[0];
    float y = pie[1];
    float rho2 = x * x + y * y;
    if (rho2 < DLAT * DLAT) return ROBOT_INALCANZABLE;
    float r = sqrtf(rho2 - DLAT * DLAT);
    float th0 = atan2f(y, x) - atan2f(DLAT, r);
    th0 = atan2f(sinf(th0), cosf(th0));
    if (fabsf(th0) > PI_F / 2.0f) {      /* el pie queda detras del eje de la coxa */
        r = -r;
        th0 = atan2f(y, x) - atan2f(DLAT, r);
        th0 = atan2f(sinf(th0), cosf(th0));
    }
    float a = r - L1;
    float b = pie[2] - H1;
    float c = (a * a + b * b - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
    if (fabsf(c) > 1.000001f) return ROBOT_INALCANZABLE;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    float beta = -acosf(c);
    float th1 = atan2f(b, a) - atan2f(L3 * sinf(beta), L2 + L3 * cosf(beta));
    float th2 = -beta - GAMMA;
    th1 = atan2f(sinf(th1), cosf(th1));
    q[0] = th0 * k_signo[0];
    q[1] = th1 * k_signo[1];
    q[2] = th2 * k_signo[2];
    for (int i = 0; i < 3; i++) {
        if (q[i] < k_min[i] || q[i] > k_max[i]) return ROBOT_LIM_COXA + i;
    }
    return ROBOT_OK;
}

void robot_j0_a_cuerpo(int pos, const float p[3], float out[3])
{
    float c = cosf(k_dir[pos]);
    float s = sinf(k_dir[pos]);
    float x = p[0];
    float y = p[1];
    out[0] = k_x[pos] + c * x - s * y;
    out[1] = k_y[pos] + s * x + c * y;
    out[2] = p[2];
}

void robot_cuerpo_a_j0(int pos, const float p[3], float out[3])
{
    float c = cosf(k_dir[pos]);
    float s = sinf(k_dir[pos]);
    float x = p[0] - k_x[pos];
    float y = p[1] - k_y[pos];
    out[0] = c * x + s * y;
    out[1] = -s * x + c * y;
    out[2] = p[2];
}

float robot_factor_apoyo(int marcha)
{
    return k_apoyo[marcha];
}

/* Los pies, recorridos en orden 0..5, van en sentido antihorario: cada lado debe dejar
 * el centro del cuerpo a su izquierda. Supone el centro de masa en el origen. */
float robot_margen(const robot_pata_t pata[ROBOT_PATAS])
{
    int idx[6];
    int n = 0;
    for (int i = 0; i < 6; i++) {
        if (pata[i].apoyo) {
            idx[n] = i;
            n++;
        }
    }
    if (n < 3) return -999.0f;
    float m = 1e9f;
    for (int k = 0; k < n; k++) {
        int ia = idx[k];
        int ib = idx[(k + 1) % n];
        float ax = pata[ia].pie[0];
        float ay = pata[ia].pie[1];
        float dx = pata[ib].pie[0] - ax;
        float dy = pata[ib].pie[1] - ay;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1.0f) continue;
        float dist = (dy * ax - dx * ay) / len;
        if (dist < m) m = dist;
    }
    return m;
}

/* Pies y angulos de las seis patas en una fase del ciclo (0..1). Devuelve el primer
 * motivo distinto de ROBOT_OK. */
int robot_marcha(const robot_cfg_t *c, float fase, robot_pata_t pata[ROBOT_PATAS], float *margen)
{
    int res = ROBOT_OK;
    float apoyo = k_apoyo[c->marcha];
    float vuelo = 1.0f - apoyo;
    for (int i = 0; i < 6; i++) {
        float local[3];
        local[0] = c->radio;
        local[1] = 0.0f;
        local[2] = -c->altura;
        float nom[3];
        robot_j0_a_cuerpo(i, local, nom);
        float f = fase - k_desfase[c->marcha][i];
        f = f - floorf(f);
        float u;
        float z = 0.0f;
        if (f < vuelo) {
            float s = f / vuelo;
            pata[i].apoyo = false;
            pata[i].avance = s;
            u = -0.5f + s - sinf(2.0f * PI_F * s) / (2.0f * PI_F);
            z = c->elevacion * 0.5f * (1.0f - cosf(2.0f * PI_F * s));
        } else {
            float s = (f - vuelo) / apoyo;
            pata[i].apoyo = true;
            pata[i].avance = s;
            u = 0.5f - s;
        }
        /* Traslacion mas giro alrededor del centro. */
        float ang = u * c->paso_giro;
        float ca = cosf(ang);
        float sa = sinf(ang);
        pata[i].pie[0] = ca * nom[0] - sa * nom[1] + u * c->paso_x;
        pata[i].pie[1] = sa * nom[0] + ca * nom[1] + u * c->paso_y;
        pata[i].pie[2] = nom[2] + z;
        float pj[3];
        robot_cuerpo_a_j0(i, pata[i].pie, pj);
        pata[i].motivo = robot_inversa(pj, pata[i].q);
        if (res == ROBOT_OK) res = pata[i].motivo;
    }
    *margen = robot_margen(pata);
    if (res == ROBOT_OK && *margen < 0.0f) res = ROBOT_INESTABLE;
    return res;
}

/* Distancia de un punto al segmento A-B, en planta. */
static float robot_punto_seg(float px, float py, float ax, float ay, float bx, float by)
{
    float dx = bx - ax;
    float dy = by - ay;
    float l2 = dx * dx + dy * dy;
    float u = 0.0f;
    if (l2 > 0.001f) u = ((px - ax) * dx + (py - ay) * dy) / l2;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    float ex = px - ax - u * dx;
    float ey = py - ay - u * dy;
    return sqrtf(ex * ex + ey * ey);
}

/* Separacion en planta entre las patas i y j, vistas como el segmento J0 -> pie. Si una
 * cruza a la otra vale 0. */
static float robot_separacion(const robot_pata_t pata[ROBOT_PATAS], int i, int j)
{
    float ax = k_x[i];
    float ay = k_y[i];
    float bx = pata[i].pie[0];
    float by = pata[i].pie[1];
    float cx = k_x[j];
    float cy = k_y[j];
    float dx = pata[j].pie[0];
    float dy = pata[j].pie[1];
    float o1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    float o2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
    float o3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
    float o4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);
    if (o1 * o2 < 0.0f && o3 * o4 < 0.0f) return 0.0f;
    float m = robot_punto_seg(bx, by, cx, cy, dx, dy);
    float n = robot_punto_seg(dx, dy, ax, ay, bx, by);
    if (n < m) m = n;
    n = robot_punto_seg(ax, ay, cx, cy, dx, dy);
    if (n < m) m = n;
    n = robot_punto_seg(cx, cy, ax, ay, bx, by);
    if (n < m) m = n;
    return m;
}

/* Recorre el ciclo completo: alcance, limites, estabilidad, choques entre vecinas y
 * velocidad articular. Se detiene en el primer fallo. */
int robot_validar(const robot_cfg_t *c, robot_val_t *v)
{
    static robot_pata_t p[6];
    static float ant[18];
    v->motivo = ROBOT_CONFIG;
    v->pos = -1;
    v->fase = 0.0f;
    v->margen = 1e9f;
    v->dq = 0.0f;
    v->sep = 1e9f;
    if (c->marcha < 0 || c->marcha > 2) return v->motivo;
    if (!(c->radio > 50.0f && c->radio < 215.0f)) return v->motivo;
    if (!(c->altura >= 20.0f && c->altura <= 160.0f)) return v->motivo;
    if (!(c->elevacion >= 0.0f && c->elevacion < c->altura)) return v->motivo;
    if (!(fabsf(c->paso_x) <= 200.0f && fabsf(c->paso_y) <= 200.0f && fabsf(c->paso_giro) <= 1.0f)) return v->motivo;
    for (int k = 0; k <= 240; k++) {
        float m;
        float f = (float)k / 240.0f;
        int r = robot_marcha(c, f, p, &m);
        if (m < v->margen) v->margen = m;
        for (int i = 0; i < 6; i++) {
            float s = robot_separacion(p, i, (i + 1) % 6);
            if (s < v->sep) v->sep = s;
            if (r == ROBOT_OK && s < ROBOT_SEP_MIN) {
                r = ROBOT_CHOQUE;
                v->pos = i;
            }
        }
        if (r != ROBOT_OK) {
            v->motivo = r;
            v->fase = f;
            for (int i = 0; i < 6; i++) {
                if (p[i].motivo != ROBOT_OK) {
                    v->pos = i;
                    break;
                }
            }
            return r;
        }
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                float d = fabsf(p[i].q[j] - ant[i * 3 + j]) * 240.0f;
                if (k > 0 && d > v->dq) v->dq = d;
                ant[i * 3 + j] = p[i].q[j];
            }
        }
    }
    v->motivo = ROBOT_OK;
    return ROBOT_OK;
}
