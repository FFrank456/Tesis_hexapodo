#pragma once
/*
 * Cinematica y marcha del hexapodo (Unidad 2). Calculo puro: no usa CAN ni mueve nada.
 * Unidades: mm y rad. Marco del cuerpo: origen en el centro de las coxas, a la altura
 * de J0; x hacia el frente, y hacia el lado de las patas 1-2-3, z arriba (giro + =
 * antihorario visto desde arriba).
 *
 * Posiciones 0..5 = patas 1..6; la pata n lleva el ID n (vista superior del usuario, 25-nov):
 *   1 delantera derecha   2 media derecha    3 trasera derecha
 *   4 trasera izquierda   5 media izquierda  6 delantera izquierda
 * OJO: "derecha" e "izquierda" son las de quien mira al robot de frente. El lado 1-2-3 es
 * el izquierdo del propio robot (por eso es +y).
 */
#include <stdbool.h>

#define ROBOT_PATAS 6
#define ROBOT_SEP_MIN 25.0f     /* mm entre los ejes de dos patas vecinas, en planta */

typedef enum { MARCHA_ONDULADA = 0, MARCHA_RIPPLE, MARCHA_TRIPODE, MARCHA_N } marcha_t;

/* Motivo por el que una pose no sirve. */
enum { ROBOT_OK = 0, ROBOT_INALCANZABLE, ROBOT_LIM_COXA, ROBOT_LIM_FEMUR, ROBOT_LIM_TIBIA, ROBOT_INESTABLE,
       ROBOT_CONFIG, ROBOT_CHOQUE };

typedef struct {
    int marcha;
    float radio;        /* J0 -> pie en planta, con la pata centrada */
    float altura;       /* J0 sobre el suelo */
    float paso_x;       /* recorrido del pie en apoyo: al frente, al lado 1-2-3, giro */
    float paso_y;
    float paso_giro;
    float elevacion;    /* altura del pie en vuelo */
} robot_cfg_t;

typedef struct {
    float pie[3];       /* pie en el marco del cuerpo */
    float q[3];         /* angulos de ENCODER: coxa, femur, tibia */
    bool apoyo;
    float avance;       /* 0..1 dentro de su fase (apoyo o vuelo) */
    int motivo;
} robot_pata_t;

typedef struct {
    int motivo;
    int pos;            /* posicion que falla, -1 si no es de una pata */
    float fase;
    float margen;       /* minimo del ciclo, mm */
    float dq;           /* velocidad articular maxima, rad por ciclo */
    float sep;          /* separacion minima entre patas vecinas en planta, mm */
} robot_val_t;

void robot_cfg_defecto(robot_cfg_t *c);
void robot_directa(const float q[3], float pie[3]);             /* encoder -> pie, marco de J0 */
int  robot_inversa(const float pie[3], float q[3]);             /* pie, marco de J0 -> encoder */
void robot_j0_a_cuerpo(int pos, const float p[3], float out[3]);
void robot_cuerpo_a_j0(int pos, const float p[3], float out[3]);
float robot_factor_apoyo(int marcha);                           /* fraccion del ciclo en apoyo */
int  robot_marcha(const robot_cfg_t *c, float fase, robot_pata_t pata[ROBOT_PATAS], float *margen);
int  robot_validar(const robot_cfg_t *c, robot_val_t *v);
float robot_margen(const robot_pata_t pata[ROBOT_PATAS]);       /* mm; < 0 = el centro cae fuera del apoyo */
