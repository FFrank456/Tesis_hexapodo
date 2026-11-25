#pragma once
#include <stdbool.h>

/* Comandos del robot en la maestra (fase 1: calculo y simulacion, NUNCA envia CAN).
 * robot_comando usa strtok(NULL, " ,") para leer sus argumentos: se llama con la
 * primera palabra ya leida. Devuelve false si la palabra no es suya. */
void robot_iniciar(void);      /* carga la configuracion guardada */
bool robot_comando(const char *palabra);
void robot_sim_tick(void);
void robot_ayuda(void);
