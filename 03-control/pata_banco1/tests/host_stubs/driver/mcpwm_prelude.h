#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct fake_gen { int gpio; int force; uint32_t cmp; } *mcpwm_gen_handle_t;
typedef struct fake_cmp { uint32_t value; } *mcpwm_cmpr_handle_t;
typedef void *mcpwm_oper_handle_t; typedef void *mcpwm_timer_handle_t;
typedef struct { int dummy; } mcpwm_timer_event_data_t;
typedef bool (*mcpwm_timer_event_cb_t)(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t *, void *);
typedef struct { mcpwm_timer_event_cb_t on_full, on_empty, on_stop; } mcpwm_timer_event_callbacks_t;
typedef struct { int group_id, clk_src; uint32_t resolution_hz, period_ticks; int count_mode; } mcpwm_timer_config_t;
typedef struct { int group_id; struct { int update_gen_action_on_tez; } flags; } mcpwm_operator_config_t;
typedef struct { struct { int update_cmp_on_tez; } flags; } mcpwm_comparator_config_t;
typedef struct { int gen_gpio_num; } mcpwm_generator_config_t;
#define MCPWM_TIMER_CLK_SRC_PLL160M 0
#define MCPWM_TIMER_COUNT_MODE_UP 0
#define MCPWM_TIMER_START_NO_STOP 0
#define MCPWM_GEN_TIMER_EVENT_ACTION(a,b,c) 0
#define MCPWM_GEN_COMPARE_EVENT_ACTION(a,b,c) 0
int mcpwm_new_timer(const mcpwm_timer_config_t *c, mcpwm_timer_handle_t *t);
int mcpwm_new_operator(const mcpwm_operator_config_t *c, mcpwm_oper_handle_t *o);
int mcpwm_operator_connect_timer(mcpwm_oper_handle_t o, mcpwm_timer_handle_t t);
int mcpwm_new_comparator(mcpwm_oper_handle_t o, const mcpwm_comparator_config_t *c, mcpwm_cmpr_handle_t *h);
int mcpwm_new_generator(mcpwm_oper_handle_t o, const mcpwm_generator_config_t *c, mcpwm_gen_handle_t *h);
int mcpwm_comparator_set_compare_value(mcpwm_cmpr_handle_t c, uint32_t v);
int mcpwm_generator_set_force_level(mcpwm_gen_handle_t g, int level, bool hold);
int mcpwm_timer_register_event_callbacks(mcpwm_timer_handle_t t, const mcpwm_timer_event_callbacks_t *c, void *u);
int mcpwm_timer_enable(mcpwm_timer_handle_t t);
int mcpwm_timer_start_stop(mcpwm_timer_handle_t t, int c);
static inline int mcpwm_generator_set_action_on_timer_event(mcpwm_gen_handle_t g, int a) { (void)g; (void)a; return 0; }
static inline int mcpwm_generator_set_action_on_compare_event(mcpwm_gen_handle_t g, int a) { (void)g; (void)a; return 0; }
