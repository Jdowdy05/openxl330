#ifndef OBSERVABILITY_H
#define OBSERVABILITY_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {uint32_t end_sample,end_latest,end_max,post_sample,post_latest,post_max,overwritten,late,uart_latest,uart_max,origin;uint16_t uart_entries,uart_entries_max,flags,pad;} obs_state;
typedef struct {volatile uint32_t *base;uint16_t free_bytes,span_bytes,flags,sweeps;} obs_watermark;
extern obs_state observation;
extern obs_watermark observation_stack;
extern volatile uint32_t observation_activity,observation_uart_cycles,observation_uart_entries;
void obs_publish(obs_state *,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
bool obs_consume(obs_state *,uint32_t);
void obs_serialize(const obs_state *,uint32_t,uint16_t,uint16_t,uint16_t,uint16_t,uint8_t *);
void obs_watermark_init(obs_watermark *,volatile uint32_t *,uint16_t);
uint16_t obs_watermark_measure(const obs_watermark *,bool *);
void obs_watermark_commit(obs_watermark *,uint16_t,bool,bool,bool);
uint16_t obs_watermark_scan(obs_watermark *,uint32_t,uint32_t,bool,bool);
#endif
