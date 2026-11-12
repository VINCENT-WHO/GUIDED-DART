#pragma once
#include <stdint.h>
extern uint8_t camera_workspace[];
extern void bench_puts(const char *s);
int perf_dcmi_irq(void);
int perf_dma_irq(void);
int camera_perf_base(void);
int camera_read_register(unsigned reg);
int camera_write_register(unsigned reg,unsigned value);
void camera_perf_bus(void);
int camera_configure(void);
void camera_timing(void);
int camera_start(void);
void camera_stop(void);
int camera_take(const uint8_t **bytes,uint32_t *length,uint32_t *time_us);
void camera_release(unsigned slot);
extern volatile uint32_t camera_frames,camera_drops,camera_errors,camera_invalid;
