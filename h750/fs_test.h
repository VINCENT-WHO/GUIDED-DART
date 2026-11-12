#pragma once
#include <stdint.h>
void fs_disk_set_ready(uint32_t ready,uint32_t sectors);
extern volatile uint32_t fs_disk_write_enabled,fs_disk_last_error;
extern volatile uint32_t fs_disk_reads,fs_disk_writes,fs_disk_read_sectors,fs_disk_write_sectors;
void bench_puts(const char *s);
