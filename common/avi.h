#ifndef DART_AVI_H
#define DART_AVI_H
#include <stdint.h>
void dart_avi_header(uint8_t header[224],uint32_t frames,uint32_t span_us,uint32_t bytes,uint32_t movi_bytes,uint32_t biggest,uint32_t width,uint32_t height);
#endif
