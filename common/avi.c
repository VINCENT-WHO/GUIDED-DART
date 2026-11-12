#include "avi.h"
#include "protocol.h"
#include <string.h>
void dart_avi_header(uint8_t h[224],uint32_t n,uint32_t us,uint32_t bytes,uint32_t movi,uint32_t biggest,uint32_t width,uint32_t height){
    memset(h,0,224);memcpy(h,"RIFF",4);dart_put32(h+4,bytes-8);memcpy(h+8,"AVI LIST",8);
    dart_put32(h+16,192);memcpy(h+20,"hdrlavih",8);dart_put32(h+28,56);
    dart_put32(h+32,n?us/n:20000);dart_put32(h+36,us?(uint32_t)((uint64_t)bytes*1000000/us):0);
    dart_put32(h+44,0x10);dart_put32(h+48,n);dart_put32(h+56,1);dart_put32(h+60,biggest);dart_put32(h+64,width);dart_put32(h+68,height);
    memcpy(h+88,"LIST",4);dart_put32(h+92,116);memcpy(h+96,"strlstrh",8);dart_put32(h+104,56);
    memcpy(h+108,"vidsMJPG",8);dart_put32(h+128,us?us:1);dart_put32(h+132,n?n*1000000:1);
    dart_put32(h+140,n);dart_put32(h+144,biggest);dart_put32(h+148,0xFFFFFFFF);
    h[160]=(uint8_t)width;h[161]=(uint8_t)(width>>8);h[162]=(uint8_t)height;h[163]=(uint8_t)(height>>8);
    memcpy(h+164,"strf",4);dart_put32(h+168,40);dart_put32(h+172,40);dart_put32(h+176,width);dart_put32(h+180,height);
    dart_put32(h+184,0x00180001);memcpy(h+188,"MJPG",4);dart_put32(h+192,biggest);
    memcpy(h+212,"LIST",4);dart_put32(h+216,movi+4);memcpy(h+220,"movi",4);
}
