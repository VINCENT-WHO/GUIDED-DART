#ifndef DART_VISION_PROTOCOL_H
#define DART_VISION_PROTOCOL_H
/* USB wire: little-endian IEEE754 binary32. No structure serialization.
 * Board -> host: A5 5A | encoder-estimated mechanical yaw (4 bytes) | CRC16 = 8 bytes.
 * Host -> board: 5A A5 | absolute mechanical yaw in degrees (4 bytes) | CRC16 = 8 bytes.
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <float.h>
#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Vision protocol requires IEEE754 binary32 float"
#endif
#define DV_FEEDBACK_SIZE 8u
#define DV_YAW_SIZE 8u
#define DV_MAX_SIZE DV_FEEDBACK_SIZE

typedef struct { uint8_t bytes[DV_MAX_SIZE]; size_t count; } DVParser;
static inline uint16_t DVRead16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | (uint16_t)p[1]<<8); }
static inline uint32_t DVRead32(const uint8_t *p)
{ return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24; }
static inline void DVWrite16(uint8_t *p,uint16_t v)
{ p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8); }
static inline float DVReadFloat(const uint8_t *p)
{ uint32_t v=DVRead32(p);float f;memcpy(&f,&v,4);return f; }
static inline void DVWriteFloat(uint8_t *p,float f)
{ uint32_t v;memcpy(&v,&f,4);for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(v>>(8*i)); }
/* CRC-16/CCITT-FALSE: poly=1021, init=FFFF, refin/refout=false, xorout=0000. */
static inline uint16_t DVCRC(const uint8_t *p,size_t size)
{
    uint16_t crc=0xffffu;
    for(size_t i=0;i<size;++i){crc^=(uint16_t)p[i]<<8;
        for(unsigned b=0;b<8;++b)crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:(uint32_t)crc<<1);}
    return crc;
}
static inline void DVEncodeYaw(float yaw,uint8_t *p)
{ p[0]=0x5a;p[1]=0xa5;DVWriteFloat(p+2,yaw);DVWrite16(p+6,DVCRC(p,6)); }
static inline int DVDecodeYaw(const uint8_t *p,float *yaw)
{
    if(p[0]!=0x5a || p[1]!=0xa5 || DVRead16(p+6)!=DVCRC(p,6))return 0;
    *yaw=DVReadFloat(p+2);return 1;
}
static inline void DVEncodeYawFeedback(float yaw,uint8_t *p)
{ p[0]=0xa5;p[1]=0x5a;DVWriteFloat(p+2,yaw);DVWrite16(p+6,DVCRC(p,6)); }
static inline int DVDecodeYawFeedback(const uint8_t *p,float *yaw)
{
    if(p[0]!=0xa5 || p[1]!=0x5a || DVRead16(p+6)!=DVCRC(p,6))return 0;
    *yaw=DVReadFloat(p+2);return 1;
}
static inline int DVPushYaw(DVParser *p,uint8_t byte,float *yaw)
{
    p->bytes[p->count++]=byte;
    if(p->count<DV_YAW_SIZE)return 0;
    if(DVDecodeYaw(p->bytes,yaw)){p->count=0;return 1;}
    memmove(p->bytes,p->bytes+1,DV_YAW_SIZE-1);p->count=DV_YAW_SIZE-1;return 0;
}
static inline int DVPushYawFeedback(DVParser *p,uint8_t byte,float *yaw)
{
    p->bytes[p->count++]=byte;
    if(p->count<DV_FEEDBACK_SIZE)return 0;
    if(DVDecodeYawFeedback(p->bytes,yaw)){p->count=0;return 1;}
    memmove(p->bytes,p->bytes+1,DV_FEEDBACK_SIZE-1);p->count=DV_FEEDBACK_SIZE-1;return 0;
}
#endif
