#include "protocol.h"
#include <string.h>
void dart_put32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(i*8));}
uint32_t dart_get32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
uint16_t dart_crc16(const void *data,size_t length){
    const uint8_t *p=data;uint16_t c=0xFFFF;
    for(size_t i=0;i<length;i++){c^=(uint16_t)p[i]<<8;for(unsigned j=0;j<8;j++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}
    return c;
}
size_t dart_encode(uint8_t type,uint32_t seq,const void *data,size_t n,uint8_t out[DART_WIRE_MAX]){
    if(n>DART_PAYLOAD_MAX || (n&&!data))return 0;
    uint8_t raw[DART_PAYLOAD_MAX+10];raw[0]=DART_VERSION;raw[1]=type;raw[2]=(uint8_t)n;raw[3]=(uint8_t)(n>>8);dart_put32(raw+4,seq);
    if(n)memcpy(raw+8,data,n);uint16_t crc=dart_crc16(raw,n+8);raw[n+8]=(uint8_t)crc;raw[n+9]=(uint8_t)(crc>>8);
    size_t write=1,code_pos=0;uint8_t code=1;
    for(size_t i=0;i<n+10;i++){
        if(raw[i]==0){out[code_pos]=code;code_pos=write++;code=1;}
        else{out[write++]=raw[i];code++;}
    }
    out[code_pos]=code;out[write++]=0;return write;
}
int dart_parse(dart_parser *p,uint8_t b,dart_packet *out){
    if(b){if(p->count<sizeof(p->data)&&!p->discard)p->data[p->count++]=b;else p->discard=1;return 0;}
    if(p->discard){p->discard=p->count=0;p->errors++;return -1;}
    if(!p->count)return 0;
    uint8_t raw[DART_PAYLOAD_MAX+10];unsigned r=0,w=0,n=p->count;p->count=0;
    while(r<n){unsigned code=p->data[r++];if(!code||r+code-1>n)goto bad;
        for(unsigned j=1;j<code;j++){if(w>=sizeof(raw))goto bad;raw[w++]=p->data[r++];}
        if(code!=255&&r<n){if(w>=sizeof(raw))goto bad;raw[w++]=0;}
    }
    if(w<10||raw[0]!=DART_VERSION)goto bad;
    unsigned length=raw[2]|((unsigned)raw[3]<<8);
    if(length>DART_PAYLOAD_MAX||w!=length+10)goto bad;
    if(dart_crc16(raw,w-2)!=(uint16_t)(raw[w-2]|((uint16_t)raw[w-1]<<8)))goto bad;
    out->type=raw[1];out->seq=dart_get32(raw+4);out->length=(uint16_t)length;if(length)memcpy(out->data,raw+8,length);return 1;
bad:p->errors++;return -1;
}
