/* Board-specific electrical camera test, not a flight/vision application.
 * SCCB PB8/PB9, XCLK PA8, RESET PA10, PWDN PD7 (physical H750 pin 88).
 * ST OV5640 BSP component retained unmodified in ../vendor.
 * DMA buffer is AXI SRAM, never DTCM. Caches remain disabled in this test.
 */
#include "stm32h7xx_hal.h"
#include "ov5640.h"
#include "camera_test.h"
#include "camera_perf.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

extern void bench_puts(const char *s);
static OV5640_Object_t sensor;
static DCMI_HandleTypeDef dcmi;
static DMA_HandleTypeDef dma;
#define pixels (camera_workspace)
static uint32_t width=160,height=120,valid_bytes,frame_crc,frame_number,probed,initialized,clock_mhz=24;
static uint32_t bus_errors,last_reg,last_phase;
static volatile uint32_t frame_event,capture_error,dma_complete;

static void say(const char *fmt,...) {
    char s[120]; va_list ap; va_start(ap,fmt); vsnprintf(s,sizeof(s),fmt,ap); va_end(ap); bench_puts(s);
}
static void us(uint32_t n) {
    uint32_t begin=DWT->CYCCNT, cycles=(SystemCoreClock/1000000U)*n;
    while ((uint32_t)(DWT->CYCCNT-begin)<cycles) {}
}
static void sda(unsigned high) { GPIOB->BSRR=high?GPIO_PIN_9:((uint32_t)GPIO_PIN_9<<16); }
static int scl(unsigned high) {
    GPIOB->BSRR=high?GPIO_PIN_8:((uint32_t)GPIO_PIN_8<<16);
    if (high) {
        uint32_t begin=DWT->CYCCNT;
        while (!(GPIOB->IDR&GPIO_PIN_8))
            if ((uint32_t)(DWT->CYCCNT-begin)>SystemCoreClock/1000U) return -1;
    }
    us(5); return 0;
}
static void stop(void) { sda(0);us(5);(void)scl(1);sda(1);us(5); }
static int start(void) {
    sda(1);if(scl(1))return -1;
    if(!(GPIOB->IDR&GPIO_PIN_9))return -1;
    sda(0);us(5);return scl(0);
}
static int send(uint8_t v) {
    for(unsigned i=0;i<8;i++) {sda(v&0x80U);us(5);if(scl(1))return -1;(void)scl(0);v<<=1;}
    sda(1);us(5);if(scl(1))return -1;
    int nack=(GPIOB->IDR&GPIO_PIN_9)!=0;(void)scl(0);return nack?-1:0;
}
static int receive(uint8_t *v,unsigned nack) {
    *v=0;sda(1);
    for(unsigned i=0;i<8;i++) {if(scl(1))return -1;*v=(uint8_t)((*v<<1)|((GPIOB->IDR>>9)&1));(void)scl(0);}
    sda(nack);if(scl(1))return -1;(void)scl(0);sda(1);return 0;
}
static int fail(uint32_t phase) {last_phase=phase;++bus_errors;stop();return -1;}
static int32_t bus_write(uint16_t addr,uint16_t reg,uint8_t *d,uint16_t n) {
    last_reg=reg;
    if(start())return fail(1);
    if(send((uint8_t)addr))return fail(2);
    if(send(reg>>8))return fail(3);
    if(send(reg&255))return fail(4);
    for(unsigned i=0;i<n;i++)if(send(d[i]))return fail(5);
    stop();last_phase=0;
    /* Sensor reset requires a settling interval before the next SCCB write. */
    if(reg==0x3008 && n && (d[0]&0x80)) HAL_Delay(10);
    return 0;
}
static int32_t bus_read(uint16_t addr,uint16_t reg,uint8_t *d,uint16_t n) {
    last_reg=reg;
    if(start())return fail(1);
    if(send((uint8_t)addr))return fail(2);
    if(send(reg>>8))return fail(3);
    if(send(reg&255))return fail(4);
    stop();us(5);
    if(start())return fail(6);
    if(send((uint8_t)(addr|1)))return fail(7);
    for(unsigned i=0;i<n;i++)if(receive(&d[i],i+1==n))return fail(8);
    stop();last_phase=0;return 0;
}
static int32_t nothing(void) {return 0;}
static int32_t ticks(void) {return (int32_t)HAL_GetTick();}
static int rd(uint16_t reg) {uint8_t v;if(bus_read(0x78,reg,&v,1))return -1;return v;}
static int wr(uint16_t reg,uint8_t v) {return bus_write(0x78,reg,&v,1);}

static void clock_set(uint32_t mhz) {
    /* PLL1Q is 48MHz. MCO1 /2 -> 24MHz or /4 -> 12MHz. */
    HAL_RCC_MCOConfig(RCC_MCO1,RCC_MCO1SOURCE_PLL1QCLK,mhz==12?RCC_MCODIV_4:RCC_MCODIV_2);
    clock_mhz=mhz;
}
static void gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOE_CLK_ENABLE();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    GPIO_InitTypeDef g={0};g.Mode=GPIO_MODE_OUTPUT_OD;g.Pull=GPIO_NOPULL;g.Speed=GPIO_SPEED_FREQ_LOW;
    g.Pin=GPIO_PIN_8|GPIO_PIN_9;GPIOB->BSRR=g.Pin;HAL_GPIO_Init(GPIOB,&g);
    g.Mode=GPIO_MODE_OUTPUT_PP;g.Pin=GPIO_PIN_10;GPIOA->BSRR=(uint32_t)g.Pin<<16;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_7;GPIOD->BSRR=g.Pin;HAL_GPIO_Init(GPIOD,&g);
    clock_set(clock_mhz);HAL_Delay(10);
    GPIOD->BSRR=(uint32_t)GPIO_PIN_7<<16;HAL_Delay(10);
    GPIOA->BSRR=GPIO_PIN_10;HAL_Delay(30);
    /* Recover only if SDA is stuck low; open-drain SCL cannot force high. */
    if(!(GPIOB->IDR&GPIO_PIN_9)) {for(unsigned i=0;i<9;i++){(void)scl(0);(void)scl(1);}stop();}
}
static void probe(unsigned check_rw) {
    initialized=probed=valid_bytes=0;bus_errors=0;gpio_init();
    say("CAM BUS scl=%lu sda=%lu xclk=%lu reset=1 pwdn=0\r\n",(GPIOB->IDR>>8)&1UL,(GPIOB->IDR>>9)&1UL,(unsigned long)clock_mhz);
    unsigned good=0;
    for(unsigned i=0;i<10;i++) {
        int hi=rd(0x300A),lo=rd(0x300B);
        if(hi==0x56&&lo==0x40)good++;
        say("CAM ID n=%u hi=%d lo=%d valid=%u err=%lu phase=%lu\r\n",i,hi,lo,hi==0x56&&lo==0x40,(unsigned long)bus_errors,(unsigned long)last_phase);
        HAL_Delay(5);
    }
    int old=-1,test=-1,back=-1,write_ok=0;
    if(good==10) {
        if(check_rw) {
            old=rd(0x503D);
            if(old>=0 && !wr(0x503D,(uint8_t)(old^0x80))) {
                test=rd(0x503D);(void)wr(0x503D,(uint8_t)old);back=rd(0x503D);
                write_ok=test==(old^0x80)&&back==old;
            }
        }
        probed=1;
    }
    say("CAM REGTEST requested=%u old=%d test=%d restored=%d ok=%d\r\n",check_rw,old,test,back,write_ok);
    say("CAM PROBE END ids=%u/10 rw=%d errors=%lu\r\n",good,write_ok,(unsigned long)bus_errors);
}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef *h) {
    if(h->Instance!=DCMI)return;
    __HAL_RCC_DCMI_CLK_ENABLE();__HAL_RCC_DMA2_CLK_ENABLE();
    GPIO_InitTypeDef g={0};g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_NOPULL;
    g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;g.Alternate=GPIO_AF13_DCMI;
    g.Pin=GPIO_PIN_4|GPIO_PIN_6;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_6|GPIO_PIN_7;HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_6|GPIO_PIN_7;HAL_GPIO_Init(GPIOC,&g);
    g.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;HAL_GPIO_Init(GPIOE,&g);
    dma.Instance=DMA2_Stream1;dma.Init.Request=DMA_REQUEST_DCMI;
    dma.Init.Direction=DMA_PERIPH_TO_MEMORY;dma.Init.PeriphInc=DMA_PINC_DISABLE;dma.Init.MemInc=DMA_MINC_ENABLE;
    dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;dma.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
    dma.Init.Mode=DMA_NORMAL;dma.Init.Priority=DMA_PRIORITY_HIGH;
    dma.Init.FIFOMode=DMA_FIFOMODE_ENABLE;dma.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;
    dma.Init.MemBurst=DMA_MBURST_SINGLE;dma.Init.PeriphBurst=DMA_PBURST_SINGLE;
    (void)HAL_DMA_Init(&dma);__HAL_LINKDMA(h,DMA_Handle,dma);
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn,1,0);HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_SetPriority(DCMI_IRQn,1,0);HAL_NVIC_EnableIRQ(DCMI_IRQn);
}
void DMA2_Stream1_IRQHandler(void) {if(!perf_dma_irq())HAL_DMA_IRQHandler(&dma);}
void DCMI_IRQHandler(void) {if(!perf_dcmi_irq()) HAL_DCMI_IRQHandler(&dcmi);}
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h) {(void)h;frame_event++;}
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *h) {capture_error=h->ErrorCode;}
static void dcmi_init(unsigned polarity) {
    /* Remove stale FIFO/FRAME interrupt state from a previous failed snapshot. */
    HAL_NVIC_DisableIRQ(DCMI_IRQn);HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
    if(dcmi.Instance) {
        (void)HAL_DCMI_Stop(&dcmi);
        (void)HAL_DCMI_DeInit(&dcmi);
        (void)HAL_DMA_DeInit(&dma);
    }
    __HAL_RCC_DCMI_FORCE_RESET();__HAL_RCC_DCMI_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(DCMI_IRQn);HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
    memset(&dcmi,0,sizeof(dcmi));memset(&dma,0,sizeof(dma));
    dcmi.Instance=DCMI;dcmi.Init.SynchroMode=DCMI_SYNCHRO_HARDWARE;
    dcmi.Init.PCKPolarity=(polarity&1)?DCMI_PCKPOLARITY_FALLING:DCMI_PCKPOLARITY_RISING;
    dcmi.Init.VSPolarity=(polarity&2)?DCMI_VSPOLARITY_LOW:DCMI_VSPOLARITY_HIGH;
    dcmi.Init.HSPolarity=(polarity&4)?DCMI_HSPOLARITY_HIGH:DCMI_HSPOLARITY_LOW;
    dcmi.Init.CaptureRate=DCMI_CR_ALL_FRAME;dcmi.Init.ExtendedDataMode=DCMI_EXTEND_DATA_8B;
    dcmi.Init.JPEGMode=DCMI_JPEG_DISABLE;dcmi.Init.ByteSelectMode=DCMI_BSM_ALL;
    dcmi.Init.ByteSelectStart=DCMI_OEBS_ODD;dcmi.Init.LineSelectMode=DCMI_LSM_ALL;
    dcmi.Init.LineSelectStart=DCMI_OELS_ODD;(void)HAL_DCMI_Init(&dcmi);
}
static uint32_t crc32(const uint8_t *d,uint32_t n) {
    uint32_t crc=0xffffffffU;for(uint32_t i=0;i<n;i++) {crc^=d[i];for(unsigned b=0;b<8;b++)crc=(crc>>1)^((crc&1)?0xEDB88320U:0);}
    return ~crc;
}
static void init(unsigned res) {
    if(!probed){say("CAM INIT REJECT need_successful_PROBE\r\n");return;}
    initialized=valid_bytes=0;memset(&sensor,0,sizeof(sensor));
    OV5640_IO_t io={0};io.Init=nothing;io.DeInit=nothing;io.Address=0x78;io.WriteReg=bus_write;io.ReadReg=bus_read;io.GetTick=ticks;
    int ret=OV5640_RegisterBusIO(&sensor,&io);
    if(!ret)ret=OV5640_Init(&sensor,res?OV5640_R320x240:OV5640_R160x120,OV5640_RGB565);
    if(!ret)ret=OV5640_SetPCLK(&sensor,OV5640_PCLK_7M);
    /* Datasheet 0x4740: PCLK active high, HREF high, VSYNC high.
     * Set explicitly: BSP polarity constant names do not convey this mapping.
     */
    if(!ret)ret=wr(0x4740,0x23);
    if(!ret)ret=OV5640_Start(&sensor);
    if(!ret){initialized=1;width=res?320:160;height=res?240:120;dcmi_init(0);}
    HAL_Delay(500);
    say("CAM INIT result=%d w=%lu h=%lu errors=%lu last=%04lX phase=%lu\r\n",ret,(unsigned long)width,(unsigned long)height,(unsigned long)bus_errors,(unsigned long)last_reg,(unsigned long)last_phase);
}
static void captured(DMA_HandleTypeDef *h) {
    if(((DMA_Stream_TypeDef *)h->Instance)->NDTR!=0)return;
    /* Freeze exactly the programmed image bytes before FIFO can overrun when
     * a NORMAL DMA stream stops. Do not wait for a later frame interrupt.
     */
    DCMI->CR&=~(DCMI_CR_CAPTURE|DCMI_CR_ENABLE);
    __DSB();dma_complete=1;
}
static void snapshot(unsigned polarity,unsigned verbose) {
    if(!initialized){say("CAM SNAP REJECT need_successful_INIT\r\n");return;}
    valid_bytes=0;frame_event=capture_error=dma_complete=0;uint32_t size=width*height*2;
    memset(pixels,0xA5,size);dcmi_init(polarity);DCMI->ICR=0x1FU;__DSB();
    uint32_t begin=HAL_GetTick();
    (void)HAL_DCMI_ConfigCrop(&dcmi,0,0,width*2-1,height-1);
    (void)HAL_DCMI_EnableCrop(&dcmi);
    __disable_irq();
    int ret=HAL_DCMI_Start_DMA(&dcmi,DCMI_MODE_SNAPSHOT,(uint32_t)pixels,size/4);
    dma.XferCpltCallback=captured;
    __enable_irq();
    while(!ret&&!dma_complete&&!capture_error&&!dcmi.ErrorCode&&HAL_GetTick()-begin<3000){}
    __DSB();uint32_t ndtr=((DMA_Stream_TypeDef *)dma.Instance)->NDTR,ris=DCMI->RISR,derr=dcmi.ErrorCode,dmaerr=dma.ErrorCode;
    uint32_t done_before_stop=dma_complete;
    (void)HAL_DCMI_Stop(&dcmi);
    if(!ret&&done_before_stop&&!capture_error&&!derr&&!dmaerr&&ndtr==0)valid_bytes=size;
    frame_crc=crc32(pixels,size);frame_number++;
    uint8_t bits_or=0,bits_and=255;
    for(uint32_t i=0;i<size;i++){bits_or|=pixels[i];bits_and&=pixels[i];}
    if(!verbose)return;
    say("CAM SNAP n=%lu ok=%lu bytes=%lu crc=%08lX ms=%lu pol=%u\r\n",(unsigned long)frame_number,(unsigned long)(valid_bytes!=0),(unsigned long)valid_bytes,(unsigned long)frame_crc,(unsigned long)(HAL_GetTick()-begin),polarity);
    say("CAM DMA ret=%d frames=%lu err=%08lX dmaerr=%08lX left=%lu ris=%08lX\r\n",ret,(unsigned long)frame_event,(unsigned long)(capture_error|derr),(unsigned long)dmaerr,(unsigned long)ndtr,(unsigned long)ris);
    say("CAM COMPLETE dma=%lu polreg=%02X crop_w=%lu crop_h=%lu\r\n",(unsigned long)done_before_stop,rd(0x4740),(unsigned long)(width*2),(unsigned long)height);
    say("CAM BITS or=%02X and=%02X changed=%02X\r\n",bits_or,bits_and,bits_or^bits_and);
    say("CAM SNAP END\r\n");
}
static void signals(void) {
    uint32_t p=0,h=0,v=0,old=0,first=1,begin=HAL_GetTick(),seen_or=0,seen_and=7;
    uint32_t total=0,ph=0,hh=0,vh=0,counts[4]={0};uint8_t ors[4]={0},ands[4]={255,255,255,255};
    while(HAL_GetTick()-begin<200) {
        uint32_t a=GPIOA->IDR,b=GPIOB->IDR,c=GPIOC->IDR,e=GPIOE->IDR;
        uint32_t s=((a>>6)&1)|(((a>>4)&1)<<1)|(((b>>7)&1)<<2);
        total++;ph+=(s&1)!=0;hh+=(s&2)!=0;vh+=(s&4)!=0;
        unsigned hv=s>>1;uint8_t d=(uint8_t)(((c>>6)&3)|((e&3)<<2)|((e&16))|((b&64)>>1)|((e&32)<<1)|((e&64)<<1));
        counts[hv]++;ors[hv]|=d;ands[hv]&=d;
        seen_or|=s;seen_and&=s;
        if(!first){uint32_t change=s^old;p+=(change&1)!=0;h+=(change&2)!=0;v+=(change&4)!=0;}
        old=s;first=0;
    }
    say("CAM SIGNAL sample_ms=200 pclk_edges=%lu href_edges=%lu vsync_edges=%lu or=%lX and=%lX\r\n",(unsigned long)p,(unsigned long)h,(unsigned long)v,(unsigned long)seen_or,(unsigned long)seen_and);
    say("CAM LEVEL n=%lu pclk_hi=%lu href_hi=%lu vsync_hi=%lu\r\n",(unsigned long)total,(unsigned long)ph,(unsigned long)hh,(unsigned long)vh);
    for(unsigned i=0;i<4;i++)say("CAM DATASTATE hv=%u n=%lu or=%02X and=%02X changed=%02X\r\n",i,(unsigned long)counts[i],ors[i],ands[i],ors[i]^ands[i]);
}
static void dump(void) {
    if(!valid_bytes){say("CAM DUMP REJECT no_valid_snapshot\r\n");return;}
    say("CAM IMAGE BEGIN n=%lu w=%lu h=%lu bytes=%lu crc=%08lX format=RGB565\r\n",(unsigned long)frame_number,(unsigned long)width,(unsigned long)height,(unsigned long)valid_bytes,(unsigned long)frame_crc);
    for(uint32_t off=0;off<valid_bytes;off+=32) {
        static const char hex[]="0123456789ABCDEF";char s[90];
        int k=snprintf(s,sizeof(s),"CAM DATA %06lX ",(unsigned long)off);
        uint32_t n=valid_bytes-off;if(n>32)n=32;
        for(uint32_t i=0;i<n;i++){uint8_t b=pixels[off+i];s[k++]=hex[b>>4];s[k++]=hex[b&15];}
        s[k++]='\r';s[k++]='\n';s[k]=0;bench_puts(s);HAL_Delay(6);
    }
    say("CAM IMAGE END n=%lu crc=%08lX\r\n",(unsigned long)frame_number,(unsigned long)frame_crc);
}
void camera_command(const char *cmd) {
    if(!strcmp(cmd,"ID"))probe(0);
    else if(!strcmp(cmd,"PROBE"))probe(1);
    else if(!strcmp(cmd,"INIT"))init(0);
    else if(!strcmp(cmd,"INIT320"))init(1);
    else if(!strncmp(cmd,"SNAP",4))snapshot(cmd[4]?((unsigned)strtoul(cmd+4,NULL,10)&7):6,1);
    else if(!strncmp(cmd,"VIDEO ",6)&&strlen(cmd)==12) {
        unsigned t=0;int valid=1;
        for(unsigned i=6;i<12;i++) {
            unsigned c=(unsigned char)cmd[i];
            if(c>='0'&&c<='9')c-='0';else if(c>='A'&&c<='F')c=c-'A'+10;else {valid=0;break;}
            t=(t<<4)|c;
        }
        (void)t;(void)valid;
        say("CAM VIDEO use unified START command\r\n");
    }
    else if(!strcmp(cmd,"SIGNAL"))signals();
    else if(!strcmp(cmd,"DUMP"))dump();
    else if(!strcmp(cmd,"BARS ON")||!strcmp(cmd,"BARS OFF")) {
        int ret=initialized?OV5640_ColorbarModeConfig(&sensor,cmd[5]=='O'&&cmd[6]=='N'?COLORBAR_MODE_ENABLE:COLORBAR_MODE_DISABLE):-1;
        HAL_Delay(200);say("CAM BARS result=%d\r\n",ret);
    }
    else if(!strcmp(cmd,"CLK12")) {if(probed)clock_set(12);else clock_mhz=12;say("CAM CLOCK MHz=12\r\n");}
    else if(!strcmp(cmd,"CLK24")) {if(probed)clock_set(24);else clock_mhz=24;say("CAM CLOCK MHz=24\r\n");}
    else if(!strncmp(cmd,"REG ",4)) {unsigned r=(unsigned)strtoul(cmd+4,NULL,16);say("CAM REG addr=%04X value=%d\r\n",r,probed?rd((uint16_t)r):-1);}
    else say("CAM COMMANDS PROBE INIT INIT320 BARS ON/OFF SNAP[0..7] SIGNAL DUMP REG CLK12/24\r\n");
    say("CAM COMMAND END\r\n");
}

int camera_video_prepare(unsigned *w,unsigned *h) {
    if(!initialized)return -1;
    if(OV5640_ColorbarModeConfig(&sensor,COLORBAR_MODE_DISABLE))return -1;
    HAL_Delay(300);*w=width;*h=height;return 0;
}
int camera_video_frame(const unsigned char **data,unsigned *bytes) {
    snapshot(6,0);
    if(!valid_bytes)return -1;
    *data=pixels;*bytes=valid_bytes;return 0;
}

/* Shared electrical setup; perf owns workspace only while the old capture is idle. */
int camera_perf_base(void) {probe(1);if(!probed)return -1;init(0);return initialized?0:-1;}
int camera_read_register(unsigned r) {return rd((uint16_t)r);}
int camera_write_register(unsigned r,unsigned v) {return wr((uint16_t)r,(uint8_t)v);}
void camera_perf_bus(void) {dcmi_init(6);HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);HAL_NVIC_DisableIRQ(DCMI_IRQn);(void)HAL_DMA_DeInit(&dma);}
