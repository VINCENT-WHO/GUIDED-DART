/* Continuous JPEG frame queue, adapted from the supplied board-verified driver. */
#include "stm32h7xx_hal.h"
#include "camera_perf.h"
#include <string.h>
#include <stdio.h>
extern uint32_t board_micros(void);
uint8_t camera_workspace[512u*1024u] __attribute__((section(".camera_buffer"),aligned(32)));
static uint8_t d2_workspace[192u*1024u] __attribute__((section(".camera_d2"),aligned(32)));
#define CAPACITY (64u*1024u)
#define SLOTS 10u
static uint8_t *buffers[SLOTS];
static volatile unsigned active,current,state[SLOTS];
static uint32_t sizes[SLOTS],times[SLOTS];
volatile uint32_t camera_frames,camera_drops,camera_errors,camera_invalid;
static DMA_HandleTypeDef pdma;
static unsigned configured;
static int wr(unsigned a,unsigned b){return camera_write_register(a,b);}
static int pair(unsigned a,unsigned v){return wr(a,v>>8)||wr(a+1,v&255);}
static int change(unsigned a,unsigned mask,unsigned v){int old=camera_read_register(a);return old<0?-1:wr(a,((unsigned)old&~mask)|(v&mask));}
int camera_configure(void){
    configured=0;
    if(camera_perf_base())return -1;
    int e=0;e|=wr(0x3008,0x42);e|=wr(0x3039,0);e|=wr(0x3034,0x1A);
    e|=wr(0x3035,0x11);e|=wr(0x3036,0x78);e|=wr(0x3037,0x13);e|=wr(0x3108,1);
    e|=pair(0x3800,0);e|=pair(0x3802,4);e|=pair(0x3804,2623);e|=pair(0x3806,1951);
    e|=pair(0x3808,320);e|=pair(0x380A,240);e|=pair(0x380C,1892);e|=pair(0x380E,1015);
    e|=pair(0x3810,16);e|=pair(0x3812,6);e|=wr(0x3814,0x31);e|=wr(0x3815,0x31);
    e|=wr(0x3618,0);e|=wr(0x3612,0x29);e|=wr(0x3708,0x64);e|=wr(0x3709,0x52);e|=wr(0x370C,3);
    e|=change(0x3820,1,1);e|=change(0x3821,0x21,0x21);
    e|=wr(0x4300,0x30);e|=wr(0x501F,0);e|=wr(0x5001,0xA3);
    e|=change(0x3002,0x1C,0);e|=change(0x3006,0x28,0x28);
    e|=wr(0x4713,3);e|=wr(0x4407,12);e|=wr(0x440E,0);
    e|=wr(0x460B,0x37);e|=wr(0x460C,0x22);e|=wr(0x3824,4);e|=wr(0x4740,0x23);
    e|=pair(0x4602,320);e|=pair(0x4604,240);
    e|=wr(0x4001,2);e|=wr(0x4004,2);e|=change(0x3A00,4,0);e|=wr(0x3503,0);
    e|=pair(0x3A02,1011);e|=pair(0x3A14,1011);
    /* Recalculate anti-flicker exposure steps for the 96 MHz / 1892 HTS
     * timing. Inheriting the low-clock BSP's limits can extend exposure. */
    e|=pair(0x3A08,507);e|=pair(0x3A0A,423);e|=wr(0x3A0E,1);e|=wr(0x3A0D,2);
    e|=pair(0x350C,0);e|=wr(0x503D,0);e|=wr(0x3008,2);HAL_Delay(600);
    configured=e==0;return e;
}
void camera_timing(void){
    static const unsigned regs[]={0x3034,0x3035,0x3036,0x3037,0x3108,
        0x3500,0x3501,0x3502,0x3503,0x350A,0x350B,0x350C,0x350D,
        0x380C,0x380D,0x380E,0x380F,0x3A00,0x3A02,0x3A03,
        0x3A08,0x3A09,0x3A0A,0x3A0B,0x3A0D,0x3A0E,0x3A14,0x3A15,0x460C,0x3824};
    char line[64];
    for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);i++){
        int value=camera_read_register(regs[i]);
        snprintf(line,sizeof(line),"CAM TIMING reg=%04X value=%d\r\n",regs[i],value);bench_puts(line);
    }
    bench_puts("CAM TIMING END\r\n");
}
static void dma_off(void){DMA2_Stream1->CR&=~DMA_SxCR_EN;uint32_t t=DWT->CYCCNT;while(DMA2_Stream1->CR&DMA_SxCR_EN)if((uint32_t)(DWT->CYCCNT-t)>SystemCoreClock/1000){camera_errors|=8;break;}}
static void arm(unsigned slot){DMA2->LIFCR=0xF40;DMA2_Stream1->M0AR=(uint32_t)buffers[slot];DMA2_Stream1->NDTR=CAPACITY/4;__DMB();DMA2_Stream1->CR|=DMA_SxCR_EN;}
int perf_dma_irq(void){return active?1:0;}
int perf_dcmi_irq(void){
    if(!active)return 0;
    uint32_t f=DCMI->MISR;DCMI->ICR=f;
    if(f&DCMI_RIS_OVR_RIS)camera_errors|=1;
    if(f&DCMI_RIS_ERR_RIS)camera_errors|=2;
    if(f&DCMI_RIS_FRAME_RIS){
        camera_frames++;uint32_t t=DWT->CYCCNT;
        while((DCMI->SR&DCMI_SR_FNE)&&((uint32_t)(DWT->CYCCNT-t)<SystemCoreClock/10000)){}
        dma_off();__DSB();uint32_t n=(CAPACITY/4-DMA2_Stream1->NDTR)*4;unsigned ok=0;
        if(n>=4&&n<=CAPACITY&&buffers[current][0]==255&&buffers[current][1]==216){
            uint32_t low=n>64?n-64:2;
            for(uint32_t i=n-1;i>low;i--)if(buffers[current][i-1]==255&&buffers[current][i]==217){n=i+1;ok=1;break;}
        }
        if(ok){unsigned next=SLOTS;for(unsigned i=0;i<SLOTS;i++)if(state[i]==0){next=i;break;}
            if(next<SLOTS){sizes[current]=n;times[current]=board_micros();__DMB();state[current]=2;current=next;state[current]=1;}
            else camera_drops++;
        }else camera_invalid++;
        arm(current);
    }
    return 1;
}
int camera_start(void){
    if(!configured)return -1;
    __HAL_RCC_D2SRAM1_CLK_ENABLE();__HAL_RCC_D2SRAM2_CLK_ENABLE();__HAL_RCC_D2SRAM3_CLK_ENABLE();
    camera_perf_bus();unsigned n=0;for(unsigned i=0;i<7;i++)buffers[n++]=camera_workspace+i*CAPACITY;
    for(unsigned i=0;i<3;i++)buffers[n++]=d2_workspace+i*CAPACITY;
    memset(&pdma,0,sizeof(pdma));pdma.Instance=DMA2_Stream1;pdma.Init.Request=DMA_REQUEST_DCMI;
    pdma.Init.Direction=DMA_PERIPH_TO_MEMORY;pdma.Init.MemInc=DMA_MINC_ENABLE;pdma.Init.PeriphInc=DMA_PINC_DISABLE;
    pdma.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;pdma.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
    pdma.Init.Mode=DMA_NORMAL;pdma.Init.Priority=DMA_PRIORITY_VERY_HIGH;pdma.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
    if(HAL_DMA_Init(&pdma)!=HAL_OK)return -1;
    DMA2_Stream1->PAR=(uint32_t)&DCMI->DR;
    DMA2_Stream1->CR&=~(DMA_SxCR_TCIE|DMA_SxCR_HTIE|DMA_SxCR_TEIE|DMA_SxCR_DMEIE);
    /* Board-verified polarity combination 5. */
    DCMI->CR=DCMI_CR_JPEG|DCMI_CR_PCKPOL|DCMI_CR_HSPOL;
    DCMI->ICR=0x1F;DCMI->IER=DCMI_IER_FRAME_IE|DCMI_IER_OVR_IE|DCMI_IER_ERR_IE;
    memset((void*)state,0,sizeof(state));current=0;state[0]=1;camera_frames=camera_drops=camera_errors=camera_invalid=0;
    active=1;arm(0);HAL_NVIC_SetPriority(DCMI_IRQn,2,0);HAL_NVIC_ClearPendingIRQ(DCMI_IRQn);HAL_NVIC_EnableIRQ(DCMI_IRQn);
    DCMI->CR|=DCMI_CR_ENABLE|DCMI_CR_CAPTURE;return 0;
}
void camera_stop(void){HAL_NVIC_DisableIRQ(DCMI_IRQn);DCMI->IER=0;DCMI->CR&=~(DCMI_CR_CAPTURE|DCMI_CR_ENABLE);dma_off();active=0;DCMI->ICR=0x1F;}
int camera_take(const uint8_t **bytes,uint32_t *length,uint32_t *time){
    int best=-1;uint32_t irq=__get_PRIMASK();__disable_irq();
    for(unsigned i=0;i<SLOTS;i++)if(state[i]==2&&(best<0||(int32_t)(times[i]-times[best])<0))best=(int)i;
    if(best>=0){state[best]=3;*bytes=buffers[best];*length=sizes[best];*time=times[best];}
    __set_PRIMASK(irq);return best;
}
void camera_release(unsigned slot){if(slot<SLOTS){__DMB();state[slot]=0;}}
