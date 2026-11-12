/* F103：读取 BMI088、向 H750 发送原始数据、保留手动串口调试。 */
#include "stm32f1xx.h"
#include "protocol.h"
#include "timebase.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#define ACC_CS (1u<<14)
#define GYR_CS (1u<<12)
#define TX_SIZE 4096u
#define LINK_RX_SIZE 2048u
static volatile uint32_t ticks;
static volatile uint8_t pc_rx[256],link_rx[LINK_RX_SIZE],link_tx[TX_SIZE];
static volatile unsigned pc_h,pc_t,rx_h,rx_t,tx_h,tx_t;
static volatile uint32_t spi_errors,uart_errors,tx_drops,missed_ticks,sequence;
static volatile uint32_t link_rx_count,link_tx_count;
static volatile uint8_t last_rx[32];
static volatile unsigned sampling;
static unsigned accel_ok,gyro_ok,accel_id,gyro_id,pwm_on;
static uint32_t pulses[4]={1500,1500,1500,1500},last_manual;
static uint32_t sampling_started;
/* WATCH 模式允许无限待机；普通 IMU START 仍保留原来的 12 秒保护。 */
static unsigned watching;
static uint32_t led_state,led_received;
static dart_parser parser;
static uint32_t msg_seq;
void _init(void){} void _fini(void){}
void SysTick_Handler(void){ticks++;}
static uint32_t micros(void){
    uint32_t irq=__get_PRIMASK();__disable_irq();uint32_t ms=ticks,cycles=SysTick->VAL,load=SysTick->LOAD;
    if(SCB->ICSR&SCB_ICSR_PENDSTSET_Msk){ms++;cycles=SysTick->VAL;}
    __set_PRIMASK(irq);return dart_systick_micros(ms,load,cycles);
}
static void delay_us(uint32_t us){uint32_t begin=DWT->CYCCNT;while((uint32_t)(DWT->CYCCNT-begin)<us*(SystemCoreClock/1000000u)){} }
static void delay_ms(uint32_t ms){uint32_t start=ticks;while((uint32_t)(ticks-start)<ms){} }
static unsigned wait_bits(volatile uint32_t *r,uint32_t bits){for(unsigned n=0;n<1000000;n++)if((*r&bits)==bits)return 1;return 0;}
static void pin(GPIO_TypeDef *p,unsigned n,unsigned mode){volatile uint32_t *r=n<8?&p->CRL:&p->CRH;unsigned s=(n&7)*4;*r=(*r&~(15u<<s))|(mode<<s);}
static void clock_init(void){
    SystemCoreClock=8000000;RCC->CR|=RCC_CR_HSEON;
    if(wait_bits(&RCC->CR,RCC_CR_HSERDY)){
        FLASH->ACR=FLASH_ACR_PRFTBE|FLASH_ACR_LATENCY_2;
        RCC->CFGR=RCC_CFGR_PPRE1_DIV2|RCC_CFGR_ADCPRE_DIV6|RCC_CFGR_PLLSRC|RCC_CFGR_PLLMULL6;
        RCC->CR|=RCC_CR_PLLON;if(wait_bits(&RCC->CR,RCC_CR_PLLRDY)){
            RCC->CFGR|=RCC_CFGR_SW_PLL;if(wait_bits(&RCC->CFGR,RCC_CFGR_SWS_PLL))SystemCoreClock=72000000;
        }
    }
    if(SystemCoreClock!=72000000){RCC->CFGR=0;RCC->CR&=~RCC_CR_PLLON;}
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;DWT->CYCCNT=0;
    SysTick_Config(SystemCoreClock/1000);NVIC_SetPriority(SysTick_IRQn,0);
}
static void pc_write(const char *s){while(*s){uint32_t t=micros();while(!(USART3->SR&USART_SR_TXE))if((uint32_t)(micros()-t)>5000){uart_errors++;return;}USART3->DR=(uint8_t)*s++;}}
static void report(const char *fmt,...){char s[220];va_list ap;va_start(ap,fmt);vsnprintf(s,sizeof(s),fmt,ap);va_end(ap);pc_write(s);}
static int packet(uint8_t type,const void *p,unsigned n){
    uint8_t wire[DART_WIRE_MAX];unsigned count=(unsigned)dart_encode(type,msg_seq++,p,n,wire);
    if(!count)return 0;
    uint32_t irq=__get_PRIMASK();__disable_irq();unsigned free=(tx_t-tx_h-1u)&(TX_SIZE-1u);
    if(count>free){tx_drops++;__set_PRIMASK(irq);return 0;}
    for(unsigned i=0;i<count;i++){link_tx[tx_h]=wire[i];tx_h=(tx_h+1u)&(TX_SIZE-1u);}
    __DMB();USART1->CR1|=USART_CR1_TXEIE;__set_PRIMASK(irq);return 1;
}
static void link_text(uint8_t type,const char *s){packet(type,s,(unsigned)strlen(s));}
void USART1_IRQHandler(void){
    uint32_t f=USART1->SR;
    if(f&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){
        uint8_t b=(uint8_t)USART1->DR;if(f&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))uart_errors++;
        if(f&USART_SR_RXNE){last_rx[link_rx_count&31]=b;link_rx_count++;unsigned next=(rx_h+1)&(LINK_RX_SIZE-1);if(next==rx_t)uart_errors++;else{link_rx[rx_h]=b;rx_h=next;}}
    }
    if((f&USART_SR_TXE)&&(USART1->CR1&USART_CR1_TXEIE)){
        if(tx_t!=tx_h){USART1->DR=link_tx[tx_t];link_tx_count++;tx_t=(tx_t+1)&(TX_SIZE-1);}else USART1->CR1&=~USART_CR1_TXEIE;
    }
}
void USART3_IRQHandler(void){
    uint32_t f=USART3->SR;if(f&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){
        uint8_t b=(uint8_t)USART3->DR;if(f&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))uart_errors++;
        if(f&USART_SR_RXNE){unsigned next=(pc_h+1)&255;if(next==pc_t)uart_errors++;else{pc_rx[pc_h]=b;pc_h=next;}}
    }
}
static void board_init(void){
    RCC->APB2ENR|=RCC_APB2ENR_AFIOEN|RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN|RCC_APB2ENR_USART1EN|RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR|=RCC_APB1ENR_USART3EN|RCC_APB1ENR_TIM2EN|RCC_APB1ENR_TIM3EN;
    AFIO->MAPR=AFIO_MAPR_SWJ_CFG_JTAGDISABLE|AFIO_MAPR_SPI1_REMAP;
    for(unsigned i=0;i<4;i++)pin(GPIOA,i,4);
    GPIOA->BSRR=(1u<<4)|(1u<<5)|(1u<<6);for(unsigned i=4;i<7;i++)pin(GPIOA,i,2);
    GPIOB->BSRR=ACC_CS|GYR_CS;pin(GPIOB,12,2);pin(GPIOB,14,2);
    pin(GPIOB,3,11);pin(GPIOB,4,4);pin(GPIOB,5,11);pin(GPIOB,13,4);pin(GPIOB,15,4);
    SPI1->CR1=SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_1|SPI_CR1_SPE; /* /8 =9MHz */
    pin(GPIOB,10,11);pin(GPIOB,11,4);
    uint32_t apb1=SystemCoreClock==72000000?36000000:SystemCoreClock;
    USART3->BRR=(apb1+57600)/115200;USART3->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
    pin(GPIOA,9,11);pin(GPIOA,10,4);
    USART1->BRR=(SystemCoreClock+460800)/921600;USART1->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
    /* At 921600 baud a byte arrives every10.8us. Link service must preempt
       the SPI sampling ISR to avoid losing SYNC/STOP packets. */
    NVIC_SetPriority(USART1_IRQn,0);NVIC_EnableIRQ(USART1_IRQn);NVIC_SetPriority(USART3_IRQn,3);NVIC_EnableIRQ(USART3_IRQn);
    TIM3->PSC=SystemCoreClock/1000000-1;TIM3->ARR=999;TIM3->EGR=TIM_EGR_UG;TIM3->SR=0;
    TIM3->DIER=TIM_DIER_UIE;NVIC_SetPriority(TIM3_IRQn,1);NVIC_EnableIRQ(TIM3_IRQn);
}
static int spi_wait(uint32_t mask,unsigned set){uint32_t t=DWT->CYCCNT;while(((SPI1->SR&mask)!=0)!=set)if((uint32_t)(DWT->CYCCNT-t)>SystemCoreClock/20000){spi_errors++;return 0;}return 1;}
static uint8_t xfer(uint8_t v){if(!spi_wait(SPI_SR_TXE,1))return 255;*(volatile uint8_t*)&SPI1->DR=v;if(!spi_wait(SPI_SR_RXNE,1))return 255;return *(volatile uint8_t*)&SPI1->DR;}
static void imu_read(uint32_t cs,uint8_t reg,uint8_t *b,unsigned n){GPIOB->BRR=cs;delay_us(1);xfer(reg|128);if(cs==ACC_CS)xfer(0);for(unsigned i=0;i<n;i++)b[i]=xfer(0);spi_wait(SPI_SR_BSY,0);GPIOB->BSRR=cs;delay_us(1);}
static uint8_t reg(uint32_t cs,uint8_t r){uint8_t v;imu_read(cs,r,&v,1);return v;}
static void wr(uint32_t cs,uint8_t r,uint8_t v){GPIOB->BRR=cs;delay_us(1);xfer(r&127);xfer(v);spi_wait(SPI_SR_BSY,0);GPIOB->BSRR=cs;delay_ms(2);}
static void imu_init(void){
    delay_ms(100);reg(ACC_CS,0);delay_ms(5);accel_id=reg(ACC_CS,0);gyro_id=reg(GYR_CS,0);
    if(accel_id==0x1E){wr(ACC_CS,0x7E,0xB6);delay_ms(50);reg(ACC_CS,0);delay_ms(5);wr(ACC_CS,0x7C,0);delay_ms(5);wr(ACC_CS,0x7D,4);delay_ms(50);wr(ACC_CS,0x40,0xAC);wr(ACC_CS,0x41,3);accel_ok=reg(ACC_CS,0)==0x1E&&reg(ACC_CS,0x40)==0xAC&&(reg(ACC_CS,0x41)&3)==3;}
    if(gyro_id==0x0F){wr(GYR_CS,0x14,0xB6);delay_ms(50);wr(GYR_CS,0x11,0);delay_ms(50);wr(GYR_CS,0x0F,0);wr(GYR_CS,0x10,2);wr(GYR_CS,0x15,0x80);gyro_ok=reg(GYR_CS,0)==0x0F&&(reg(GYR_CS,0x10)&7)==2;}
}
static int16_t s16(const uint8_t *p){return (int16_t)(p[0]|((uint16_t)p[1]<<8));}
void TIM3_IRQHandler(void){
    TIM3->SR=0;if(!sampling)return;
    dart_imu s={0};uint8_t b[9];uint32_t before=spi_errors,start=micros();
    s.time_us=start;s.sequence=sequence++;s.missed_ticks=missed_ticks;
    if(gyro_ok){uint8_t fresh=reg(GYR_CS,0x0A);imu_read(GYR_CS,2,b,6);for(unsigned i=0;i<3;i++)s.gyro[i]=s16(b+i*2);s.flags|=IMU_GYRO_VALID;if(fresh&128)s.flags|=IMU_GYRO_FRESH;}
    if(accel_ok){uint8_t fresh=reg(ACC_CS,3);imu_read(ACC_CS,0x12,b,9);for(unsigned i=0;i<3;i++)s.accel[i]=s16(b+i*2);s.accel_sensor_time=b[6]|((uint32_t)b[7]<<8)|((uint32_t)b[8]<<16);s.flags|=IMU_ACCEL_VALID;if(fresh&128)s.flags|=IMU_ACCEL_FRESH;}
    if(before!=spi_errors){s.flags=IMU_SPI_ERROR;}
    if((uint32_t)(micros()-start)>800){s.flags|=IMU_SCHEDULE_LATE;missed_ticks++;}
    packet(DART_IMU,&s,sizeof(s));
    if(TIM3->SR&TIM_SR_UIF)missed_ticks++;
}
static void sampling_set(unsigned on){TIM3->CR1=0;sampling=0;if(on&&(accel_ok||gyro_ok)){sequence=missed_ticks=0;TIM3->CNT=0;TIM3->SR=0;sampling_started=ticks;sampling=1;TIM3->CR1=TIM_CR1_CEN;}}
static void pwm_set(unsigned on){
    if(!on){TIM2->CR1=TIM2->CCER=0;for(unsigned i=0;i<4;i++)pin(GPIOA,i,4);pwm_on=0;return;}
    TIM2->PSC=SystemCoreClock/1000000-1;TIM2->ARR=19999;
    TIM2->CCMR1=TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2PE;
    TIM2->CCMR2=TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3PE|TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4PE;
    volatile uint32_t *cc=&TIM2->CCR1;for(unsigned i=0;i<4;i++){cc[i]=pulses[i];pin(GPIOA,i,11);}
    TIM2->EGR=TIM_EGR_UG;TIM2->SR=0;TIM2->CCER=TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;TIM2->CR1=TIM_CR1_ARPE|TIM_CR1_CEN;pwm_on=1;last_manual=ticks;
}
static void process_link(void){
    while(rx_t!=rx_h){uint8_t b=link_rx[rx_t];rx_t=(rx_t+1)&(LINK_RX_SIZE-1);dart_packet p;if(dart_parse(&parser,b,&p)!=1)continue;
        if(p.type==DART_REPLY){char text[DART_PAYLOAD_MAX+1];memcpy(text,p.data,p.length);text[p.length]=0;report("H7: %s\r\n",text);}
        else if(p.type==DART_COMMAND){char text[DART_PAYLOAD_MAX+1];memcpy(text,p.data,p.length);text[p.length]=0;
            if(!strcmp(text,"IMU START")||!strcmp(text,"IMU WATCH")){
                if(accel_ok&&gyro_ok&&SystemCoreClock==72000000&&!pwm_on){
                    watching=!strcmp(text,"IMU WATCH");sampling_set(1);
                    link_text(DART_REPLY,watching?"IMU WATCHING":"IMU STARTED");
                }else link_text(DART_REPLY,"IMU ERROR ID/CONFIG/CLOCK/PWM");
            }
            else if(!strcmp(text,"IMU STOP")){sampling_set(0);watching=0;link_text(DART_REPLY,"IMU STOPPED");}
            else if(!strncmp(text,"LED ",4)){led_state=(uint32_t)strtoul(text+4,NULL,10);led_received=ticks;}
        }else if(p.type==DART_SYNC&&p.length==4){uint8_t reply[8];memcpy(reply,p.data,4);dart_put32(reply+4,micros());packet(DART_SYNC,reply,8);}
    }
}
static void status(void){report("DART F103 v=2.0 clk=%lu aid=%02X gid=%02X acc=%u gyro=%u active=%u samples=%lu late=%lu txdrop=%lu spierr=%lu uarterr=%lu crc=%lu pwm=%u\r\n",(unsigned long)SystemCoreClock,accel_id,gyro_id,accel_ok,gyro_ok,sampling,(unsigned long)sequence,(unsigned long)missed_ticks,(unsigned long)tx_drops,(unsigned long)spi_errors,(unsigned long)uart_errors,(unsigned long)parser.errors,pwm_on);}

static void update_led(void){
    /* 共阳 RGB 低电平点亮。PA5 为绿灯；其它支路颜色以实物为准。
       状态：1=准备慢闪，2=就绪常亮，3=记录快闪，4=整理文件双闪，
       5=完成慢闪，6=故障三闪，7=USB读卡；H750失联则显示故障。 */
    unsigned state=led_state;
    if((uint32_t)(ticks-led_received)>3500)state=6;
    uint32_t phase=ticks%2000u;unsigned pin_number=5,on=0;
    if(state==1)on=phase<250;
    else if(state==2)on=1;
    else if(state==3)on=(ticks%200u)<100;
    else if(state==4)on=phase<100||(phase>=200&&phase<300);
    else if(state==5)on=phase<1000;
    else if(state==6){pin_number=4;on=phase<100||(phase>=200&&phase<300)||(phase>=400&&phase<500);}
    else if(state==7){pin_number=6;on=1;}
    GPIOA->BSRR=(1u<<4)|(1u<<5)|(1u<<6);
    if(on)GPIOA->BRR=1u<<pin_number;
}
static void command(char *s){
    unsigned ch,us;char tail;
    if(!strncmp(s,"PING ",5))report("PONG %s\r\n",s+5);
    else if(!strcmp(s,"STATUS"))status();
    else if(!strcmp(s,"LINK DIAG")){report("LINK tx=%lu rx=%lu crc=%lu sr=%08lX brr=%lu\r\n",(unsigned long)link_tx_count,(unsigned long)link_rx_count,(unsigned long)parser.errors,(unsigned long)USART1->SR,(unsigned long)USART1->BRR);char hex[65];unsigned count=link_rx_count<32?link_rx_count:32;for(unsigned i=0;i<count;i++)snprintf(hex+i*2,3,"%02X",last_rx[(link_rx_count-count+i)&31]);hex[count*2]=0;report("LINK LAST %s\r\n",hex);}
    else if(!strcmp(s,"PWM OFF")){pwm_set(0);pc_write("OK PWM OFF\r\n");}
    else if(!strcmp(s,"PWM ON")){if(sampling)pc_write("ERR RECORDING\r\n");else{pwm_set(1);pc_write("OK PWM ON timeout=3000ms\r\n");}}
    else if(!strcmp(s,"PWM KEEP")){last_manual=ticks;pc_write("OK KEEP\r\n");}
    else if(sscanf(s,"PWM %u %u %c",&ch,&us,&tail)==2&&ch>=1&&ch<=4&&us>=1000&&us<=2000){pulses[ch-1]=us;(&TIM2->CCR1)[ch-1]=us;last_manual=ticks;pc_write("OK PWM SET\r\n");}
    else if(!strncmp(s,"H7 ",3)&&strlen(s+3)<=DART_PAYLOAD_MAX){link_text(DART_COMMAND,s+3);pc_write("OK H7 TX\r\n");}
    else pc_write("COMMANDS PING STATUS H7 STATUS/H7 START <6HEX> 10/H7 STOP PWM ON/OFF/KEEP/<ch> <us>\r\n");
}
int main(void){
    clock_init();board_init();imu_init();pc_write("BOOT Dart AutoRecord F103 2.0; PWM OFF; link=921600; gyro=1000Hz accel=1600Hz\r\n");status();
    char line[96];unsigned n=0,discard=0;uint32_t heartbeat=0;
    for(;;){process_link();while(pc_t!=pc_h){char c=(char)pc_rx[pc_t];pc_t=(pc_t+1)&255;if(c=='\r')continue;if(c=='\n'){if(!discard){line[n]=0;command(line);}n=discard=0;}else if(n<sizeof(line)-1&&!discard)line[n++]=c;else discard=1;}
        if(pwm_on&&(uint32_t)(ticks-last_manual)>3000){pwm_set(0);pc_write("EVENT PWM TIMEOUT\r\n");}
        update_led();
        if(sampling&&!watching&&(uint32_t)(ticks-sampling_started)>12000){sampling_set(0);link_text(DART_REPLY,"IMU STOPPED");pc_write("EVENT IMU SESSION TIMEOUT\r\n");}
        if((uint32_t)(ticks-heartbeat)>1000){heartbeat=ticks;link_text(DART_REPLY,"F103 ALIVE");}
    }
}
