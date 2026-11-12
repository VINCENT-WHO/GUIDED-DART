/* H750 PA11/PA12 native USB FS, read-only TF mass storage.
 * ST USB MSC middleware is retained unmodified in vendor.
 * No format, trim or write callback implementation. Application commands that
 * could touch SD/camera are rejected while a USB host owns the card.
 */
#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_msc.h"
#include "usbd_ctlreq.h"
#include "diskio.h"
#include "fs_test.h"
#include "usb_tf.h"

static USBD_HandleTypeDef usb;
static volatile unsigned application_busy;
static uint8_t device_desc[18]={18,USB_DESC_TYPE_DEVICE,0,2,0,0,0,64,0x83,0x04,0x20,0x57,0x08,0,1,2,3,1};
static uint8_t lang_desc[4]={4,USB_DESC_TYPE_STRING,9,4};
static uint8_t strings[64];
static uint8_t *dev(USBD_SpeedTypeDef s,uint16_t *n){(void)s;*n=sizeof(device_desc);return device_desc;}
static uint8_t *lang(USBD_SpeedTypeDef s,uint16_t *n){(void)s;*n=sizeof(lang_desc);return lang_desc;}
static uint8_t *str(const char *s,uint16_t *n){USBD_GetString((uint8_t *)s,strings,n);return strings;}
static uint8_t *mfr(USBD_SpeedTypeDef s,uint16_t *n){(void)s;return str("Dart Bench",n);}
static uint8_t *product(USBD_SpeedTypeDef s,uint16_t *n){(void)s;return str("Dart TF ReadOnly",n);}
static uint8_t *serial(USBD_SpeedTypeDef s,uint16_t *n){(void)s;return str("DARTTF200364500000",n);}
static uint8_t *config(USBD_SpeedTypeDef s,uint16_t *n){(void)s;return str("TF ReadOnly",n);}
static uint8_t *interface(USBD_SpeedTypeDef s,uint16_t *n){(void)s;return str("TF ReadOnly MSC",n);}
static USBD_DescriptorsTypeDef desc={dev,lang,mfr,product,serial,config,interface};

static int8_t inquiry[36]={0,0x80,2,2,31,0,0,0,
    'D','A','R','T',' ',' ',' ',' ',
    'T','F',' ','R','E','A','D',' ','O','N','L','Y',' ',' ',' ',' ',
    '0','0','0','7'};
static int8_t init(uint8_t lun){return lun==0&&!application_busy&&disk_status(0)==0?0:-1;}
static int8_t capacity(uint8_t lun,uint32_t *blocks,uint16_t *bytes){
    if(lun||application_busy||disk_ioctl(0,GET_SECTOR_COUNT,blocks)!=RES_OK)return -1;
    *bytes=512;return 0;
}
static int8_t ready(uint8_t lun){return lun==0&&!application_busy&&disk_status(0)==0?0:-1;}
static int8_t protected(uint8_t lun){(void)lun;return 1;}
static int8_t read_blocks(uint8_t lun,uint8_t *b,uint32_t a,uint16_t n){return lun==0&&!application_busy&&disk_read(0,b,a,n)==RES_OK?0:-1;}
static int8_t write_blocks(uint8_t lun,uint8_t *b,uint32_t a,uint16_t n){(void)lun;(void)b;(void)a;(void)n;return -1;}
static int8_t max_lun(void){return 0;}
static USBD_StorageTypeDef storage={init,capacity,ready,protected,read_blocks,write_blocks,max_lun,inquiry};

int usb_tf_connected(void){return usb.dev_state==USBD_STATE_CONFIGURED;}
int usb_tf_host_active(void){return usb_tf_connected()&&!application_busy;}
void usb_tf_set_busy(unsigned busy){application_busy=busy;__DMB();}
void usb_tf_start(void){
    RCC_PeriphCLKInitTypeDef c={0};c.PeriphClockSelection=RCC_PERIPHCLK_USB;c.UsbClockSelection=RCC_USBCLKSOURCE_PLL;
    if(HAL_RCCEx_PeriphCLKConfig(&c)!=HAL_OK){bench_puts("H750 USB_TF CLOCK_ERROR\r\n");return;}
    HAL_PWREx_EnableUSBVoltageDetector();fs_disk_write_enabled=0;
    if(USBD_Init(&usb,&desc,0)!=USBD_OK||USBD_RegisterClass(&usb,&USBD_MSC)!=USBD_OK||
       USBD_MSC_RegisterStorage(&usb,&storage)!=USBD_OK||USBD_Start(&usb)!=USBD_OK){
        bench_puts("H750 USB_TF INIT_ERROR\r\n");return;
    }
    bench_puts("H750 USB_TF READY native_FS MSC_READ_ONLY\r\n");
}
