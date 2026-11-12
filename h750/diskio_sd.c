/* Polling SDMMC bridge for FatFs. No DMA/cache dependency, format, or trim.
 * Only an explicit file-test session may enable disk writes.
 */
#include "stm32h7xx_hal.h"
#include "diskio.h"
#include "fs_test.h"
#include "usb_tf.h"
#include "camera_perf.h"
#include <string.h>

extern SD_HandleTypeDef hsd1;
volatile uint32_t fs_disk_write_enabled;
volatile uint32_t fs_disk_last_error;
volatile uint32_t fs_disk_reads, fs_disk_writes;
volatile uint32_t fs_disk_read_sectors, fs_disk_write_sectors;
static uint32_t disk_ready, disk_sectors;
static volatile unsigned dma_done,dma_failed;
void SDMMC1_IRQHandler(void) {HAL_SD_IRQHandler(&hsd1);}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *h) {(void)h;dma_done=1;}
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *h) {(void)h;dma_done=1;}
void HAL_SD_ErrorCallback(SD_HandleTypeDef *h) {(void)h;dma_failed=1;}
static DRESULT wait_transfer(void);
static DRESULT transfer_dma(BYTE *data,LBA_t sector,UINT count,unsigned write) {
    BYTE *bounce=camera_workspace+480U*1024U;
    while(count) {
        UINT n=count>64?64:count;
        if(write&&bounce!=data)memcpy(bounce,data,n*512U);
        dma_done=dma_failed=0;__DMB();
        HAL_StatusTypeDef r=write?HAL_SD_WriteBlocks_DMA(&hsd1,bounce,sector,n):HAL_SD_ReadBlocks_DMA(&hsd1,bounce,sector,n);
        uint32_t t=HAL_GetTick();
        if(r==HAL_OK)while(!dma_done&&!dma_failed&&HAL_GetTick()-t<5000U){}
        if(r!=HAL_OK||dma_failed||!dma_done) {fs_disk_last_error=HAL_SD_GetError(&hsd1)|HAL_SD_ERROR_TIMEOUT;return RES_ERROR;}
        if(wait_transfer()!=RES_OK)return RES_ERROR;
        if(!write&&data!=bounce)memcpy(data,bounce,n*512U);
        sector+=n;count-=n;data+=n*512U;
    }
    return RES_OK;
}


void fs_disk_set_ready(uint32_t ready, uint32_t sectors)
{
    disk_ready = ready;
    disk_sectors = sectors;
    fs_disk_write_enabled = 0;
}

static DRESULT wait_transfer(void)
{
    uint32_t started = HAL_GetTick();
    do {
        HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState(&hsd1);
        if (state == HAL_SD_CARD_TRANSFER) return RES_OK;
        if (state == HAL_SD_CARD_ERROR) {
            fs_disk_last_error = HAL_SD_GetError(&hsd1);
            return RES_ERROR;
        }
    } while (HAL_GetTick() - started < 5000U);
    fs_disk_last_error = HAL_SD_GetError(&hsd1) | HAL_SD_ERROR_TIMEOUT;
    return RES_ERROR;
}

DSTATUS disk_status(BYTE drive)
{
    return drive == 0 && disk_ready ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE drive)
{
    /* The explicit SD bring-up already initialized the card and selected 4bit.
     * Never reset SDMMC underneath a mounted filesystem.
     */
    return disk_status(drive);
}

static int valid_range(LBA_t sector, UINT count)
{
    return count > 0 && sector < disk_sectors && count <= disk_sectors - sector;
}

DRESULT disk_read(BYTE drive, BYTE *buffer, LBA_t sector, UINT count)
{
    if (disk_status(drive)) return RES_NOTRDY;
    if (!buffer || !valid_range(sector, count)) return RES_PARERR;
    if (wait_transfer() != RES_OK) return RES_ERROR;
    if (transfer_dma(buffer,sector,count,0) != RES_OK) {
        fs_disk_last_error = HAL_SD_GetError(&hsd1);
        return RES_ERROR;
    }
    ++fs_disk_reads; fs_disk_read_sectors += count;
    return wait_transfer();
}

DRESULT disk_write(BYTE drive, const BYTE *buffer, LBA_t sector, UINT count)
{
    if (disk_status(drive)) return RES_NOTRDY;
    if (!fs_disk_write_enabled || usb_tf_host_active()) return RES_WRPRT;
    /* Sector 0 contains the partition table. File operations must not write it. */
    if (!buffer || !sector || !valid_range(sector, count)) return RES_PARERR;
    if (wait_transfer() != RES_OK) return RES_ERROR;
    if (transfer_dma((BYTE *)buffer,sector,count,1) != RES_OK) {
        fs_disk_last_error = HAL_SD_GetError(&hsd1);
        return RES_ERROR;
    }
    ++fs_disk_writes; fs_disk_write_sectors += count;
    return wait_transfer();
}

DRESULT disk_ioctl(BYTE drive, BYTE command, void *buffer)
{
    if (disk_status(drive)) return RES_NOTRDY;
    switch (command) {
    case CTRL_SYNC: return wait_transfer();
    case GET_SECTOR_COUNT:
        if (!buffer) return RES_PARERR;
        *(LBA_t *)buffer = disk_sectors; return RES_OK;
    case GET_SECTOR_SIZE:
        if (!buffer) return RES_PARERR;
        *(WORD *)buffer = 512; return RES_OK;
    case GET_BLOCK_SIZE:
        if (!buffer) return RES_PARERR;
        *(DWORD *)buffer = 1; return RES_OK;
    default: return RES_PARERR; /* No format, trim, erase or arbitrary control. */
    }
}
