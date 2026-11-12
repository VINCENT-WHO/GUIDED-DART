/* H750 clock/SD wiring adapted from the supplied board-verified sources. */
#include "stm32h7xx_hal.h"
#include "sd_trace.h"
#include "fs_test.h"
#include <string.h>
SD_HandleTypeDef hsd1;
typedef struct {
 uint32_t magic,version,hse_ok,pll_ok,clock_hz,millis,pings;
 uint32_t sd_init_ok,sd_1bit_ok,sd_4bit_ok,sd_crc_match,sd_error;
 uint32_t blocks,block_size,crc_1bit,crc_4bit,signature_55aa;
 uint32_t cid[4],sd_stage,sd_kernel_hz,sd_clock_div,uart_rx_errors;
} report_t;
volatile report_t g_report;
static uint8_t block[512] __attribute__((aligned(32)));
static uint32_t sd_clock_div=1;
static HAL_StatusTypeDef clock_init(void)
{
    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) return HAL_ERROR;
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    uint32_t start = HAL_GetTick();
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
        if (HAL_GetTick() - start > 100) return HAL_TIMEOUT;
    }
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 3; osc.PLL.PLLN = 192;
    osc.PLL.PLLP = 2; osc.PLL.PLLQ = 16; osc.PLL.PLLR = 4;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN = 0;
    HAL_StatusTypeDef result = HAL_RCC_OscConfig(&osc);
    g_report.hse_ok = (RCC->CR & RCC_CR_HSERDY) != 0;
    if (result != HAL_OK) return result;
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1; clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2; clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2; clk.APB4CLKDivider = RCC_APB4_DIV2;
    result = HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
    if (result != HAL_OK) return result;
    g_report.pll_ok = 1;
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
    periph.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
    return HAL_RCCEx_PeriphCLKConfig(&periph);
}

void HAL_SD_MspInit(SD_HandleTypeDef *sd)
{
    if (sd->Instance != SDMMC1) return;
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    HAL_NVIC_SetPriority(SDMMC1_IRQn,3,0);HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SDMMC1_FORCE_RESET(); __HAL_RCC_SDMMC1_RELEASE_RESET();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP; gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_12; gpio.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_2; gpio.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOD, &gpio);
}

static uint32_t crc32(const uint8_t *data, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0);
    }
    return ~crc;
}

static void sd_test(void)
{
    fs_disk_set_ready(0, 0);
    sd_trace_reset();
    g_report.sd_stage = 0;
    g_report.sd_kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC);
    g_report.sd_clock_div = sd_clock_div;
    g_report.sd_init_ok = g_report.sd_1bit_ok = g_report.sd_4bit_ok = 0;
    g_report.sd_crc_match = g_report.sd_error = g_report.blocks = 0;
    g_report.block_size = g_report.crc_1bit = g_report.crc_4bit = g_report.signature_55aa = 0;
    if (!g_report.pll_ok) { g_report.sd_error = 0xFFFFFFFFU; return; }
    memset(&hsd1, 0, sizeof(hsd1));
    hsd1.Instance = SDMMC1;
    hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd1.Init.ClockDiv = sd_clock_div; /* Default 6MHz; SD SLOW selects 400kHz. */
    g_report.sd_stage = 1;
    if (HAL_SD_Init(&hsd1) != HAL_OK) { g_report.sd_error = HAL_SD_GetError(&hsd1); return; }
    g_report.sd_init_ok = 1;
    g_report.sd_stage = 2;
    HAL_SD_CardInfoTypeDef info;
    if (HAL_SD_GetCardInfo(&hsd1, &info) != HAL_OK) { g_report.sd_error = HAL_SD_GetError(&hsd1); return; }
    g_report.blocks = info.LogBlockNbr; g_report.block_size = info.LogBlockSize;
    for (unsigned i = 0; i < 4; ++i) g_report.cid[i] = hsd1.CID[i];
    g_report.sd_stage = 3;
    if (HAL_SD_ReadBlocks(&hsd1, block, 0, 1, 1000) != HAL_OK) { g_report.sd_error = HAL_SD_GetError(&hsd1); return; }
    g_report.sd_1bit_ok = 1;
    g_report.crc_1bit = crc32(block, sizeof(block));
    g_report.signature_55aa = block[510] == 0x55 && block[511] == 0xAA;
    g_report.sd_stage = 4;
    if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK) { g_report.sd_error = HAL_SD_GetError(&hsd1); return; }
    g_report.sd_stage = 5;
    if (HAL_SD_ReadBlocks(&hsd1, block, 0, 1, 1000) != HAL_OK) { g_report.sd_error = HAL_SD_GetError(&hsd1); return; }
    g_report.sd_4bit_ok = 1;
    g_report.crc_4bit = crc32(block, sizeof(block));
    g_report.sd_crc_match = g_report.crc_1bit == g_report.crc_4bit;
    g_report.sd_stage = 6;
    if (g_report.sd_crc_match) fs_disk_set_ready(1, g_report.blocks);
}


int board_clock_start(void){return clock_init()==HAL_OK?0:-1;}
int board_storage_start(void){sd_test();return g_report.sd_crc_match?0:-2;}
unsigned board_sd_ready(void){return g_report.sd_crc_match;}
uint32_t board_sd_blocks(void){return g_report.blocks;}
