/* Linker wrappers: leave the upstream ST HAL/LL sources unchanged and capture
 * command results without adding UART delays inside card initialization.
 */
#include "stm32h7xx_hal.h"
#include "sd_trace.h"
#include <string.h>

sd_trace_entry_t sd_trace_entries[SD_TRACE_CAPACITY];
uint32_t sd_trace_total;

void sd_trace_reset(void)
{
    sd_trace_total = 0;
    memset(sd_trace_entries, 0, sizeof(sd_trace_entries));
}

static void record(SDMMC_TypeDef *sd, uint32_t result)
{
    sd_trace_entry_t *entry = &sd_trace_entries[sd_trace_total % SD_TRACE_CAPACITY];
    entry->cmd = sd->CMD & 0x3FU;
    entry->arg = sd->ARG;
    entry->error = result;
    entry->response_cmd = sd->RESPCMD & 0x3FU;
    entry->response = sd->RESP1;
    entry->status = sd->STA;
    ++sd_trace_total;
}

#define WRAP0(name) \
    extern uint32_t __real_##name(SDMMC_TypeDef *sd); \
    uint32_t __wrap_##name(SDMMC_TypeDef *sd) { \
        uint32_t result = __real_##name(sd); record(sd, result); return result; \
    }
#define WRAP1(name, type) \
    extern uint32_t __real_##name(SDMMC_TypeDef *sd, type value); \
    uint32_t __wrap_##name(SDMMC_TypeDef *sd, type value) { \
        uint32_t result = __real_##name(sd, value); record(sd, result); return result; \
    }

WRAP0(SDMMC_CmdGoIdleState)
WRAP0(SDMMC_CmdOperCond)
WRAP1(SDMMC_CmdAppCommand, uint32_t)
WRAP1(SDMMC_CmdAppOperCommand, uint32_t)
WRAP0(SDMMC_CmdSendCID)
WRAP1(SDMMC_CmdSetRelAdd, uint16_t *)
WRAP1(SDMMC_CmdSendCSD, uint32_t)
WRAP1(SDMMC_CmdSelDesel, uint32_t)
WRAP1(SDMMC_CmdBlockLength, uint32_t)
WRAP0(SDMMC_CmdStatusRegister)
WRAP0(SDMMC_CmdSendSCR)
WRAP1(SDMMC_CmdBusWidth, uint32_t)
WRAP1(SDMMC_CmdSendStatus, uint32_t)
WRAP1(SDMMC_CmdReadSingleBlock, uint32_t)
