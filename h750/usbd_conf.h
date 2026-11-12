#pragma once
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#define USBD_MAX_NUM_INTERFACES 1U
#define USBD_MAX_NUM_CONFIGURATION 1U
#define USBD_MAX_STR_DESC_SIZ 64U
#define USBD_SELF_POWERED 0U
#define USBD_MAX_POWER 250U
#define USBD_DEBUG_LEVEL 0U
#define MSC_MEDIA_PACKET 4096U
#define MSC_BOT_MAX_LUN 0U
#define USBD_malloc USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);
