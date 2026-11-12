#ifndef DART_SD_TRACE_H
#define DART_SD_TRACE_H
#include <stdint.h>

/* Last 32 command results; snapshots are taken after the stock ST command
 * helper returns. STA flags may already have been cleared by that helper.
 */
typedef struct {
    uint32_t cmd, arg, error, response_cmd, response, status;
} sd_trace_entry_t;
enum { SD_TRACE_CAPACITY = 32 };
extern sd_trace_entry_t sd_trace_entries[SD_TRACE_CAPACITY];
extern uint32_t sd_trace_total;
void sd_trace_reset(void);
#endif
