#ifndef DART_TIMEBASE_H
#define DART_TIMEBASE_H
#include <stdint.h>
/* Caller snapshots ms/VAL atomically and accounts for any pending SysTick.
   Derive the fraction from SysTick itself, never from an unrelated DWT phase. */
static inline uint32_t dart_systick_micros(uint32_t ms,uint32_t reload,uint32_t value){
    return ms*1000u+(reload-value)*1000u/(reload+1u);
}
/* last may advance in an IRQ immediately after now was sampled. Treat that
   small negative interval as fresh, not as an unsigned four-billion-us gap.
   All configured timeouts are less than2^31us. */
static inline int dart_is_stale(uint32_t now,uint32_t last,uint32_t timeout){
    return (int32_t)(now-last)>(int32_t)timeout;
}
#endif
