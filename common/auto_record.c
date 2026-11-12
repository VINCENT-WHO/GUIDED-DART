#include "auto_record.h"
#include <stdio.h>

const char dart_imu_csv_header[] =
    "\xEF\xBB\xBF"
    "序号,相对触发时间_s,角速度X_deg_s,角速度Y_deg_s,角速度Z_deg_s,"
    "加速度X_g,加速度Y_g,加速度Z_g,加速度合量_g,有效标志,丢失采样累计,IMU时间_us\r\n";

static uint64_t norm_squared(const int16_t v[3])
{
    uint64_t sum = 0;
    for (unsigned i = 0; i < 3; ++i) {
        int32_t x = v[i];
        sum += (uint64_t)((int64_t)x * x);
    }
    return sum;
}

int dart_trigger_update(dart_trigger *state, const dart_imu *sample)
{
    int valid = (sample->flags & IMU_ACCEL_VALID) &&
                !(sample->flags & (IMU_SPI_ERROR | IMU_SCHEDULE_LATE));
    /* 用整数平方比较，避免引入浮点开方，也避免把 2g 误当成扣重力后的 2g。 */
    uint64_t measured = norm_squared(sample->accel) * 24000ull * 24000ull;
    uint64_t threshold = (uint64_t)DART_AUTO_THRESHOLD_MG *
                         DART_AUTO_THRESHOLD_MG * 32768ull * 32768ull;
    if (!valid || measured <= threshold) {
        state->above = 0;
        state->have_last = 0;
        return 0;
    }
    if (!state->have_last ||
        (uint32_t)(sample->time_us - state->last_sample_us) > DART_AUTO_MAX_SAMPLE_GAP_US)
        state->above = 0;
    state->last_sample_us = sample->time_us;
    state->have_last = 1;
    if (!state->above) {
        state->above = 1;
        state->first_above_us = sample->time_us;
    }
    return (uint32_t)(sample->time_us - state->first_above_us) >= DART_AUTO_HOLD_US;
}

uint32_t dart_accel_norm_mg(const int16_t accel[3])
{
    /* 二分求整数平方根。中间值使用 64 位，覆盖三个轴同时满量程。 */
    uint64_t squared = norm_squared(accel);
    uint32_t low = 0, high = 56757;
    while (low < high) {
        uint32_t middle = low + (high - low + 1) / 2;
        if ((uint64_t)middle * middle <= squared) low = middle;
        else high = middle - 1;
    }
    return (uint32_t)(((uint64_t)low * 24000 + 16384) / 32768);
}

static void fixed(char *out, size_t capacity, int64_t value, unsigned scale, unsigned decimals)
{
    /* 整数定点格式化不依赖 newlib 的浮点 printf；负的零点几也保留符号。 */
    uint64_t magnitude = value < 0 ? (uint64_t)-value : (uint64_t)value;
    snprintf(out, capacity, "%s%lu.%0*lu", value < 0 ? "-" : "",
             (unsigned long)(magnitude / scale), (int)decimals,
             (unsigned long)(magnitude % scale));
}

int dart_imu_csv_line(char *out, size_t capacity, const dart_imu *s, uint32_t trigger_f1_us)
{
    char time[24], gyro[3][24], accel[3][24], norm[24];
    fixed(time, sizeof(time), (int32_t)(s->time_us - trigger_f1_us), 1000000, 6);
    for (unsigned i = 0; i < 3; ++i) {
        fixed(gyro[i], sizeof(gyro[i]), (int64_t)s->gyro[i] * 2000000 / 32768, 1000, 3);
        fixed(accel[i], sizeof(accel[i]), (int64_t)s->accel[i] * 24000000 / 32768, 1000000, 6);
    }
    fixed(norm, sizeof(norm), dart_accel_norm_mg(s->accel), 1000, 3);
    return snprintf(out, capacity, "%lu,%s,%s,%s,%s,%s,%s,%s,%s,%u,%lu,%lu\r\n",
                    (unsigned long)s->sequence, time, gyro[0], gyro[1], gyro[2],
                    accel[0], accel[1], accel[2], norm, s->flags,
                    (unsigned long)s->missed_ticks, (unsigned long)s->time_us);
}
