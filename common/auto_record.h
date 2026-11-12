#ifndef DART_AUTO_RECORD_H
#define DART_AUTO_RECORD_H

#include "protocol.h"

/* BMI088 仍使用已经验证的 ±24g 量程；阈值以毫 g 表示，便于修改。 */
#define DART_AUTO_THRESHOLD_MG 2000u
#define DART_AUTO_HOLD_US 20000u
#define DART_AUTO_DURATION_US 10000000u
#define DART_AUTO_PRE_SAMPLES 200u
#define DART_AUTO_MAX_SAMPLE_GAP_US 2500u

typedef struct {
    uint32_t first_above_us;
    uint32_t last_sample_us;
    unsigned above;
    unsigned have_last;
} dart_trigger;

/* 只判断加速度合量，不要求安装方向固定。无效样本/采样间断会清除计时。 */
int dart_trigger_update(dart_trigger *state, const dart_imu *sample);
uint32_t dart_accel_norm_mg(const int16_t accel[3]);

/* 输出 Excel 可读的十进制数据；相对时间可以为负，表示触发前的缓存。 */
int dart_imu_csv_line(char *out, size_t capacity, const dart_imu *sample,
                      uint32_t trigger_f1_us);
extern const char dart_imu_csv_header[];

#endif
