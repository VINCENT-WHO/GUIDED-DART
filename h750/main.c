/* H750 自动记录器：上电准备 -> 加速度触发 -> 记录10秒 -> 生成可读文件。
 * 相机、TF 均在触发前完成初始化；只采集数据，不进行姿态控制。
 */
#include "stm32h7xx_hal.h"
#include "protocol.h"
#include "timebase.h"
#include "auto_record.h"
#include "avi.h"
#include "camera_perf.h"
#include "camera_test.h"
#include "usb_tf.h"
#include "fs_test.h"
#include "ff.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

extern int board_clock_start(void);
extern int board_storage_start(void);
extern unsigned board_sd_ready(void);

typedef enum { WAIT_BOOT, PREPARING, ARMED, RECORDING, SAVING, DONE, ERROR_STATE,
               USB_READER, DISARMED } record_state;
static const char *const state_names[] = {
    "BOOT_WAIT", "PREPARING", "READY", "RECORDING", "SAVING", "DONE", "ERROR", "USB_READ", "DISARMED"
};
static record_state state = WAIT_BOOT;
volatile uint32_t g_boot_stage, g_fault_cfsr, g_fault_hfsr;
static uint32_t link_seq, log_seq, uart_errors, reply_overflows, imu_overflows;
static dart_parser parser;
typedef struct { uint32_t rx_us; dart_imu imu; } imu_entry;
static imu_entry imu_queue[1024] __attribute__((section(".camera_d2"), aligned(32)));
static imu_entry pre_trigger[DART_AUTO_PRE_SAMPLES] __attribute__((section(".camera_d2"), aligned(32)));
static volatile unsigned imu_head, imu_tail, accept_imu;
static dart_packet replies[8];
static volatile unsigned reply_head, reply_tail, remote_started, remote_stopped;
static FATFS fs;
static FIL video, logfile, csvfile;
static unsigned video_open, log_open, csv_open, mounted, failed, camera_mode, camera_running;
static unsigned automatic, auto_fired, auto_disabled, first_imu = 1;
static unsigned pre_head, pre_count;
static dart_trigger trigger;
static uint32_t start_us, end_us, trigger_f1_us, prepare_us, session_token;
static uint32_t saved_frames, saved_imu, first_frame, last_frame, biggest, movi_bytes;
static uint32_t last_seq, imu_gaps, max_write_ms, session_imu_overflows, session_crc_errors, session_uart_errors;
static uint32_t start_camera_frames, start_camera_drops, start_camera_invalid;
static uint32_t csv_rows;
static char directory[16], video_path[40], log_path[40], csv_path[40], info_path[40];
#define MAX_FRAMES 600u
static uint32_t offsets[MAX_FRAMES], sizes[MAX_FRAMES];
static uint8_t log_buffer[8192], convert_buffer[512];
static unsigned log_used;
static uint32_t sync_sent, sync_ready_time, sync_f1_time;
static volatile unsigned sync_ready;
static unsigned sync_pending;
static volatile uint32_t last_imu_rx;
static uint32_t last_good_frame, last_led_sent;

void _init(void) {} void _fini(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }
uint32_t board_micros(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint32_t ms = HAL_GetTick(), cycles = SysTick->VAL, load = SysTick->LOAD;
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) { ms++; cycles = SysTick->VAL; }
    __set_PRIMASK(irq);
    return dart_systick_micros(ms, load, cycles);
}

static int wire_send(uint8_t type, const void *data, unsigned n)
{
    uint8_t wire[DART_WIRE_MAX];
    size_t length = dart_encode(type, link_seq++, data, n, wire);
    if (!length) return -1;
    for (size_t i = 0; i < length; i++) {
        uint32_t begin = board_micros();
        while (!(USART3->ISR & USART_ISR_TXE_TXFNF))
            if ((uint32_t)(board_micros() - begin) > 2000) { uart_errors++; return -1; }
        USART3->TDR = wire[i];
    }
    return 0;
}

static void reply(const char *fmt, ...)
{
    char text[DART_PAYLOAD_MAX + 1]; va_list args;
    va_start(args, fmt); vsnprintf(text, sizeof(text), fmt, args); va_end(args);
    wire_send(DART_REPLY, text, (unsigned)strlen(text));
}
void bench_puts(const char *s)
{
    char line[DART_PAYLOAD_MAX + 1]; unsigned n = 0;
    while (*s) {
        if (*s == '\r') { s++; continue; }
        if (*s == '\n') { if (n) { wire_send(DART_REPLY, line, n); n = 0; } s++; continue; }
        line[n++] = *s++;
        if (n == DART_PAYLOAD_MAX) { wire_send(DART_REPLY, line, n); n = 0; }
    }
    if (n) wire_send(DART_REPLY, line, n);
}
void HardFault_Handler(void)
{
    g_fault_cfsr = SCB->CFSR; g_fault_hfsr = SCB->HFSR;
    reply("H750 HARDFAULT stage=%lu cfsr=%08lX hfsr=%08lX",
          (unsigned long)g_boot_stage, (unsigned long)g_fault_cfsr, (unsigned long)g_fault_hfsr);
    for (;;) {}
}
static void uart_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_USART3_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP; gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);
    USART3->CR1 = 0; USART3->CR2 = 0; USART3->CR3 = 0; USART3->PRESC = 0;
    USART3->BRR = (HAL_RCC_GetPCLK1Freq() + 460800) / 921600; USART3->ICR = 0xFFFFFFFF;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    /* 串口优先于相机帧结束中断，避免等待 DMA 排空时丢字节。 */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0); HAL_NVIC_EnableIRQ(USART3_IRQn);
}
void USART3_IRQHandler(void)
{
    uint32_t flags = USART3->ISR;
    if (flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        uart_errors++; USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
    if (!(flags & USART_ISR_RXNE_RXFNE)) return;
    dart_packet p;
    if (dart_parse(&parser, (uint8_t)USART3->RDR, &p) != 1) return;
    if (p.type == DART_IMU && p.length == sizeof(dart_imu) && accept_imu) {
        unsigned next = (imu_head + 1) & 1023;
        if (next == imu_tail) { imu_overflows++; return; }
        last_imu_rx = board_micros(); imu_queue[imu_head].rx_us = last_imu_rx;
        memcpy(&imu_queue[imu_head].imu, p.data, sizeof(dart_imu)); __DMB(); imu_head = next;
    } else if (p.type == DART_SYNC && p.length == 8) {
        if (dart_get32(p.data) == sync_sent) {
            sync_ready_time = board_micros(); sync_f1_time = dart_get32(p.data + 4); sync_ready = 1;
        }
    } else {
        if (p.type == DART_REPLY &&
            ((p.length == 11 && !memcmp(p.data, "IMU STARTED", 11)) ||
             (p.length == 12 && !memcmp(p.data, "IMU WATCHING", 12)))) remote_started = 1;
        if (p.type == DART_REPLY && p.length == 11 && !memcmp(p.data, "IMU STOPPED", 11)) remote_stopped = 1;
        unsigned next = (reply_head + 1) & 7;
        if (next == reply_tail) { reply_overflows++; return; }
        replies[reply_head] = p; __DMB(); reply_head = next;
    }
}

static void send_led(void)
{
    unsigned led = state == ARMED ? 2 : state == RECORDING ? 3 : state == SAVING ? 4 :
                   state == DONE ? 5 : state == ERROR_STATE ? 6 : state == USB_READER ? 7 : 1;
    char text[16]; snprintf(text, sizeof(text), "LED %u", led);
    wire_send(DART_COMMAND, text, (unsigned)strlen(text)); last_led_sent = HAL_GetTick();
}
static FRESULT write_exact(FIL *file, const void *data, UINT count)
{
    UINT written = 0; uint32_t begin = HAL_GetTick();
    FRESULT result = f_write(file, data, count, &written);
    uint32_t elapsed = HAL_GetTick() - begin;
    if (elapsed > max_write_ms) max_write_ms = elapsed;
    return result == FR_OK && written != count ? FR_DISK_ERR : result;
}
static int flush_log(void)
{
    if (!log_used) return 0;
    if (write_exact(&logfile, log_buffer, log_used) != FR_OK) { failed = 1; return -1; }
    log_used = 0; return 0;
}
static int log_packet(uint8_t kind, uint32_t time, const void *data, unsigned n)
{
    uint8_t payload[DART_PAYLOAD_MAX], wire[DART_WIRE_MAX];
    if (n + 4 > sizeof(payload)) return -1;
    dart_put32(payload, time); if (n) memcpy(payload + 4, data, n);
    unsigned count = (unsigned)dart_encode(kind, log_seq++, payload, n + 4, wire);
    if (log_used + count > sizeof(log_buffer) && flush_log()) return -1;
    memcpy(log_buffer + log_used, wire, count); log_used += count; return 0;
}
static void log_event(const char *text) { if (log_open) log_packet(19, board_micros(), text, (unsigned)strlen(text)); }
static void save_imu(const imu_entry *entry)
{
    if (first_imu) first_imu = 0;
    else if (entry->imu.sequence != last_seq + 1) imu_gaps += entry->imu.sequence - last_seq - 1;
    last_seq = entry->imu.sequence;
    if (!log_packet(16, entry->rx_us, &entry->imu, sizeof(entry->imu))) saved_imu++;
}
static void sync_request(void)
{
    uint8_t time[4]; sync_sent = board_micros(); dart_put32(time, sync_sent); sync_ready = 0;
    wire_send(DART_SYNC, time, 4); sync_pending = 1;
}

static void begin_recording(const imu_entry *entry)
{
    /* 以确认超过阈值的那个样本为零时刻，不把准备文件的时间计入10秒。 */
    start_us = entry->rx_us; end_us = start_us + DART_AUTO_DURATION_US;
    trigger_f1_us = entry->imu.time_us; state = RECORDING;
    if (automatic) auto_fired = 1;
    start_camera_frames = camera_frames; start_camera_drops = camera_drops; start_camera_invalid = camera_invalid;
    char text[76];
    snprintf(text, sizeof(text), "TRIGGER f1_us=%lu h7_us=%lu auto=%u norm_mg=%lu",
             (unsigned long)trigger_f1_us, (unsigned long)start_us, automatic,
             (unsigned long)dart_accel_norm_mg(entry->imu.accel));
    log_event(text);
    unsigned oldest = (pre_head + DART_AUTO_PRE_SAMPLES - pre_count) % DART_AUTO_PRE_SAMPLES;
    for (unsigned i = 0; i < pre_count && !failed; i++) save_imu(&pre_trigger[(oldest + i) % DART_AUTO_PRE_SAMPLES]);
    pre_count = 0;
    reply("SESSION START id=%06lX duration=10 auto=%u", (unsigned long)session_token, automatic);
    send_led();
}
static void drain_imu(void)
{
    while (imu_tail != imu_head) {
        imu_entry entry = imu_queue[imu_tail]; __DMB(); imu_tail = (imu_tail + 1) & 1023;
        if (state == RECORDING) {
            /* STOP 的往返期间还可能收到数据，只保存截止时刻以前的样本。 */
            if ((int32_t)(entry.rx_us - end_us) < 0) save_imu(&entry);
        } else if (state == PREPARING || state == ARMED) {
            pre_trigger[pre_head] = entry; pre_head = (pre_head + 1) % DART_AUTO_PRE_SAMPLES;
            if (pre_count < DART_AUTO_PRE_SAMPLES) pre_count++;
            if (state == ARMED && dart_trigger_update(&trigger, &entry.imu)) begin_recording(&entry);
        }
    }
}
static void save_frame(const uint8_t *bytes, uint32_t n, uint32_t time)
{
    if ((int32_t)(time - start_us) < 0 || (int32_t)(time - end_us) >= 0) return;
    if (saved_frames >= MAX_FRAMES) { failed = 1; return; }
    uint8_t chunk[8]; memcpy(chunk, "00dc", 4); dart_put32(chunk + 4, n);
    offsets[saved_frames] = 4 + movi_bytes; sizes[saved_frames] = n;
    if (write_exact(&video, chunk, 8) != FR_OK || write_exact(&video, bytes, n) != FR_OK) { failed = 1; return; }
    if (n & 1) { uint8_t zero = 0; if (write_exact(&video, &zero, 1) != FR_OK) { failed = 1; return; } }
    if (!saved_frames) first_frame = time;
    last_frame = time; if (n > biggest) biggest = n;
    uint8_t info[12]; dart_put32(info, saved_frames); dart_put32(info + 4, n); dart_put32(info + 8, camera_frames);
    log_packet(17, time, info, 12); saved_frames++; movi_bytes += 8 + n + (n & 1);
}
static void stop_stream(void)
{
    wire_send(DART_COMMAND, "IMU STOP", 8);
    uint32_t begin = board_micros();
    while (!remote_stopped && (uint32_t)(board_micros() - begin) < 30000) {}
    accept_imu = 0;
}
static void release_card(void)
{
    if (mounted) { f_mount(NULL, "0:", 0); mounted = 0; }
    fs_disk_write_enabled = 0; usb_tf_set_busy(0);
}
static void cancel_preparation(void)
{
    /* 仅删除本次尚未触发、由本程序新建的空会话，不碰任何已有采集文件。 */
    stop_stream();
    if (camera_running) { camera_stop(); camera_running = 0; }
    if (video_open) { f_close(&video); f_unlink(video_path); video_open = 0; }
    if (log_open) { f_close(&logfile); f_unlink(log_path); log_open = 0; }
    if (csv_open) { f_close(&csvfile); f_unlink(csv_path); csv_open = 0; }
    if (directory[0]) f_unlink(directory);
    release_card(); pre_count = 0;
}

static int export_csv(void)
{
    /* 高速采集结束后才转换CSV；避免每毫秒格式化文本影响采样和录像。 */
    FIL input; dart_parser decode = {0}; dart_packet packet;
    if (f_open(&input, log_path, FA_READ) != FR_OK) return -1;
    unsigned used = 0; int error = 0; UINT count;
    for (;;) {
        if (f_read(&input, convert_buffer, sizeof(convert_buffer), &count) != FR_OK) { error = 1; break; }
        if (!count) break;
        for (unsigned i = 0; i < count; i++) {
            if (dart_parse(&decode, convert_buffer[i], &packet) != 1) continue;
            if (packet.type != 16 || packet.length != 4 + sizeof(dart_imu)) continue;
            dart_imu sample; memcpy(&sample, packet.data + 4, sizeof(sample));
            char line[256]; int n = dart_imu_csv_line(line, sizeof(line), &sample, trigger_f1_us);
            if (n < 0 || (unsigned)n >= sizeof(line)) { error = 1; break; }
            if (used + (unsigned)n > sizeof(log_buffer)) {
                if (write_exact(&csvfile, log_buffer, used) != FR_OK) { error = 1; break; }
                used = 0;
            }
            memcpy(log_buffer + used, line, (unsigned)n); used += (unsigned)n; csv_rows++;
        }
        if (error) break;
        if (HAL_GetTick() - last_led_sent >= 500) send_led();
    }
    if (used && write_exact(&csvfile, log_buffer, used) != FR_OK) error = 1;
    if (f_close(&input) != FR_OK || decode.errors || decode.count || csv_rows != saved_imu) error = 1;
    return error ? -1 : 0;
}
static void write_info(void)
{
    FIL info;
    if (f_open(&info, info_path, FA_WRITE | FA_CREATE_NEW) != FR_OK) { failed = 1; return; }
    char text[2400];
    int n = snprintf(text, sizeof(text),
        "\xEF\xBB\xBF飞镖自动记录器 2.0\r\n"
        "会话编号：%lu\r\n启动方式：%s\r\n结果：%s\r\n"
        "触发阈值：三轴加速度合量严格大于 2.000g，连续至少20ms。\r\n"
        "采集时间：触发确认后10秒；IMU另保留至多约0.2秒触发前数据。\r\n"
        "触发时刻：F103=%lu us；H750接收=%lu us（各自开机计时，无真实日期）。\r\n"
        "录像：320x240 MJPEG，目标约25fps；保存帧数=%lu。\r\n"
        "IMU原始记录=%lu条；CSV=%lu行；序号缺口=%lu。\r\n"
        "IMU队列溢出=%lu；串口CRC错误=%lu；UART错误=%lu。\r\n"
        "录像期间相机丢帧=%lu；无效JPEG=%lu；相机错误位=%lu。\r\n"
        "数据：IMU.CSV可直接用Excel打开，表头含单位；IMU.DLG保留原始数据和视频帧时间。\r\n"
        "CSV时间：以触发确认样本为0秒，负数表示触发前缓存。\r\n"
        "角速度单位：度/秒；加速度单位：g，1g=9.80665m/s^2，静止合量约1g。\r\n"
        "XYZ为传感器原始轴，尚未做机体坐标变换、零偏校准或姿态解算。\r\n"
        "有效标志位：1陀螺有效，2加速度有效，4加速度新数据，8陀螺新数据，16 SPI错误，32采样迟到。\r\n"
        "刷新位只是读取瞬间的状态；1000Hz为定时读取频率，六轴并非硬件同步采样。\r\n"
        "合量触发与姿态无关，横向推动也可能触发；不是仅检测箭体轴向。\r\n"
        "视频帧时间为采集结束时间，非曝光开始时间；两个芯片时间由DLG同步包估算。\r\n"
        "文件时间不代表真实日期；每次上电只自动触发一次。完成后才可断电。\r\n",
        (unsigned long)session_token, automatic ? "加速度自动触发" : "电脑手动命令",
        failed ? "存在错误，请检查计数与原始日志" : "正常结束",
        (unsigned long)trigger_f1_us, (unsigned long)start_us,
        (unsigned long)saved_frames, (unsigned long)saved_imu, (unsigned long)csv_rows,
        (unsigned long)imu_gaps, (unsigned long)(imu_overflows - session_imu_overflows),
        (unsigned long)(parser.errors - session_crc_errors), (unsigned long)(uart_errors - session_uart_errors),
        (unsigned long)(camera_drops - start_camera_drops), (unsigned long)(camera_invalid - start_camera_invalid),
        (unsigned long)camera_errors);
    if (n < 0 || (unsigned)n >= sizeof(text) || write_exact(&info, text, (UINT)n) != FR_OK) failed = 1;
    if (f_close(&info) != FR_OK) failed = 1;
}
static void close_session(void)
{
    if (camera_running) { camera_stop(); camera_running = 0; }
    stop_stream(); drain_imu();
    if (camera_mode) {
        const uint8_t *bytes; uint32_t n, time; int slot;
        while ((slot = camera_take(&bytes, &n, &time)) >= 0) {
            if (!failed) save_frame(bytes, n, time); camera_release((unsigned)slot);
        }
        if (!saved_frames) failed = 1;
    }
    state = SAVING; send_led();
    if (video_open && !failed) {
        uint8_t header[224], index[16]; memcpy(index, "idx1", 4); dart_put32(index + 4, saved_frames * 16);
        if (write_exact(&video, index, 8) != FR_OK) failed = 1;
        for (unsigned i = 0; i < saved_frames && !failed; i++) {
            memcpy(index, "00dc", 4); dart_put32(index + 4, 16); dart_put32(index + 8, offsets[i]); dart_put32(index + 12, sizes[i]);
            if (write_exact(&video, index, 16) != FR_OK) failed = 1;
        }
        uint32_t bytes = (uint32_t)f_tell(&video);
        uint32_t span = saved_frames > 1 ? (uint32_t)((uint64_t)(last_frame - first_frame) * saved_frames / (saved_frames - 1)) : 40000;
        if (f_truncate(&video) != FR_OK) failed = 1;
        dart_avi_header(header, saved_frames, span, bytes, movi_bytes, biggest, 320, 240);
        if (f_lseek(&video, 0) != FR_OK || write_exact(&video, header, sizeof(header)) != FR_OK) failed = 1;
    }
    if (video_open) { if (f_close(&video) != FR_OK) failed = 1; video_open = 0; }
    char summary[76];
    snprintf(summary, sizeof(summary), "END frames=%lu imu=%lu gaps=%lu fail=%u",
             (unsigned long)saved_frames, (unsigned long)saved_imu, (unsigned long)imu_gaps, failed);
    log_event(summary);
    if (log_open) {
        if (flush_log() || f_truncate(&logfile) != FR_OK) failed = 1;
        if (f_close(&logfile) != FR_OK) failed = 1;
        log_open = 0;
    }
    if (csv_open) {
        if (export_csv()) failed = 1;
        if (f_truncate(&csvfile) != FR_OK) failed = 1;
        if (f_close(&csvfile) != FR_OK) failed = 1;
        csv_open = 0;
    }
    /* 转换完成后追加最终标记；FINAL 覆盖采集和CSV转换两部分结果。 */
    if (f_open(&logfile, log_path, FA_WRITE | FA_OPEN_EXISTING) == FR_OK) {
        log_open = 1;
        if (f_lseek(&logfile, f_size(&logfile)) != FR_OK) failed = 1;
        snprintf(summary, sizeof(summary), "FINAL failed=%u csv_rows=%lu", failed, (unsigned long)csv_rows);
        log_event(summary); if (flush_log()) failed = 1;
        if (f_close(&logfile) != FR_OK) failed = 1; log_open = 0;
    } else failed = 1;
    write_info(); release_card(); state = failed ? ERROR_STATE : DONE; send_led();
    reply("SESSION END id=%06lX frames=%lu imu=%lu failed=%u",
          (unsigned long)session_token, (unsigned long)saved_frames, (unsigned long)saved_imu, failed);
    reply("SESSION FILE %s CSV=%lu", directory[0] ? directory : csv_path, (unsigned long)csv_rows);
    reply("SESSION ERR gaps=%lu qdrop=%lu crc=%lu uart=%lu write_ms=%lu",
          (unsigned long)imu_gaps, (unsigned long)(imu_overflows - session_imu_overflows),
          (unsigned long)(parser.errors - session_crc_errors), (unsigned long)(uart_errors - session_uart_errors), (unsigned long)max_write_ms);
}

static int select_paths(uint32_t token, unsigned auto_mode)
{
    directory[0] = 0;
    if (auto_mode) {
        /* FAT 8.3兼容目录：REC00001、REC00002……，永不覆盖已有记录。 */
        DIR root; FILINFO item; uint32_t maximum = 0;
        if (f_opendir(&root, "0:") != FR_OK) return -1;
        FRESULT result;
        while ((result = f_readdir(&root, &item)) == FR_OK && item.fname[0]) {
            if (strlen(item.fname) == 8 && !memcmp(item.fname, "REC", 3) && strspn(item.fname + 3, "0123456789") == 5) {
                uint32_t value = (uint32_t)strtoul(item.fname + 3, NULL, 10); if (value > maximum) maximum = value;
            }
        }
        f_closedir(&root); if (result != FR_OK || maximum >= 99999) return -1;
        session_token = maximum + 1; snprintf(directory, sizeof(directory), "REC%05lu", (unsigned long)session_token);
        if (f_mkdir(directory) != FR_OK) { directory[0] = 0; return -1; }
        snprintf(video_path, sizeof(video_path), "%s/VIDEO.AVI", directory);
        snprintf(log_path, sizeof(log_path), "%s/IMU.DLG", directory);
        snprintf(csv_path, sizeof(csv_path), "%s/IMU.CSV", directory);
        snprintf(info_path, sizeof(info_path), "%s/INFO.TXT", directory);
    } else {
        session_token = token;
        snprintf(video_path, sizeof(video_path), "D%06lX.AVI", (unsigned long)token);
        snprintf(log_path, sizeof(log_path), "D%06lX.DLG", (unsigned long)token);
        snprintf(csv_path, sizeof(csv_path), "D%06lX.CSV", (unsigned long)token);
        snprintf(info_path, sizeof(info_path), "D%06lX.TXT", (unsigned long)token);
        FILINFO info;
        const char *paths[] = {video_path, log_path, csv_path, info_path};
        for (unsigned i = 0; i < 4; i++) if (f_stat(paths[i], &info) != FR_NO_FILE) return -1;
    }
    return 0;
}
static void prepare_session(uint32_t token, unsigned auto_mode, unsigned use_camera)
{
    if (state == RECORDING || state == SAVING || usb_tf_connected() || !board_sd_ready()) { reply("ERR BUSY/USB/SD"); return; }
    automatic = auto_mode; camera_mode = use_camera; failed = 0;
    saved_frames = saved_imu = imu_gaps = movi_bytes = biggest = log_used = log_seq = max_write_ms = csv_rows = 0;
    pre_head = pre_count = 0; first_imu = 1; memset(&trigger, 0, sizeof(trigger));
    session_imu_overflows = imu_overflows; session_crc_errors = parser.errors; session_uart_errors = uart_errors;
    directory[0] = 0; state = PREPARING; send_led(); usb_tf_set_busy(1);
    if (camera_mode && camera_configure()) goto fail;
    if (f_mount(&fs, "0:", 1) != FR_OK) goto fail;
    mounted = 1; FATFS *volume; DWORD clusters;
    if (f_getfree("0:", &clusters, &volume) != FR_OK || (uint64_t)clusters * volume->csize * 512 < 96u * 1024u * 1024u) goto fail;
    fs_disk_write_enabled = 1;
    if (select_paths(token, auto_mode)) goto fail;
    if (camera_mode) {
        if (f_open(&video, video_path, FA_WRITE | FA_CREATE_NEW) != FR_OK) goto fail;
        video_open = 1; if (f_expand(&video, 64u * 1024u * 1024u, 1) != FR_OK) goto fail;
        uint8_t header[224] = {0}; if (write_exact(&video, header, sizeof(header)) != FR_OK) goto fail;
    }
    if (f_open(&logfile, log_path, FA_WRITE | FA_CREATE_NEW) != FR_OK) goto fail;
    log_open = 1; if (f_expand(&logfile, 1024u * 1024u, 1) != FR_OK) goto fail;
    if (f_open(&csvfile, csv_path, FA_WRITE | FA_CREATE_NEW) != FR_OK) goto fail;
    csv_open = 1; if (f_expand(&csvfile, 4u * 1024u * 1024u, 1) != FR_OK) goto fail;
    if (write_exact(&csvfile, dart_imu_csv_header, (UINT)strlen(dart_imu_csv_header)) != FR_OK) goto fail;
    log_event("DART2 gyro=1000Hz/2000dps accel=1600Hz/24g image=320x240/25fps");
    log_event("CONFIG threshold_mg=2000 hold_us=20000 duration_us=10000000 pre=200");
    if (camera_mode) { if (camera_start()) goto fail; camera_running = 1; }
    remote_started = remote_stopped = 0; imu_head = imu_tail = 0;
    last_imu_rx = last_good_frame = prepare_us = board_micros(); accept_imu = 1;
    wire_send(DART_COMMAND, "IMU WATCH", 9); sync_request();
    reply("SESSION PREPARE auto=%u id=%06lX", automatic, (unsigned long)session_token);
    return;
fail:
    failed = 1; cancel_preparation(); state = ERROR_STATE; send_led();
    reply("ERR PREPARE camera/TF/space/file; no recording started");
}
static void status(void)
{
    reply("DART H750 v=2.0 state=%s sd=%u frames=%lu imu=%lu", state_names[state], board_sd_ready(),
          (unsigned long)saved_frames, (unsigned long)saved_imu);
    reply("AUTO threshold=2000mg hold=20ms duration=10s fired=%u disabled=%u", auto_fired, auto_disabled);
    reply("ERR cam=%lu drop=%lu imuq=%lu crc=%lu uart=%lu", (unsigned long)camera_errors,
          (unsigned long)camera_drops, (unsigned long)imu_overflows, (unsigned long)parser.errors, (unsigned long)uart_errors);
}
static void command(char *text)
{
    unsigned token, seconds; char tail;
    if (!strncmp(text, "PING ", 5)) reply("PONG %s", text + 5);
    else if (!strcmp(text, "STATUS") || !strcmp(text, "AUTO STATUS")) status();
    else if (!strcmp(text, "STOP") || !strcmp(text, "AUTO OFF")) {
        auto_disabled = 1;
        if (state == RECORDING) { end_us = board_micros(); close_session(); }
        else if (state == PREPARING || state == ARMED) { cancel_preparation(); state = DISARMED; send_led(); }
        else { if (state == WAIT_BOOT) state = DISARMED; reply("OK IDLE AUTO_OFF"); }
    } else if (!strcmp(text, "AUTO ARM")) {
        if (auto_fired || state == RECORDING || state == SAVING) reply("ERR AUTO ONE_SHOT; reboot to rearm");
        else { if (state == PREPARING || state == ARMED) cancel_preparation(); auto_disabled = 0; prepare_session(0, 1, 1); }
    } else if ((sscanf(text, "START %x %u %c", &token, &seconds, &tail) == 2 ||
                sscanf(text, "LOG START %x %u %c", &token, &seconds, &tail) == 2) && token <= 0xFFFFFF && seconds == 10) {
        if (state == PREPARING || state == ARMED) cancel_preparation();
        auto_disabled = 1; prepare_session(token, 0, strncmp(text, "LOG ", 4) != 0);
    } else if (!strcmp(text, "CAM ID") && state != RECORDING && state != PREPARING && state != ARMED && !usb_tf_connected()) camera_command("ID");
    else if (!strcmp(text, "CAM TIMING") && state != RECORDING && state != PREPARING && state != ARMED && !usb_tf_connected()) camera_timing();
    else if (!strcmp(text, "CAM INIT") && state != RECORDING && state != PREPARING && state != ARMED && !usb_tf_connected()) {
        int result = camera_configure(); reply("CAM CONFIG result=%d", result); if (!result) camera_timing();
    }
    else reply("COMMANDS STATUS AUTO OFF/ARM START/LOG START <6HEX> 10 STOP CAM ID");
}
static void process_replies(void)
{
    while (reply_tail != reply_head) {
        dart_packet p = replies[reply_tail]; reply_tail = (reply_tail + 1) & 7;
        if (p.type == DART_COMMAND) { char text[DART_PAYLOAD_MAX + 1]; memcpy(text, p.data, p.length); text[p.length] = 0; command(text); }
    }
}

int main(void)
{
    SCB_EnableICache(); HAL_Init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_boot_stage = 1; uart_init();
    int clock = board_clock_start(); g_boot_stage = 2; uart_init();
    HAL_Delay(200); int storage = board_storage_start(); g_boot_stage = 3;
    __HAL_RCC_D2SRAM1_CLK_ENABLE(); __HAL_RCC_D2SRAM2_CLK_ENABLE(); __HAL_RCC_D2SRAM3_CLK_ENABLE(); __DSB();
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); uart_init(); usb_tf_start();
    g_boot_stage = 4; uint32_t boot_tick = HAL_GetTick();
    reply("BOOT Dart AutoRecord H750 2.0 clock=%d storage=%d", clock, storage);
    if (clock || storage) state = ERROR_STATE;
    for (;;) {
        process_replies();
        if (HAL_GetTick() - last_led_sent >= 500) send_led();
        /* 电脑在原生USB端连接时只读卡，避免拿起板子读文件时误触发。 */
        if (usb_tf_connected() && (state == WAIT_BOOT || state == PREPARING || state == ARMED)) {
            if (state == PREPARING || state == ARMED) cancel_preparation();
            state = USB_READER; send_led();
        }
        if (state == WAIT_BOOT && !auto_disabled && HAL_GetTick() - boot_tick >= 2500) prepare_session(0, 1, 1);
        if (state == PREPARING || state == ARMED || state == RECORDING) {
            drain_imu();
            const uint8_t *bytes; uint32_t n, time; int slot = camera_mode ? camera_take(&bytes, &n, &time) : -1;
            if (slot >= 0) {
                last_good_frame = board_micros();
                if (state == RECORDING) save_frame(bytes, n, time);
                camera_release((unsigned)slot);
            }
            if (sync_ready && log_open) {
                uint8_t pair[12]; dart_put32(pair, sync_sent); dart_put32(pair + 4, sync_f1_time); dart_put32(pair + 8, sync_ready_time);
                log_packet(18, sync_ready_time, pair, 12); sync_ready = 0; sync_pending = 0;
            }
            if (state == PREPARING && remote_started && pre_count && (!camera_mode || camera_frames)) {
                if (automatic) { state = ARMED; memset(&trigger, 0, sizeof(trigger)); reply("AUTO READY threshold=2000mg hold=20ms folder=%s", directory); send_led(); }
                else { unsigned latest = (pre_head + DART_AUTO_PRE_SAMPLES - 1) % DART_AUTO_PRE_SAMPLES; begin_recording(&pre_trigger[latest]); }
            }
            if (state == PREPARING && (uint32_t)(board_micros() - prepare_us) > 2000000u) failed = 1;
            if (state == ARMED || state == RECORDING) {
                if ((uint32_t)(board_micros() - sync_sent) > 1000000u && !sync_pending) sync_request();
                if (sync_pending && (uint32_t)(board_micros() - sync_sent) > 100000u) sync_pending = 0;
                if (dart_is_stale(board_micros(), last_imu_rx, 1000000u) ||
                    (camera_mode && dart_is_stale(board_micros(), last_good_frame, 1000000u)) || camera_errors) failed = 1;
                if (imu_overflows != session_imu_overflows) failed = 1;
            }
            if (state == RECORDING && (failed || (int32_t)(board_micros() - end_us) >= 0)) close_session();
            else if (failed && (state == PREPARING || state == ARMED)) {
                cancel_preparation(); state = ERROR_STATE; send_led(); reply("ERR AUTO IMU/CAMERA/QUEUE; not armed");
            }
        }
    }
}
