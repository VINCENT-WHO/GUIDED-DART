#ifndef DART_PROTOCOL_H
#define DART_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#define DART_VERSION 1u
#define DART_PAYLOAD_MAX 80u
#define DART_WIRE_MAX 96u
enum { DART_IMU=1, DART_COMMAND=2, DART_REPLY=3, DART_SYNC=4 };
typedef struct { uint8_t type; uint32_t seq; uint16_t length; uint8_t data[DART_PAYLOAD_MAX]; } dart_packet;
typedef struct { uint8_t data[DART_WIRE_MAX]; unsigned count,discard; uint32_t errors; } dart_parser;
typedef struct {
    uint32_t sequence, time_us, accel_sensor_time;
    int16_t gyro[3], accel[3];
    uint16_t flags, reserved;
    uint32_t missed_ticks;
} dart_imu;
_Static_assert(sizeof(dart_imu)==32, "IMU format must stay 32 bytes");
enum { IMU_GYRO_VALID=1, IMU_ACCEL_VALID=2, IMU_ACCEL_FRESH=4, IMU_GYRO_FRESH=8,
       IMU_SPI_ERROR=16, IMU_SCHEDULE_LATE=32 };
uint16_t dart_crc16(const void *data,size_t length);
size_t dart_encode(uint8_t type,uint32_t seq,const void *data,size_t length,uint8_t out[DART_WIRE_MAX]);
int dart_parse(dart_parser *parser,uint8_t byte,dart_packet *out);
void dart_put32(uint8_t *p,uint32_t v);
uint32_t dart_get32(const uint8_t *p);
#endif
