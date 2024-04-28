//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_LIDARTYPES_H
#define BOTCONTROLLER_LIDARTYPES_H

#include <cstdint>
#include <cmath>
#include <vector>

#define LIDAR_POINT_PER_PACK    12
#define LIDAR_HEADER            0x54
#define LIDAR_DATA_PKG_INFO     0x2C
#define LIDAR_HEALTH_PKG_INFO   0xE0
#define LIDAR_MANUFACT_PKG_INF  0x0F

typedef struct  __attribute__((packed)) {
    uint8_t header;
    uint8_t information;
    uint16_t speed;
    uint16_t product_version;
    uint32_t sn_high;
    uint32_t sn_low;
    uint32_t hardware_version;
    uint32_t firmware_version;
    uint8_t crc8;
} LiDARManufactureInfoType;

typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t intensity;
} LidarPointStructType;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructType point[ LIDAR_POINT_PER_PACK ];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARMeasureDataType;

typedef struct __attribute__((packed)) {
    uint8_t  header;
    uint8_t  information;
    uint8_t error_code;
    uint8_t  crc8;
} LiDARHealthInfoType;

typedef union __attribute__((packed)) {
    LiDARManufactureInfoType manufacture;
    LiDARMeasureDataType measure;
    LiDARHealthInfoType health;
} LiDARFrame;

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)

typedef struct __attribute__((packed)) PointData
{
    // Polar coordinate representation
    float angle;        // Angle ranges from 0 to 359 degrees
    uint16_t distance;  // Distance is measured in millimeters
    uint8_t intensity;  // Intensity is 0 to 255
    // Cartesian coordinate representation
    double x;
    double y;

    void computeXY()
    {
        double sx = distance + 5.9;
        double sy = ( distance * 0.11923 ) - 18.975571;
        double shift = atan( sy / sx ) * 180.f / 3.14159;

        double angle2 = ANGLE_TO_RADIAN( angle - shift ) - M_PI_2;

        x = cos( angle2 ) * distance;
        y = sin( angle2 ) * distance;
    }
} PointData;

typedef std::vector<PointData> Points2D;

#endif //BOTCONTROLLER_LIDARTYPES_H
