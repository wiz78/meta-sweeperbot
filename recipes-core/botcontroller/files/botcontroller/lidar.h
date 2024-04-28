//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_LIDAR_H
#define BOTCONTROLLER_LIDAR_H

#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include "lidarTypes.h"

class MainController;

class Lidar
{
public:
    enum class Sides { Front = 0, Right = 1, Back = 2, Left = 3 };

    explicit Lidar( MainController& mainController );

    void flush();

    bool processFrame( const LiDARFrame *frame, size_t size );

    [[nodiscard]] std::vector<PointData> getPoints() const { return { fullScan, fullScan + fullScanPoints }; }

    [[nodiscard]] int getMinDistance( Sides side ) const { return minDistances[ static_cast<int>( side ) ].load( std::memory_order_relaxed ); }

private:
    static constexpr std::size_t tmp_lidar_scan_data_len = 680;
    static constexpr int lidar_measure_freq_ = 4000;
    MainController& mainController;
    PointData tmp_lidar_scan_data[ tmp_lidar_scan_data_len ]{};
    std::size_t tmp_lidar_scan_data_front = 0;
    std::size_t tmp_lidar_scan_data_back = 0;
    PointData fullScan[ tmp_lidar_scan_data_len ]{};
    std::size_t fullScanPoints = 0;
    struct __attribute__((packed)) {
        unsigned char cmd;
        uint16_t minDistances[4];
    } minDistancesPacket{};
    std::atomic<int> minDistances[4];

    [[nodiscard]] static uint8_t calCRC8( const uint8_t *p, uint8_t len );

    bool storePacket( const LiDARMeasureDataType& dataPkg );
    bool assembleScan( double speed );
    void resetDistances();
};

#endif //BOTCONTROLLER_LIDAR_H
