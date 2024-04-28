//
// Created by simone on 3/18/24.
//

#ifndef BOTCONTROLLER_TELEMETRY_H
#define BOTCONTROLLER_TELEMETRY_H

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <vector>

#include <Eigen/Geometry>

#include "floorMap.h"

class MainController;
class OccupancyGrid;
class PointData;

class Telemetry
{
public:
    enum class [[maybe_unused]] PacketType
    {
        Undefined = -1,
        LIDAR = 0,
        IMU = 1,
        LIDARFlush = 2,
        Stopped = 3,
        StoppedByObstacle = 4,
        Rotation = 5,
        OccupancyGrid = 6,
        RecreateMap = 7,
        TransformationMatrix = 8,
        Yaw = 9,
        SetTarget = 10,
        LIDARFrame = 0x54,
        ESPFirmwareGotPacket = 252,
        ESPFirmwareStop = 253,
        ESPReady = 254,
        LOG = 255,
    };

    explicit Telemetry( MainController& mainController );

    void pushPacket( const std::vector<char>& data ) { pushPacket( std::string{ data.data(), data.data() + data.size() } ); }
    void pushPacket( PacketType type ) { auto data = static_cast<uint8_t>( type ); pushPacket( std::string{ &data, &data + 1 } ); }

    void sendLIDARPointCloud( const std::vector<PointData>& points );
    void sendGrid( const OccupancyGrid& grid );
    void sendTransformationMatrix( FloorMap::PM::TransformationParameters matrix );
    void sendYaw( float yaw );

private:
    struct __attribute__((packed)) PacketHeader
    {
        uint32_t size;
        int8_t packetType;
    };

    struct __attribute__((packed)) SetTargetPacket
    {
        struct PacketHeader header;
        uint32_t x;
        uint32_t y;
    };

    MainController& mainController;
    int sock = -1;
    bool firstConnectionAttempt = true;
    std::chrono::steady_clock::time_point lastConnectTime{};
    std::chrono::steady_clock::time_point lastGridTime{};
    std::queue<std::string> packets;
    std::mutex packetsMutex;
    std::condition_variable cv;
    std::mutex mutex;

#if defined(TELEMETRY_HOST) && defined(TELEMETRY_PORT)
    [[noreturn]] void telemetryThread();
    [[noreturn]] void recvCommandsThread();

    void connect();
#endif

    [[nodiscard]] bool sendData( const void *buffer, size_t len ) const;
    [[nodiscard]] bool recvData( void *buffer, size_t len ) const;

    void pushPacket( std::string packet );
    void pushPacket( const std::ostringstream& packet ) { pushPacket( packet.str() ); }
    [[nodiscard]] std::string popPacket();

    void setTarget( const struct SetTargetPacket *packet ) const;
};

#endif //BOTCONTROLLER_TELEMETRY_H
