//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_MAINCONTROLLER_H
#define BOTCONTROLLER_MAINCONTROLLER_H

#include "lidarTypes.h"
#include "movementPlanner.h"

#include <condition_variable>
#include <atomic>
#include <vector>
#include <memory>
#include <mutex>

class Lidar;
class FloorMap;
class Esp32Comm;
class Telemetry;
class OccupancyGrid;

class MainController final
{
public:
    MainController();

    [[nodiscard]] Lidar& getLidar() { return *lidar; }
    [[nodiscard]] Esp32Comm& getEsp32Comm() { return *esp32Comm; }
    [[nodiscard]] Telemetry& getTelemetry() { return *telemetry; }

    void setExploring( bool x );
    void setIMUYaw( float x ) { imuYaw.store( x, std::memory_order_relaxed ); }

    void processLidarFrame( const LiDARFrame *frame, size_t size );
    void updatePlannerWithReason( MovementPlanner::UpdateReason reason );

    void setTarget( Eigen::Vector2i pos );

private:
    Esp32Comm *esp32Comm;
    Lidar *lidar;
    std::atomic<std::shared_ptr<FloorMap>> map;
    std::atomic<std::shared_ptr<MovementPlanner>> planner = nullptr;
    Telemetry *telemetry;
    std::atomic<bool> explore = false;
    std::atomic<bool> recreateMap = false;
    bool firstMapRotation = true;
    float mapRotationOffset = 0;
    std::atomic<float> imuYaw = 0;
    std::mutex mappingMutex;
    std::condition_variable mappingCV;
    std::vector<PointData> pointsToProcess;
    float yawToProcess = 0;
    std::atomic<bool> mappingBusy = false;

    [[noreturn]] void mappingThread();
    void readyForMapping();
};

#endif //BOTCONTROLLER_MAINCONTROLLER_H
