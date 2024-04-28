//
// Created by simone on 3/16/24.
//

#include "mainController.h"
#include "movementPlanner.h"
#include "occupancyGrid.h"
#include "floorMap.h"
#include "esp32Comm.h"
#include "telemetry.h"
#include "lidar.h"
#include "log.h"

#include <thread>

MainController::MainController()
{
    telemetry = new Telemetry( *this );
    esp32Comm = new Esp32Comm( *this );
    lidar = new Lidar( *this );
    map = std::make_shared<FloorMap>();

    std::thread( &MainController::mappingThread, this ).detach();
}

void MainController::setExploring( bool x )
{
    std::lock_guard<std::mutex> lk( mappingMutex );

    if( x != explore ) {

        if( x ) {

            recreateMap.store( true, std::memory_order_relaxed );

            Log::info( "Starting autonomous exploration" );

        } else {
            std::shared_ptr<MovementPlanner> currentPlanner = planner.load( std::memory_order_relaxed );

            if( currentPlanner )
                currentPlanner->clearTarget();

            Log::info( "Stopping autonomous exploration" );
        }

        explore.store( x, std::memory_order_relaxed );
    }

    if( !x )
        esp32Comm->cmd( Esp32Comm::CMD_STOP );
}

void MainController::processLidarFrame( const LiDARFrame *frame, size_t size )
{
    if( lidar->processFrame( frame, size )) {
        std::lock_guard<std::mutex> lk( mappingMutex );

        if( mappingBusy.load( std::memory_order_relaxed ))
            Log::warning( "Dropping point cloud - mapping thread busy" );
        else {

            pointsToProcess = lidar->getPoints();
            yawToProcess = imuYaw.load( std::memory_order_relaxed );

            mappingCV.notify_one();
        }
    }
}

void MainController::mappingThread()
{
    for(;;) {
        std::unique_lock lk( mappingMutex );
        std::shared_ptr<FloorMap> currentMap;

        mappingCV.wait( lk );

        mappingBusy.store( true, std::memory_order_relaxed );

        lk.unlock();

        if( recreateMap.exchange( false, std::memory_order_relaxed )) {

            map.store( std::make_shared<FloorMap>(), std::memory_order_relaxed );
            planner.store( nullptr, std::memory_order_relaxed );

            firstMapRotation = true;
            mapRotationOffset = 0;

            telemetry->pushPacket( Telemetry::PacketType::RecreateMap );

        } else if( !firstMapRotation && map.load( std::memory_order_relaxed )->setIMURotation( yawToProcess + mapRotationOffset ))
            firstMapRotation = true;

        currentMap = map.load( std::memory_order_relaxed );

        currentMap->process( pointsToProcess );

        if( firstMapRotation ) {
            float angle = currentMap->getRotation();

            mapRotationOffset = angle;
            firstMapRotation = std::fabs( mapRotationOffset ) < 0.00001;
            mapRotationOffset -= yawToProcess;
        }

        updatePlannerWithReason( MovementPlanner::UpdateReason::NewLidarData );

        telemetry->sendYaw( yawToProcess );
        telemetry->sendLIDARPointCloud( pointsToProcess );
        telemetry->sendTransformationMatrix( currentMap->getTranformationMatrix() );

        readyForMapping();
    }
}

void MainController::readyForMapping()
{
    std::lock_guard<std::mutex> lk( mappingMutex );

    pointsToProcess.clear();

    mappingBusy.store( false, std::memory_order_relaxed );
}

void MainController::updatePlannerWithReason( MovementPlanner::UpdateReason reason )
{
    std::shared_ptr<FloorMap> currentMap = map.load( std::memory_order_relaxed );
    std::shared_ptr<MovementPlanner> currentPlanner = planner.load( std::memory_order_relaxed );

    if( explore.load( std::memory_order_relaxed ) && !currentPlanner && currentMap->hasGrid() ) {

        currentPlanner = std::make_shared<MovementPlanner>( currentMap->getGrid(), *esp32Comm, *lidar );

        planner.store( currentPlanner, std::memory_order_relaxed );
    }

    if( currentPlanner ) {

        currentPlanner->update( reason, currentMap->getPosition(), currentMap->getRotation() );

        if( !explore.load( std::memory_order_relaxed ))
            planner.store( nullptr, std::memory_order_relaxed );
    }

    if( currentMap->hasGrid() )
        telemetry->sendGrid( currentMap->getGridConst() );
}

void MainController::setTarget( Eigen::Vector2i pos )
{
    std::shared_ptr<FloorMap> currentMap = map.load( std::memory_order_relaxed );

    if( currentMap->hasGrid() && currentMap->getGrid().isValidPos( pos )) {
        std::shared_ptr<MovementPlanner> currentPlanner = std::make_shared<MovementPlanner>( currentMap->getGrid(), *esp32Comm, *lidar );

        explore.store( false, std::memory_order_relaxed );
        planner.store( currentPlanner, std::memory_order_relaxed );

        currentPlanner->moveToTarget( std::move( pos ), currentMap->getPosition(), currentMap->getRotation() );
    }
}