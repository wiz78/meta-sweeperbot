//
// Created by simone on 3/16/24.
//

#include "movementPlanner.h"
#include "occupancyGrid.h"
#include "esp32Comm.h"
#include "lidar.h"
#include "log.h"

#include <utility>

static constexpr float ROBOT_LENGTH_MM = 240;
static constexpr float ROBOT_LENGTH = ROBOT_LENGTH_MM / 1000;
static constexpr float ROBOT_WIDTH_MM = 250;
static constexpr float ROBOT_WIDTH = ROBOT_WIDTH_MM / 1000;
static constexpr float ROBOT_Y_OFFSET_MM = 180; // LIDAR offset from robot front edge
static constexpr float ROBOT_Y_OFFSET = ROBOT_Y_OFFSET_MM / 1000;
static constexpr float ROBOT_ROTATION_Y_OFFSET = 0.140; // offset from robot front edge used to mark obstacles when rotating
static constexpr float ROBOT_BACK_Y_OFFSET = ROBOT_LENGTH - ROBOT_Y_OFFSET; // LIDAR offset from robot back edge
static constexpr int MIN_FRONT_DISTANCE = 250;
static constexpr int MIN_BACK_DISTANCE = MIN_FRONT_DISTANCE / 2;
static constexpr int BACKTRACK_LEN = 60;

MovementPlanner::MovementPlanner( OccupancyGrid& grid, Esp32Comm& esp32Comm, Lidar& lidar ) : grid{ grid }, esp32Comm{ esp32Comm }, lidar{ lidar }
{
}

MovementPlanner::MovementPlanner( const MovementPlanner& copy, OccupancyGrid& grid ) : grid{ grid }, esp32Comm{ copy.esp32Comm }, lidar{ copy.lidar }
{
    state = copy.state;
    currentMapPosition = copy.currentMapPosition;
    currentRotation = copy.currentRotation;
    targetRotation = copy.targetRotation;
    targetPosition = copy.targetPosition;
    targetDirection = copy.targetDirection;
    sweepDirection = copy.sweepDirection;
    nextLine = copy.nextLine;
    lastMovement = copy.lastMovement;
    plannedPath = copy.plannedPath;
    plannedPathStep = copy.plannedPathStep;
}

void MovementPlanner::update( UpdateReason reason, const Eigen::Vector2f& position, float rotation )
{
    currentMapPosition = grid.getMapPosition( position );
    currentRotation = rotation;

    if( reason != UpdateReason::NewLidarData )
        Log::debug( "MovementPlanner::update() - reason {}, state {}", static_cast<int>( reason ), static_cast<int>( state ));

    markVisited( position );

    if( reason == UpdateReason::StoppedByObstacle ) {

        markObstacle( position, rotation );

        if((( state == State::WaitRotation ) && ( fabs( rotation - targetRotation ) > toRadians( 0.4 ))) ||
           (( state == State::WaitUntilTargetReached ) && (( position - targetPosition ).norm() > grid.getResolution() )))
            plannedPath.clear();

        moveBack();

    } else switch( state ) {

        case State::ChooseTarget:
            plan( position, rotation );
            break;

        case State::WaitRotation:
            if( reason == UpdateReason::Stopped )
                moveStraightToTarget( position, rotation );
            break;

        case State::WaitUntilTargetReached:
            if( reason == UpdateReason::Stopped ) {

                if(( lastMovement == Movement::Forward ) && ( lidar.getMinDistance( Lidar::Sides::Front ) < MIN_FRONT_DISTANCE ))
                    moveBack( MIN_FRONT_DISTANCE - lidar.getMinDistance( Lidar::Sides::Front ));
                else if(( lastMovement == Movement::Backward ) && ( lidar.getMinDistance( Lidar::Sides::Back ) < MIN_BACK_DISTANCE ))
                    moveBack( MIN_BACK_DISTANCE - lidar.getMinDistance( Lidar::Sides::Back ));
                else
                    chooseTarget( position, rotation );
            }
            break;
    }
}

void MovementPlanner::moveBack( int extraDist )
{
    extraDist += BACKTRACK_LEN;

    move(( lastMovement == Movement::Forward ) ? -extraDist : extraDist );
}

Eigen::Vector2i MovementPlanner::getTargetPosition() const
{
    return grid.getMapPosition( targetPosition );
}

void MovementPlanner::clearTarget()
{
    Eigen::Vector2i oldTarget = getTargetPosition();

    if( grid.isValidPos( oldTarget ))
        grid.getCell( oldTarget ).clearTarget();
}

void MovementPlanner::markObstacle( const Eigen::Vector2f& position, float rotation )
{
    Eigen::Vector2f heading = Eigen::Rotation2D( rotation ) * Eigen::Vector2f{ 0, -1 };
    Eigen::Vector2f worldPos = position;
    Eigen::Vector2i mapPos;

    switch( lastMovement ) {

        case Movement::Forward:
            worldPos += ROBOT_Y_OFFSET * heading;
            break;

        case Movement::Backward:
            worldPos -= ROBOT_BACK_Y_OFFSET * heading;
            break;

        case Movement::CW:
            worldPos += ROBOT_ROTATION_Y_OFFSET * heading;
            // shift to a side, along the heading normal vector
            worldPos -= ( ROBOT_WIDTH / 2 ) * Eigen::Vector2f{ heading.y(), -heading.x() };
            break;

        case Movement::CCW:
            worldPos += ROBOT_ROTATION_Y_OFFSET * heading;
            worldPos += ( ROBOT_WIDTH / 2 ) * Eigen::Vector2f{ heading.y(), -heading.x() };
            break;
    }

    mapPos = grid.getMapPosition( worldPos );

    Log::debug( "Marking obstacle" );
    Log::debug( "position: {} {}", position.x(), position.y() );
    Log::debug( "heading: {} {}", heading.x(), heading.y() );
    Log::debug( "worldPos: {} {}", worldPos.x(), worldPos.y() );
    Log::debug( "mapPos: {} {}", mapPos.x(), mapPos.y() );

    if( grid.isValidPos( mapPos ))
        grid.getCell( mapPos ).forceOccupied();
}

void MovementPlanner::markDangerZone()
{
    for( int i = grid.getXSize() - 1; i >= 0; i-- )
        for( int j = grid.getYSize() - 1; j >= 0; j-- )
            grid.getCell( i, j ).clearDangerZone();

    for( int i = grid.getXSize() - 1; i >= 0; i-- )
        for( int j = grid.getYSize() - 1; j >= 0; j-- )
            if( grid.getCell( i, j ).isOccupied() )
                for( int di = std::max( i - 2, 0 ); di < std::min( i + 3, grid.getXSize() ); di++ )
                    for( int dj = std::max( j - 2, 0 ); dj < std::min( j + 3, grid.getYSize() ); dj++ ) {
                        OccupancyGridCell& cell = grid.getCell( di, dj );

                        if( !cell.isOccupied() )
                            cell.markDangerZone();
                    }
}

void MovementPlanner::plan( const Eigen::Vector2f& position, float rotation )
{
    static constexpr int VARIANTS_NUM = 16;
    static constexpr int LOOKAHEAD_STEPS = 30;
    std::vector<OccupancyGrid *> grids;
    std::vector<MovementPlanner *> candidates;
    float bestCost = __FLT_MAX__;
    MovementPlanner *bestCandidate = nullptr;

    for( int i = 0; i < VARIANTS_NUM; i++ ) {
        grids.emplace_back( new OccupancyGrid( grid ));
        candidates.emplace_back( new MovementPlanner( *this, *grids[ i ] ));
    }

    candidates[ 0 ]->sweepDirection = oppositeDirection( candidates[ 0 ]->sweepDirection );

    candidates[ 1 ]->sweepDirection = oppositeDirection( candidates[ 1 ]->sweepDirection );
    candidates[ 1 ]->targetDirection = oppositeDirection( candidates[ 1 ]->targetDirection );

    candidates[ 2 ]->targetDirection = oppositeDirection( candidates[ 2 ]->targetDirection );

    for( int i = 4; i < 8; i++ ) {
        candidates[ i ]->sweepDirection = orthogonalDirection( candidates[ i - 4 ]->sweepDirection );
        candidates[ i ]->targetDirection = orthogonalDirection( candidates[ i - 4 ]->targetDirection );
    }

    for( int i = VARIANTS_NUM / 2; i < VARIANTS_NUM; i++ ) {
        candidates[ i ]->sweepDirection = candidates[ i - ( VARIANTS_NUM / 2 ) ]->sweepDirection;
        candidates[ i ]->targetDirection = candidates[ i - ( VARIANTS_NUM / 2 ) ]->targetDirection;
        candidates[ i ]->nextLine = !candidates[ i - ( VARIANTS_NUM / 2 ) ]->nextLine;
    }

    for( auto& candidate : candidates ) {
        Eigen::Vector2f pos = position;
        float rot = rotation;
        float cost = 0;

        candidate->simulating = true;

        for( int i = 0; i < LOOKAHEAD_STEPS; i++ ) {

            candidate->chooseTarget( pos, rot );

            if( candidate->plannedPath.empty() )
                break;

            cost += candidate->evaluatePlannedPath();

            pos = candidate->targetPosition;
            rot = candidate->currentRotation;
        }

        if( cost < bestCost ) {
            bestCost = cost;
            bestCandidate = candidate;
        }
    }

    if( bestCandidate ) {
        sweepDirection = bestCandidate->sweepDirection;
        targetDirection = bestCandidate->targetDirection;
        nextLine = bestCandidate->nextLine;
    }

    for( MovementPlanner *p : candidates )
        delete p;

    for( OccupancyGrid *g : grids )
        delete g;

    chooseTarget( position, rotation );
}

float MovementPlanner::evaluatePlannedPath()
{
    float cost = 0, dist = 0.1;

    while( hasPlannedPath() ) {
        Eigen::Vector2i nextPos = plannedPath[ plannedPathStep ];

        dist += ( nextPos - currentMapPosition ).cast<float>().norm();

        targetPosition = grid.getWorldPosition( nextPos );

        moveToTarget( grid.getWorldPosition( currentMapPosition ), currentRotation );

        currentMapPosition = nextPos;
        currentRotation += targetRotation;

        plannedPathStep++;
        cost++;

        advancePlannedPathStepUntilTurn();
    }

    if( targetRotation == 0 )
        cost--;

    cost *= 20;
    cost += 30 / dist;

    for( const auto& p : plannedPath ) {
        Eigen::Vector2f worldPos = grid.getWorldPosition( p );

        cost += static_cast<float>( markVisited( worldPos ));
        cost += static_cast<float>( grid.getCellConst( p ).getFirstFreeTime() ) * 0.5F;
    }

    plannedPath.clear();

    return cost;
}

void MovementPlanner::chooseTarget( const Eigen::Vector2f& position, float rotation )
{
    Eigen::Vector2i targetInMap{ -1, 0 };

    state = State::ChooseTarget;

    markDangerZone();

    if( !plannedPath.empty() && ( ++plannedPathStep < plannedPath.size() )) {

        advancePlannedPathStepUntilTurn();

        if( hasPlannedPath() )
            targetInMap = plannedPath[ plannedPathStep ];
        else if( needNewPlan ) {

            needNewPlan = false;

            plan( position, rotation );
            return;
        }
    }

    if( nextLine ) {

        targetInMap = findNextLineCell( currentMapPosition, 1 );

        if( targetInMap.x() < 0 ) {
            sweepDirection = oppositeDirection( sweepDirection );
            targetInMap = findNextLineCell( currentMapPosition, 1 );
        }

        makePlannedPath( targetInMap, sweepDirection );

        nextLine = false;
    }

    if( targetInMap.x() < 0 ) {

        for( int i = 0; ( i < 2 ) && ( targetInMap.x() < 0 ); i++ ) {
            targetInMap = findFarthestUnvisitedCell( currentMapPosition, targetDirection );
            targetDirection = oppositeDirection( targetDirection );
        }

        if( targetInMap.x() >= 0 ) {
            nextLine = true;
            makePlannedPath( targetInMap, oppositeDirection( targetDirection ));
        }
    }

    if( targetInMap.x() < 0 ) {

        setPlannedPath( grid.findClosestUnvisitedCell( currentMapPosition ));

        if( hasPlannedPath() )
            targetInMap = plannedPath[ plannedPathStep ];

        needNewPlan = true;
    }

    if( !simulating )
        Log::debug( "targetInMap = {}, {}", targetInMap.x(), targetInMap.y() );

    if( targetInMap.x() >= 0 ) {

        clearTarget();
        grid.getCell( targetInMap ).markTarget();

        moveToTarget( targetInMap, position, rotation );
    }
}

void MovementPlanner::makePlannedPath( Eigen::Vector2i targetInMap, WalkDirection direction )
{
    if( targetInMap.x() >= 0 ) {
        Eigen::Vector2i pos = currentMapPosition;

        plannedPath.clear();

        while( targetInMap != pos ) {

            pos += directionVectors[ static_cast<int>( direction ) ];

            plannedPath.emplace_back( pos );
        }

        plannedPathStep = static_cast<int>( plannedPath.size() ) - 1;
    }
}

Eigen::Vector2i MovementPlanner::findFarthestUnvisitedCell( Eigen::Vector2i mapPos, WalkDirection direction ) const
{
    Eigen::Vector2i ret{ -1, -1 };
    Eigen::Vector2i ds = directionVectors[ static_cast<int>( direction ) ];

    for( mapPos += ds; grid.isValidPos( mapPos ); mapPos += ds ) {
        const OccupancyGridCell& cell = grid.getCell( mapPos.x(), mapPos.y() );

        if( cell.isOccupied() || cell.isDangerZone() )
            break;

        if( cell.isFree() && !cell.isVisited() )
            ret = mapPos;
    }

    if( !simulating )
        Log::debug( "findFarthestUnvisitedCell - direction: {} - ret: {}, {}", static_cast<int>( direction ), ret.x(), ret.y() );

    return ret;
}

Eigen::Vector2i MovementPlanner::findNextLineCell( Eigen::Vector2i mapPos, int offset ) const
{
    Eigen::Vector2i ret{ -1, -1 };

    mapPos += directionVectors[ static_cast<int>( sweepDirection ) ] * 3 * offset;

    if( grid.isValidPos( mapPos )) {
        const OccupancyGridCell& cell = grid.getCell( mapPos.x(), mapPos.y() );

        if( cell.isFree() && !cell.isDangerZone() && !cell.isVisited() )
            ret = mapPos;
    }

    if( !simulating )
        Log::debug( "findNextLineCell - direction: {} - ret: {}, {}", static_cast<int>( sweepDirection ), ret.x(), ret.y() );

    return ret;
}

void MovementPlanner::setPlannedPath( MiniPath p )
{
    plannedPath = std::move( p );
    plannedPathStep = 0;

    advancePlannedPathStepUntilTurn();
}

void MovementPlanner::advancePlannedPathStepUntilTurn()
{
    if( hasPlannedPath() ) {
        Eigen::Vector2i pos = plannedPath[ plannedPathStep ];
        int i = plannedPathStep + 1;

        while(( i < plannedPath.size() ) && (( plannedPath[ i ].x() == pos.x() ) || ( plannedPath[ i ].y() == pos.y() )))
            i++;

        plannedPathStep = i - 1;
    }
}

void MovementPlanner::moveToTarget( Eigen::Vector2i targetInMap, Eigen::Vector2f position, float rotation )
{
    targetPosition = grid.getWorldPosition( std::move( targetInMap ));

    moveToTarget( std::move( position ), rotation );
}

void MovementPlanner::moveToTarget( Eigen::Vector2f position, float rotation )
{
    Eigen::Vector2f heading = Eigen::Rotation2D( rotation ) * Eigen::Vector2f{ 0, -1 };
    Eigen::Vector2f desiredHeading = ( targetPosition - position ).normalized();
    float det = ( heading.x() * desiredHeading.y() ) - ( heading.y() * desiredHeading.x() );
    float newRotation = atan2( det, desiredHeading.dot( heading ));

    if( newRotation > M_PI )
        newRotation = M_PI - newRotation;
    else if( newRotation < -M_PI )
        newRotation = -static_cast<float>( newRotation + M_PI );

    if( fabs( newRotation ) > __FLT_EPSILON__ )
        rotate( newRotation );
    else {

        targetRotation = 0;

        moveStraightToTarget( position, rotation );
    }
}

void MovementPlanner::rotate( float angle )
{
    static constexpr int halfRobotWidth = ROBOT_WIDTH_MM / 2;
    int minDistLeft = lidar.getMinDistance( Lidar::Sides::Left );
    int minDistRight = lidar.getMinDistance( Lidar::Sides::Right );

    if((( angle > M_PI_2 ) && ( minDistRight < halfRobotWidth ) && ( minDistLeft > halfRobotWidth )) ||
       (( angle < M_PI_2 ) && ( minDistLeft < halfRobotWidth ) && ( minDistRight > halfRobotWidth )))
        angle += M_PI * (( angle > 0 ) ? -2 : 2 );

    state = State::WaitRotation;
    lastMovement = ( angle > 0 ) ? Movement::CW : Movement::CCW;
    targetRotation = angle;

    if( !simulating ) {
        float absAngle = fabs( angle );
        auto *anglePtr = reinterpret_cast<unsigned int *>( &absAngle );
        char data[] = { Esp32Comm::CMD_ROTATE, static_cast<char>(( angle > 0 ) ? 1 : -1 ),
                        static_cast<char>( *anglePtr ), static_cast<char>( *anglePtr >> 8 ),
                        static_cast<char>( *anglePtr >> 16 ), static_cast<char>( *anglePtr >> 24 ) };

        esp32Comm.writePacket( data, sizeof( data ));

        Log::debug( "Rotating {}° ({} - l: {} mm - r: {} mm)", toDegrees( angle ), angle, minDistLeft, minDistRight );
    }
}

void MovementPlanner::moveStraightToTarget( const Eigen::Vector2f& position, float rotation )
{
    Eigen::Vector2f movement = targetPosition - position;
    int distance = static_cast<int>( round( movement.norm() * 1000 ));

    distance -= ROBOT_Y_OFFSET_MM - ( ROBOT_LENGTH_MM / 2 );

    if( distance > 0 )
        move( distance );
    else {
        markVisited( targetPosition );
        chooseTarget( position, rotation );
    }
}

void MovementPlanner::move( int distance )
{
    state = State::WaitUntilTargetReached;
    lastMovement = ( distance > 0 ) ? Movement::Forward : Movement::Backward;

    if( !simulating ) {
        char data[] = { Esp32Comm::CMD_MOVE, static_cast<char>( distance ), static_cast<char>( distance >> 8 ),
                        static_cast<char>( distance >> 16 ), static_cast<char>( distance >> 24 ) };

        esp32Comm.writePacket( data, sizeof( data ));
    }
}

int MovementPlanner::markVisited( Eigen::Vector2f position )
{
    Eigen::Vector2i mapPos = grid.getMapPosition( std::move( position ));
    int alreadyVisited = 0;

    if( grid.isValidPos( mapPos ))
        for( int x = std::max( mapPos.x() - 2, 0 ); x < std::min( mapPos.x() + 3, grid.getXSize() ); x++ )
            for( int y = std::max( mapPos.y() - 2, 0 ); y < std::min( mapPos.y() + 3, grid.getYSize() ); y++ ) {
                OccupancyGridCell& cell = grid.getCell( x, y );

                if( cell.isVisited() && ( mapPos.x() == x ) && ( mapPos.y() == y ))
                    alreadyVisited++;
                else
                    cell.markVisited();
            }

    return alreadyVisited;
}