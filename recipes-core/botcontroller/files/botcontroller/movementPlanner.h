//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_MOVEMENTPLANNER_H
#define BOTCONTROLLER_MOVEMENTPLANNER_H

#include <Eigen/Geometry>

class Esp32Comm;
class Lidar;
class OccupancyGrid;

class MovementPlanner
{
public:
    typedef std::function<void ( const char *, size_t )> CommandSendFunc;

    enum class UpdateReason
    {
        NewLidarData,
        Stopped,
        StoppedByObstacle,
    };

    explicit MovementPlanner( OccupancyGrid& grid, Esp32Comm& esp32Comm, Lidar& lidar );

    void update( UpdateReason reason, const Eigen::Vector2f& position, float rotation );

    [[nodiscard]] Eigen::Vector2i getTargetPosition() const;
    void clearTarget();

    void moveToTarget( Eigen::Vector2i targetInMap, Eigen::Vector2f position, float rotation );

private:
    enum class State {
        ChooseTarget,
        WaitRotation,
        WaitUntilTargetReached,
    };

    enum class WalkDirection { North, West, South, East };
    enum class Movement { Forward, Backward, CW, CCW };

    typedef std::vector<Eigen::Vector2i> MiniPath;

    static inline const Eigen::Vector2i directionVectors[] = { Eigen::Vector2i{ 0, -1 }, Eigen::Vector2i{ -1, 0 }, Eigen::Vector2i{ 0, 1 }, Eigen::Vector2i{ 1, 0 } };

    OccupancyGrid& grid;
    Esp32Comm& esp32Comm;
    Lidar& lidar;
    State state = State::ChooseTarget;
    Eigen::Vector2i currentMapPosition;
    float currentRotation = 0;
    float targetRotation = 0;
    Eigen::Vector2f targetPosition;
    WalkDirection targetDirection = WalkDirection::North;
    WalkDirection sweepDirection = WalkDirection::East;
    bool nextLine = false;
    Movement lastMovement = Movement::Forward;
    MiniPath plannedPath;
    int plannedPathStep = 0;
    bool simulating = false;
    bool needNewPlan = false;

    MovementPlanner( const MovementPlanner& copy, OccupancyGrid& grid );

    static constexpr double toRadians( double deg ) { return deg * M_PI / 180; }
    static constexpr double toDegrees( double deg ) { return deg * 180 / M_PI; }

    static constexpr WalkDirection oppositeDirection( WalkDirection dir ) { return static_cast<WalkDirection>(( static_cast<int>( dir ) + 2 ) % 4 ); }
    static constexpr WalkDirection orthogonalDirection( WalkDirection dir ) { return static_cast<WalkDirection>(( static_cast<int>( dir ) + 1 ) % 4 ); }

    void moveBack( int extraDist = 0 );

    void markObstacle( const Eigen::Vector2f& position, float rotation );
    void markDangerZone();

    void plan( const Eigen::Vector2f& position, float rotation );
    float evaluatePlannedPath();

    void chooseTarget( const Eigen::Vector2f& position, float rotation );
    void makePlannedPath( Eigen::Vector2i targetInMap, WalkDirection direction );
    [[nodiscard]] Eigen::Vector2i findFarthestUnvisitedCell( Eigen::Vector2i mapPos, WalkDirection direction ) const;
    [[nodiscard]] Eigen::Vector2i findNextLineCell( Eigen::Vector2i mapPos, int offset ) const;

    void setPlannedPath( MiniPath p );
    void advancePlannedPathStepUntilTurn();
    [[nodiscard]] bool hasPlannedPath() const { return !plannedPath.empty() && ( plannedPathStep < plannedPath.size() ); }

    void moveToTarget( Eigen::Vector2f position, float rotation );
    void moveStraightToTarget( const Eigen::Vector2f& position, float rotation );

    void rotate( float angle );
    void move( int distance );

    int markVisited( Eigen::Vector2f position );
};

#endif //BOTCONTROLLER_MOVEMENTPLANNER_H
