//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_FLOORMAP_H
#define BOTCONTROLLER_FLOORMAP_H

#include <memory>
#include <vector>

#include <pointmatcher/PointMatcher.h>

#include "segment.h"
#include "lidarTypes.h"

class OccupancyGrid;

class FloorMap
{
public:
    typedef PointMatcher<float> PM;

    FloorMap();
    ~FloorMap();

    bool setIMURotation( float angle );
    void process( const std::vector<PointData>& points );

    [[nodiscard]] bool hasGrid() const { return grid; }
    [[nodiscard]] const OccupancyGrid& getGridConst() const { return *grid; }
    [[nodiscard]] OccupancyGrid& getGrid() const { return *grid; }

    [[nodiscard]] Eigen::Vector2f getPosition() const { return ( T_to_map_from_new * position ).hnormalized(); }
    [[nodiscard]] float getRotation() const { return atan2( T_to_map_from_new( 1, 0 ), T_to_map_from_new( 0, 0 )); }
    [[nodiscard]] PM::TransformationParameters getTranformationMatrix() const { return T_to_map_from_new; }

private:
    PM::DataPoints							partialMap;
    PM::ICP									icp;
    std::shared_ptr<PM::Transformation> 	rigidTrans;
    std::shared_ptr<PM::DataPointsFilter> 	densityFilter;
    std::shared_ptr<PM::DataPointsFilter> 	maxDensitySubsample;
    std::shared_ptr<PM::DataPointsFilter> 	maxPoints;
    PM::TransformationParameters			T_to_map_from_new;
    Eigen::Vector3f							position;
    OccupancyGrid							*grid = nullptr;

    static float alignToGrid( const Segments& segments );
    static float rotationToAlign( float angle );
};

#endif //BOTCONTROLLER_FLOORMAP_H
