//
// Created by simone on 3/16/24.
//

#include "floorMap.h"
#include "lineDetector.h"
#include "occupancyGrid.h"
#include "log.h"

using namespace PointMatcherSupport;

FloorMap::FloorMap()
{
    PointMatcherSupport::Parametrizable::Parameters params;

    partialMap.featureLabels.emplace_back( PM::DataPoints::Label( "x", 1 ));
    partialMap.featureLabels.emplace_back( PM::DataPoints::Label( "y", 1 ));
    partialMap.featureLabels.emplace_back( PM::DataPoints::Label( "pad", 1 ));

    partialMap.features.conservativeResize( 3, Eigen::NoChange );

    partialMap.featureLabels.emplace_back( PM::DataPoints::Label( "x", 1 ));
    partialMap.featureLabels.emplace_back( PM::DataPoints::Label( "y", 1 ));
    partialMap.featureLabels.emplace_back( PM::DataPoints::Label( "pad", 1 ));

    partialMap.features.conservativeResize( 3, Eigen::NoChange );

    T_to_map_from_new = PM::TransformationParameters::Identity( 3, 3 );
    rigidTrans = PM::get().REG( Transformation ).create( "RigidTransformation" );

    densityFilter =	PM::get().REG( DataPointsFilter ).create( "SurfaceNormalDataPointsFilter", {
        { "knn", toParam( 10 ) },
        { "epsilon", toParam( 1 ) },
        { "keepNormals", toParam( 0 ) },
        { "keepDensities", toParam( 1 ) }
    } );

    maxDensitySubsample = PM::get().REG( DataPointsFilter ).create( "MaxDensityDataPointsFilter", { { "maxDensity", toParam( 3000 ) } } );
    maxPoints = PM::get().REG( DataPointsFilter ).create( "MaxPointCountDataPointsFilter", { { "maxCount", toParam( 1000 ) } } );

    icp.matcher = PM::get().REG( Matcher ).create( "KDTreeMatcher", {
        { "knn", "1" },
        { "epsilon", "0.005" },
    } );

    //icp.errorMinimizer = PM::get().REG( ErrorMinimizer ).create( "PointToPointErrorMinimizer" );
    icp.errorMinimizer = PM::get().REG( ErrorMinimizer ).create( "PointToPlaneErrorMinimizer" );

    icp.transformations.push_back( rigidTrans );

    //icp.readingDataPointsFilters.push_back( randSubsample );
    icp.readingDataPointsFilters.push_back( PM::get().REG( DataPointsFilter ).create( "VoxelGridDataPointsFilter", {
        { "vSizeX", toParam( 0.05 ) },
        { "vSizeY", toParam( 0.05 ) }
    } ));

    icp.referenceDataPointsFilters.push_back( PM::get().REG( DataPointsFilter ).create( "SamplingSurfaceNormalDataPointsFilter" ));
    icp.outlierFilters.push_back( PM::get().REG( OutlierFilter ).create( "TrimmedDistOutlierFilter" ));
    //icp.outlierFilters.push_back( PM::get().REG( OutlierFilter ).create( "VarTrimmedDistOutlierFilter" ));
    icp.transformationCheckers.push_back( PM::get().REG( TransformationChecker ).create( "CounterTransformationChecker", { { "maxIterationCount", toParam( 80 ) } } ));
    icp.transformationCheckers.push_back( PM::get().REG( TransformationChecker ).create( "DifferentialTransformationChecker" ));

    icp.inspector = PM::get().REG( Inspector ).create( "NullInspector" );

    position( 0 ) = 0;
    position( 1 ) = 0;
    position( 2 ) = 1;
}

FloorMap::~FloorMap()
{
    delete grid;
}

bool FloorMap::setIMURotation( float angle )
{
    bool set = fabs( angle - getRotation() ) > M_PI * ( 1.5 / 180 );

    if( set ) {

        Log::debug( "setIMURotation: {} - from map: {}", angle, getRotation() );

        T_to_map_from_new( 0, 0 ) = T_to_map_from_new( 1, 1 ) = cos( angle );
        T_to_map_from_new( 1, 0 ) = sin( angle );
        T_to_map_from_new( 0, 1 ) = -T_to_map_from_new( 1, 0 );
    }

    return true;
//	return set;
}

void FloorMap::process( const std::vector<PointData>& points )
{
    constexpr double MAX_DIST = 4500;
    size_t newPointsNum = 0;

    for( const auto& pt : points )
        if( pt.distance < MAX_DIST )
            newPointsNum++;

    if( newPointsNum < 400 )
        return;

    PM::Matrix newCloud( 3, newPointsNum );

    newPointsNum = 0;

    for( const auto& pt : points )
        if( pt.distance < MAX_DIST ) {
            newCloud( 0, newPointsNum ) = pt.x / 1000.;
            newCloud( 1, newPointsNum ) = pt.y / 1000.;
            newCloud( 2, newPointsNum++ ) = 1;
        }

    if( newPointsNum <= 0 ) {
        Log::debug( "FloorMap::process - no new points" );
        return;
    }

    PM::DataPoints data( newCloud, partialMap.featureLabels );

    if( partialMap.getNbPoints() ) {

        try {
            Eigen::Vector2f oldPos = getPosition();

            T_to_map_from_new = icp( data, partialMap, T_to_map_from_new );

            if(( oldPos - getPosition() ).norm() >= 0.03 )
                grid->update( rigidTrans->compute( data, T_to_map_from_new ).features, getPosition() );

            data = icp.getReadingFiltered();
            data = rigidTrans->compute( data, T_to_map_from_new );

            partialMap.concatenate( data );

            if( partialMap.features.cols() > 1000 )
                partialMap = maxPoints->filter( partialMap );
        }
        catch( const PM::ConvergenceError& error ) {
            Log::error( "FloorMap::process - ConvergenceError {}", error.what() );
        }
        catch( const std::runtime_error& error ) {
            Log::error( "FloorMap::process - runtime_error {}", error.what() );
            partialMap.features.conservativeResize( Eigen::NoChange, 0 );
        }

    } else if( data.getNbPoints() ) {
        Segments lines = LineDetector::findLines( points );
        float angle = alignToGrid( lines );
        float minX = 0, maxX = 0, minY = 0, maxY = 0;
        constexpr float gridResolution = 0.05;

        T_to_map_from_new( 0, 0 ) = T_to_map_from_new( 1, 1 ) = cos( angle );
        T_to_map_from_new( 1, 0 ) = sin( angle );
        T_to_map_from_new( 0, 1 ) = -T_to_map_from_new( 1, 0 );

        data = rigidTrans->compute( data, T_to_map_from_new );

        for( int i = data.getNbPoints() - 1; i >= 0; i-- ) {
            float x = data.features( 0, i );
            float y = data.features( 1, i );

            minX = std::min( minX, x );
            maxX = std::max( maxX, x );
            minY = std::min( minY, y );
            maxY = std::max( maxY, y );
        }

        delete grid;

        grid = new OccupancyGrid( gridResolution, maxX - minX, maxY - minY );

        grid->update( data.features, getPosition() );

        icp.readingDataPointsFilters.apply( data );

        partialMap.features.conservativeResize( Eigen::NoChange, data.getNbPoints() + partialMap.getNbPoints() );

        partialMap.features.rightCols( data.getNbPoints() ) = data.features;
    }
}

float FloorMap::alignToGrid( const Segments& segments )
{
    float bestAngle = 0;
    int bestMatches = 0;

    for( const auto& segment : segments ) {
        float angle = rotationToAlign( segment.angle() );
        float anglesSum = 0;
        int matches = 0;

        for( const auto& segment2 : segments ) {
            float angle2 = rotationToAlign( segment2.angle() );

            if( fabs( angle - angle2 ) <= 0.025 ) {
                matches++;
                anglesSum += angle2;
            }
        }

        if( matches > bestMatches ) {
            bestMatches = matches;
            bestAngle = anglesSum / matches;
        }
    }

    return -bestAngle;
}

float FloorMap::rotationToAlign( float angle )
{
    if( angle < 0 )
        angle = -rotationToAlign( -angle );
    else if( angle <= M_PI_4 )
        angle = -angle;
    else if( angle <= M_PI_2 + M_PI_4 )
        angle = M_PI_2 - angle;
    else
        angle = M_PI - angle;

    return angle;
}
