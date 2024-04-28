//
// Created by simone on 3/16/24.
//

#include "occupancyGrid.h"
#include "log.h"

#include <utility>
#include <vector>
#include <queue>
#include <cstring>

typedef std::pair<int, int> intPair;

OccupancyGrid::OccupancyGrid( float resolution, float xSize, float ySize )
{
    scale = 1.0f / resolution;

    this->xSize = std::max( static_cast<int>( ceil( xSize * scale )), 10 );
    this->ySize = std::max( static_cast<int>( ceil( ySize * scale )), 10 );
    extensionWidth = static_cast<int>( xSize / 2 );
    extensionHeight = static_cast<int>( ySize / 2 );

    grid = new OccupancyGridCell[ this->xSize * this->ySize ];

    clear();
    setTransformations();

    Log::info( "OccupancyGrid constructed ( {}, {} x {} )", resolution, xSize, ySize );
}

OccupancyGrid::OccupancyGrid( const OccupancyGrid& copy )
{
    *this = copy;
}

OccupancyGrid::~OccupancyGrid()
{
    delete[] grid;
}

OccupancyGrid& OccupancyGrid::operator=( const OccupancyGrid& other )
{
    if( &other != this ) {

        delete[] grid;

        scale = other.scale;
        xSize = other.xSize;
        ySize = other.ySize;
        extensionWidth = other.extensionWidth;
        extensionHeight = other.extensionHeight;
        worldToMap = other.worldToMap;
        mapToWorld = other.mapToWorld;
        currUpdateTime = other.currUpdateTime;
        currentMarkFreeTime = other.currentMarkFreeTime;
        currentMarkOccupiedTime = other.currentMarkOccupiedTime;

        grid = new OccupancyGridCell[ xSize * ySize ];

        for( int i = ( xSize * ySize ) - 1; i >= 0; i-- )
            grid[ i ] = other.grid[ i ];
    }

    return *this;
}

void OccupancyGrid::setTransformations()
{
    worldToMap = Eigen::Translation2f( static_cast<float>( xSize ) / 2, static_cast<float>( ySize ) / 2 ) * Eigen::AlignedScaling2f( scale, scale );
    mapToWorld = worldToMap.inverse();
}

void OccupancyGrid::clear()
{
    for( int i = ( xSize * ySize ) - 1; i >= 0; i-- )
        grid[ i ].reset();

    currUpdateTime = 0;
    currentMarkOccupiedTime = 0;
    currentMarkFreeTime = 0;
}

void OccupancyGrid::reshape( const PM::Matrix& scan )
{
    float minX = __FLT_MAX__, minY = __FLT_MAX__, maxX = __FLT_MIN__, maxY = __FLT_MIN__;
    Eigen::Vector2f topLeft, bottomRight;
    int top, left, right, bottom;
    int expandLeft = 0, expandRight = 0, expandTop = 0, expandBottom = 0, newXSize, newYSize;
    int modX = extensionWidth, modY = extensionHeight;

    for( long i = scan.cols() - 1; i >= 0; i-- ) {
        float x = scan( 0, i );
        float y = scan( 1, i );

        minX = std::min( x, minX );
        maxX = std::max( x, maxX );
        minY = std::min( y, minY );
        maxY = std::max( y, minY );
    }

    topLeft.x() = minX;
    topLeft.y() = minY;

    bottomRight.x() = maxX;
    bottomRight.y() = maxY;

    topLeft = worldToMap * topLeft;
    bottomRight = worldToMap * bottomRight;

    left = static_cast<int>( floor( topLeft.x() ));
    top = static_cast<int>( floor( topLeft.y() ));
    right = static_cast<int>( ceil( bottomRight.x() ));
    bottom = static_cast<int>( ceil( bottomRight.y() ));

    if( left < 0 )
        expandLeft = modX + ( modX * ( -left / modX ));

    if( right > xSize )
        expandRight = modX + ( modX * (( right - xSize ) / modX ));

    if( top < 0 )
        expandTop = modY + ( modY * ( -top / modY ));

    if( bottom > ySize )
        expandBottom = modY + ( modY * (( bottom - ySize ) / modY ));

    newXSize = xSize + expandLeft + expandRight;
    newYSize = ySize + expandTop + expandBottom;

    if(( newXSize > xSize ) || ( newYSize > ySize )) {
        auto *newGrid = new OccupancyGridCell[ newXSize * newYSize ];

        if( newXSize == xSize )
            memcpy( &newGrid[ expandTop * xSize ], grid, xSize * ySize * sizeof( grid[0] ));
        else {

            for( int i = 0; i < ySize; i++ )
                memcpy( &newGrid[ (( expandTop + i ) * newXSize ) + expandLeft ], &grid[ i * xSize ], sizeof( grid[0] ) * xSize );
        }

        delete[] grid;

        grid = newGrid;
        xSize = newXSize;
        ySize = newYSize;

        worldToMap = Eigen::Translation2f( static_cast<float>( expandLeft ), static_cast<float>( expandTop )) * worldToMap;
        mapToWorld = worldToMap.inverse();

        Log::info( "New grid ( {} x {} ), size: {} bytes", xSize, ySize, sizeof( grid[0] ) * xSize * ySize );
    }
}

void OccupancyGrid::update( const PM::Matrix& scan, const Eigen::Vector2f& position )
{
    Eigen::Vector2f start;
    Eigen::Vector2i startRounded;

    reshape( scan );

    start = worldToMap * position;

    startRounded.x() = static_cast<int>( round( start.x() ));
    startRounded.y() = static_cast<int>( round( start.y() ));

    currentMarkFreeTime = currUpdateTime + 1;
    currentMarkOccupiedTime = currUpdateTime + 2;

    for( int i = 0; i < scan.cols(); i++ ) {
        Eigen::Vector2f point;
        Eigen::Vector2i pointRounded;

        point.x() = scan( 0, i );
        point.y() = scan( 1, i );

        point = worldToMap * point;

        pointRounded.x() = static_cast<int>( round( point.x() ));
        pointRounded.y() = static_cast<int>( round( point.y() ));

        if( startRounded != pointRounded )
            traceRay( startRounded, pointRounded );
    }

    currUpdateTime += 3;
}

Eigen::Vector2i OccupancyGrid::getMapPosition( Eigen::Vector2f position ) const
{
    Eigen::Vector2i	ret;

    position = worldToMap * position;

    ret.x() = static_cast<int>( round( position.x() ));
    ret.y() = static_cast<int>( round( position.y() ));

    return ret;
}

void OccupancyGrid::traceRay( Eigen::Vector2i start, Eigen::Vector2i end )
{
    int x0 = start.x();
    int y0 = start.y();
    int x1 = end.x();
    int y1 = end.y();

    if( withinBounds( x0, y0 ) && withinBounds( x1, y1 )) {
        int dx = x1 - x0;
        int dy = y1 - y0;
        unsigned int abs_dx = abs( dx );
        unsigned int abs_dy = abs( dy );
        int offset_dx = ( dx > 0 ) ? 1 : -1;
        int offset_dy = ( dy > 0 ) ? xSize : -xSize;
        unsigned int startOffset = ( start.y() * xSize ) + start.x();

        if( abs_dx >= abs_dy ) {
            int error_y = static_cast<int>( abs_dx / 2 );

            bresenham2D( abs_dx, abs_dy, error_y, offset_dx, offset_dy, startOffset );

        } else {
            int error_x = static_cast<int>( abs_dy / 2 );

            bresenham2D( abs_dy, abs_dx, error_x, offset_dy, offset_dx, startOffset );
        }

        markOccupied(( end.y() * xSize ) + end.x() );
    }
}

void OccupancyGrid::bresenham2D( unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset )
{
    unsigned int end = abs_da - 1;

    markFree( offset );

    for( unsigned int i = 0; i < end; ++i ) {

        offset += offset_a;
        error_b += static_cast<int>( abs_db );

        if( error_b >= abs_da ) {
            offset += offset_b;
            error_b -= static_cast<int>( abs_da );
        }

        markFree( offset );
    }
}

void OccupancyGrid::markFree( unsigned int offset )
{
    OccupancyGridCell& cell = grid[ offset ];

    if( cell.getUpdateTime() < currentMarkFreeTime ) {
        cell.setUpdateTime( currentMarkFreeTime );
        cell.markFree();
    }
}

void OccupancyGrid::markOccupied( unsigned int offset )
{
    OccupancyGridCell& cell = grid[ offset ];

    if( cell.getUpdateTime() < currentMarkOccupiedTime ) {

        // if this cell has been updated as free in the current iteration, revert this
        if( cell.getUpdateTime() == currentMarkFreeTime )
            cell.undoMarkFree();

        cell.markOccupied();
        cell.setUpdateTime( currentMarkOccupiedTime );
    }
}

// Using Dijkstra's algorithm
std::vector<Eigen::Vector2i> OccupancyGrid::findClosestUnvisitedCell( const Eigen::Vector2i& mapPos ) const
{
    const int adjacentIndexes[] = { -xSize, -1, 1, xSize };
    const ssize_t numCells = xSize * ySize;
    std::priority_queue<intPair, std::vector<intPair>, std::greater<>> pq;
    std::vector<int> dist( numCells );
    int prev[ numCells ];
    size_t startIdx = getCellIndex( mapPos );
    ssize_t minDistIdx = -1;
    std::vector<Eigen::Vector2i> ret;

    for( ssize_t i = numCells - 1; i >= 0; i-- ) {
        dist[ i ] = INT_MAX;
        prev[ i ] = INT_MAX;
    }

    pq.emplace( 0, startIdx );
    dist[ startIdx ] = 0;

    while( !pq.empty() ) {
        int u = pq.top().second;

        pq.pop();

        for( int adjacentIndex : adjacentIndexes ) {
            const int v = u + adjacentIndex;

            if(( v >= 0 ) && ( v < numCells )) {
                const OccupancyGridCell& cell = getCellConst( v );

                if( cell.isFree() && !cell.isDangerZone() && ( dist[ u ] < dist[ v ] - 1 )) {

                    dist[ v ] = dist[ u ] + 1;
                    prev[ v ] = u;

                    pq.emplace( dist[ v ], v );
                }
            }
        }
    }

    for( ssize_t i = static_cast<ssize_t>( dist.size() ) - 1; i >= 0; i-- )
        if(( dist[ i ] > 0 ) && !getCellConst( static_cast<int>( i )).isVisited() && (( minDistIdx < 0 ) || ( dist[ i ] < dist[ minDistIdx ] )))
            minDistIdx = i;

    if(( minDistIdx >= 0 ) && ( dist[ minDistIdx ] < INT_MAX ))
        while( dist[ minDistIdx ] > 0 ) {

            ret.emplace( ret.begin(), minDistIdx % xSize, minDistIdx / xSize );

            minDistIdx = prev[ minDistIdx ];
        }

    return ret;
}

BinaryStream<std::ostringstream>& operator<<( BinaryStream<std::ostringstream>& stream, const OccupancyGrid& grid )
{
    stream << grid.scale;
    stream << grid.xSize;
    stream << grid.ySize;
    stream << grid.extensionWidth;
    stream << grid.extensionHeight;

    for( int i = 0; i < grid.worldToMap.rows(); i++ )
        for( int j = 0; j < grid.worldToMap.cols(); j++ )
            stream << grid.worldToMap( i, j );

    for( int i = 0; i < grid.mapToWorld.rows(); i++ )
        for( int j = 0; j < grid.mapToWorld.cols(); j++ )
            stream << grid.mapToWorld( i, j );

    for( int i = ( grid.xSize * grid.ySize ) - 1; i >= 0; i-- )
        stream << grid.grid[ i ];

    stream << grid.currUpdateTime;
    stream << grid.currentMarkOccupiedTime;
    stream << grid.currentMarkFreeTime;

    return stream;
}

BinaryStream<std::istringstream>& operator>>( BinaryStream<std::istringstream>& stream, OccupancyGrid& grid )
{
    stream >> grid.scale;
    stream >> grid.xSize;
    stream >> grid.ySize;
    stream >> grid.extensionWidth;
    stream >> grid.extensionHeight;

    for( int i = 0; i < grid.worldToMap.rows(); i++ )
        for( int j = 0; j < grid.worldToMap.cols(); j++ )
            stream >> grid.worldToMap( i, j );

    for( int i = 0; i < grid.mapToWorld.rows(); i++ )
        for( int j = 0; j < grid.mapToWorld.cols(); j++ )
            stream >> grid.mapToWorld( i, j );

    delete[] grid.grid;

    grid.grid = new OccupancyGridCell[ grid.xSize * grid.ySize ];

    for( int i = ( grid.xSize * grid.ySize ) - 1; i >= 0; i-- )
        stream >> grid.grid[ i ];

    stream >> grid.currUpdateTime;
    stream >> grid.currentMarkOccupiedTime;
    stream >> grid.currentMarkFreeTime;

    return stream;
}
