//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_OCCUPANCYGRID_H
#define BOTCONTROLLER_OCCUPANCYGRID_H

#include <Eigen/Geometry>
#include <pointmatcher/PointMatcher.h>

#include <sstream>

#include "binaryStream.h"
#include "occupancyGridCell.h"

class OccupancyGrid
{
public:
    typedef PointMatcher<float> PM;

    OccupancyGrid( float resolution, float xSize, float ySize );
    OccupancyGrid( const OccupancyGrid& copy );
    ~OccupancyGrid();

    OccupancyGrid& operator=( const OccupancyGrid& other );

    void clear();

    void update( const PM::Matrix& scan, const Eigen::Vector2f& position );

    [[nodiscard]] float getResolution() const { return 1.0F / scale; }
    [[nodiscard]] int getXSize() const { return xSize; }
    [[nodiscard]] int getYSize() const { return ySize; }

    [[nodiscard]] Eigen::Vector2i getMapPosition( Eigen::Vector2f position ) const;
    [[nodiscard]] Eigen::Vector2f getWorldPosition( Eigen::Vector2i position ) const { return mapToWorld * position.cast<float>(); }

    [[nodiscard]] bool isValidPos( const Eigen::Vector2i& mapPos ) const { return ( mapPos.x() >= 0 ) && ( mapPos.x() < xSize ) && ( mapPos.y() >= 0 ) && ( mapPos.y() < ySize ); }

    [[nodiscard]] const OccupancyGridCell& getCellConst( int x, int y ) const { return getCellConst( getCellIndex( x, y )); }
    [[nodiscard]] const OccupancyGridCell& getCellConst( int idx ) const { return grid[ idx ]; }
    [[nodiscard]] const OccupancyGridCell& getCellConst( const Eigen::Vector2i& mapPos ) const { return getCellConst( getCellIndex( mapPos )); }

    [[nodiscard]] int getCellIndex( const Eigen::Vector2i& mapPos ) const { return getCellIndex( mapPos.x(), mapPos.y() ); }
    [[nodiscard]] constexpr int getCellIndex( int x, int y ) const { return x + ( y * xSize ); }

    [[nodiscard]] OccupancyGridCell& getCell( const Eigen::Vector2i& mapPos ) const { return getCell( getCellIndex( mapPos )); }
    [[nodiscard]] OccupancyGridCell& getCell( int x, int y ) const { return getCell( getCellIndex( x, y )); }
    [[nodiscard]] OccupancyGridCell& getCell( int idx ) const { return grid[ idx ]; }

    [[nodiscard]] std::vector<Eigen::Vector2i> findClosestUnvisitedCell( const Eigen::Vector2i& mapPos ) const;

    friend BinaryStream<std::ostringstream>& operator<<( BinaryStream<std::ostringstream>& stream, const OccupancyGrid& grid );
    friend BinaryStream<std::istringstream>& operator>>( BinaryStream<std::istringstream>& stream, OccupancyGrid& grid );

private:
    float scale;
    int xSize;
    int ySize;
    int extensionWidth;
    int extensionHeight;
    Eigen::Affine2f worldToMap;
    Eigen::Affine2f mapToWorld;
    OccupancyGridCell *grid = nullptr;
    unsigned int currUpdateTime{};
    unsigned int currentMarkOccupiedTime{};
    unsigned int currentMarkFreeTime{};

    void setTransformations();
    void reshape( const PM::Matrix& scan );

    void traceRay( Eigen::Vector2i start, Eigen::Vector2i end );
    void bresenham2D( unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset );
    void markFree( unsigned int offset );
    void markOccupied( unsigned int offset );

    [[nodiscard]] bool withinBounds( int x, int y ) const { return ( x >= 0 ) && ( x < xSize ) && ( y >= 0 ) && ( y < ySize ); }
};

#endif //BOTCONTROLLER_OCCUPANCYGRID_H
