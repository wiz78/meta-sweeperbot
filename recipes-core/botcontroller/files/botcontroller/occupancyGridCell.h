//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_OCCUPANCYGRIDCELL_H
#define BOTCONTROLLER_OCCUPANCYGRIDCELL_H

#include <cmath>
#include <sstream>

#include "binaryStream.h"

class OccupancyGridCell
{
public:
    void reset();

    void markFree() { if( !isFree() ) firstFreeTime = updateTime; updateValue( freeFactor ); }
    void undoMarkFree() { updateValue( -freeFactor ); }
    void markOccupied() { if( value < 50 ) updateValue( occupiedFactor ); }
    void forceOccupied() { value = 30; }

    void markVisited() { flags |= CellFlagVisited; }
    void markDangerZone() { flags |= CellFlagDangerZone; }
    void clearDangerZone() { flags &= ~CellFlagDangerZone; }

    void markTarget() { flags |= CellFlagTarget; }
    void clearTarget() { flags &= ~CellFlagTarget; }

    [[nodiscard]] unsigned int getUpdateTime() const { return updateTime; }
    void setUpdateTime( unsigned int t ) { updateTime = t; }

    [[nodiscard]] unsigned int getFirstFreeTime() const { return firstFreeTime; }

    [[nodiscard]] float getProbability() const { float odds = std::exp( value ); return odds / ( odds + 1.0f ); }

    [[nodiscard]] bool isOccupied() const { return value > 0; }
    [[nodiscard]] bool isFree() const { return value < 0; }
    [[nodiscard]] bool isVisited() const { return flags & CellFlagVisited; }
    [[nodiscard]] bool isDangerZone() const { return flags & CellFlagDangerZone; }

    friend BinaryStream<std::ostringstream>& operator<<( BinaryStream<std::ostringstream>& stream, const OccupancyGridCell& cell );
    friend BinaryStream<std::istringstream>& operator>>( BinaryStream<std::istringstream>& stream, OccupancyGridCell& cell );

private:
    constexpr static float probToLogOdds( float prob ) { return std::log( prob / ( 1.0f - prob )); }
    static const float freeFactor;
    static const float occupiedFactor;

    enum Flags {
        CellFlagVisited = 1 << 0,
        CellFlagDangerZone = 1 << 1,
        CellFlagTarget = 1 << 2,
    };

    float value = 0;
    unsigned int updateTime = 0;
    unsigned int firstFreeTime = 0;
    unsigned char flags = 0;

    void updateValue( float offset ) { value += offset; }
};

#endif //BOTCONTROLLER_OCCUPANCYGRIDCELL_H
