//
// Created by simone on 3/16/24.
//

#include "occupancyGridCell.h"

const float OccupancyGridCell::freeFactor = OccupancyGridCell::probToLogOdds( 0.4 );
const float OccupancyGridCell::occupiedFactor = OccupancyGridCell::probToLogOdds( 0.85 );

void OccupancyGridCell::reset()
{
    value = 0;
    updateTime = 0;
}

BinaryStream<std::ostringstream>& operator<<( BinaryStream<std::ostringstream>& stream, const OccupancyGridCell& cell )
{
    stream << cell.value;
    stream << cell.updateTime;
    stream << cell.flags;

    return stream;
}

BinaryStream<std::istringstream>& operator>>( BinaryStream<std::istringstream>& stream, OccupancyGridCell& cell )
{
    stream >> cell.value;
    stream >> cell.updateTime;
    stream >> cell.flags;

    return stream;
}
