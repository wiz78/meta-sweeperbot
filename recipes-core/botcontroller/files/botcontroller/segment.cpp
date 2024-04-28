//
// Created by simone on 3/16/24.
//

#include "segment.h"

float Segment::diamondAngle() const
{
    double dx = _a.x() - _b.x();
    double dy = _a.y() - _b.y();

    if( dy >= 0 )
        return ( dx >= 0 ) ? ( dy / ( dx + dy )) : ( 1 - ( dx / ( dy - dx )));
    else
        return ( dx < 0 ) ? ( 2 - ( dy / ( -dx - dy ))) : ( 3 + ( dx / ( dx - dy )));
}
