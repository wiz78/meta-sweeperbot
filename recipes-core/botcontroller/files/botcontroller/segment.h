//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_SEGMENT_H
#define BOTCONTROLLER_SEGMENT_H

#include <vector>

#include "vector2D.h"
#include "lidarTypes.h"

class Segment
{
public:
    Segment( const PointData& a, const PointData& b ) : _a{ a.x, a.y }, _b{ b.x, b.y }
    {
    }

    Segment( const Vector2D& a, const Vector2D& b ) : _a{ a }, _b{ b }
    {
    }

    const Vector2D& a() const { return _a; }
    const Vector2D& b() const { return _b; }

    void setA( const Vector2D&x ) { _a = x; }
    void setB( const Vector2D&x ) { _b = x; }

    double length() const { return ( _a - _b ).magnitude(); }
    float angle() const { return atan2( _a.x() - _b.x(), _a.y() - _b.y() ); }
    // 0-4 = 0°-360°
    float diamondAngle() const;
    float diamondAngle2() const { float a = diamondAngle(); if( a > 2 ) a -= 2; return a; }

    Vector2D midPoint() const { return Vector2D(( _a.x() + _b.x() ) / 2, ( _a.y() + _b.y() ) / 2 ); }

private:
    Vector2D _a;
    Vector2D _b;
};

typedef std::vector<Segment> Segments;

#endif //BOTCONTROLLER_SEGMENT_H
