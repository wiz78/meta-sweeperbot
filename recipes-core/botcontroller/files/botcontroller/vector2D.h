//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_VECTOR2D_H
#define BOTCONTROLLER_VECTOR2D_H

#include <cmath>

class Vector2D
{
public:
    Vector2D( double x, double y ) : _x{ x }, _y{ y }
    {
    }

    double x() const { return _x; }
    double y() const { return _y; }

    Vector2D operator+( const Vector2D& other ) const { return Vector2D{ _x + other._x, _y + other._y }; }
    Vector2D operator-( const Vector2D& other ) const { return Vector2D{ _x - other._x, _y - other._y }; }
    Vector2D operator*( double scalar ) const { return Vector2D{ _x * scalar, _y * scalar }; }
    Vector2D operator/( double scalar ) const { return Vector2D{ _x / scalar, _y / scalar }; }
    Vector2D& operator/=( double scalar ) { _x /= scalar; _y /= scalar; return *this; }

    double magnitude() const { return hypot( _x, _y ); }
    double sqrMagnitude() const { return ( _x * _x ) + ( _y * _y ); }

    void normalize() { *this /= magnitude(); }

private:
    double _x;
    double _y;
};

#endif //BOTCONTROLLER_VECTOR2D_H
