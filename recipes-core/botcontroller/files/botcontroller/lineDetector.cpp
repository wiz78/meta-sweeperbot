//
// Created by simone on 3/16/24.
//

#include "lineDetector.h"
#include "segment.h"

#include <cmath>
#include <deque>

Segments LineDetector::findLines( const std::vector<PointData>& pointsToFit )
{
    Segments ret;
    std::deque<Segment> segments;
    std::vector<std::vector<Vector2D>> mergedPoints;

    for( int i = 0; i < pointsToFit.size() - 1; i++ )
        segments.emplace_back( pointsToFit[ i ], pointsToFit[ i + 1 ] );

    while( !segments.empty() ) {
        const Segment& a = segments[0];
        float angleA = a.diamondAngle2();

        segments.pop_front();

        if( a.length() < 200 ) {

            mergedPoints.emplace_back( std::vector<Vector2D>{ a.a(), a.b() } );

            while( !segments.empty() ) {
                std::vector<Vector2D>& currentSet = mergedPoints.back();
                const Segment& b = segments.front();
                float angleB = b.diamondAngle2();

                if( std::fabs( angleB - angleA ) > .2 )
                    break;

                if(( b.b() - currentSet.back() ).magnitude() > 100 )
                    break;

                currentSet.emplace_back( b.a() );
                currentSet.emplace_back( b.b() );

                segments.pop_front();
            }
        }
    }

    for( const auto& v : mergedPoints )
        if( v.size() > 6 ) {
            const Vector2D& start = v.front();
            const Vector2D& end = v.back();

            if(( start - end ).magnitude() > 150 ) {
                double a, b, c;

                fit( v, a, b, c );

                if( std::abs( start.x() - end.x() ) > std::abs( start.y() - end.y() )) {
                    Vector2D v1{ start.x(), (( -a / b ) * start.x() ) - ( c / b ) };
                    Vector2D v2{ end.x(), (( -a / b ) * end.x() ) - ( c / b ) };

                    ret.emplace_back( v1, v2 );

                } else {
                    Vector2D v1{ (( -b / a ) * start.y() ) - ( c / a ), start.y() };
                    Vector2D v2{ (( -b / a ) * end.y() ) - ( c / a ), end.y() };

                    ret.emplace_back( v1, v2 );
                }
            }
        }

    return ret;
}

// line equation is Ax + By + C = 0
// → y = ( -A / B ) * x - ( C / B )
// → x = ( -B / A ) * y - ( C / A )
void LineDetector::fit( const std::vector<Vector2D>& cloud, double& a, double& b, double& c )
{
    double x = 0, y = 0, xy = 0, x2 = 0, y2 = 0;
    auto cloudSize = static_cast<double>( cloud.size() );

    for( auto const& point : cloud ) {
        x  += point.x();
        y  += point.y();
        xy += point.x() * point.y();
        x2 += point.x() * point.x();
        y2 += point.y() * point.y();
    }

    x  /= cloudSize;
    y  /= cloudSize;
    xy /= cloudSize;
    x2 /= cloudSize;
    y2 /= cloudSize;

    a = ( x * y ) - xy;

    double bx = x2 - ( x * x );
    double by = y2 - ( y * y );

    if( std::abs( bx ) < std::abs( by )) { // Line is more Vertical.
        b = by;
        std::swap( a, b );
    } else // Line is more Horizontal.
        b = bx;

    c = -( a * x ) - ( b * y );
}
