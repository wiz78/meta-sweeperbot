//
// Created by simone on 3/16/24.
//

#ifndef BOTCONTROLLER_LINEDETECTOR_H
#define BOTCONTROLLER_LINEDETECTOR_H

#include "segment.h"
#include "lidarTypes.h"

#include <vector>

class LineDetector
{
public:
    static Segments findLines( const std::vector<PointData>& pointsToFit );

private:
    static constexpr uint16_t threshold = 10;

    static void fit( const std::vector<Vector2D>& cloud, double& a, double& b, double& c );
};

#endif //BOTCONTROLLER_LINEDETECTOR_H
