#include "lidar.h"
#include "esp32Comm.h"
#include "log.h"
#include "mainController.h"

#include <algorithm>
#include <cstring>

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

Lidar::Lidar( MainController& mainController ) : mainController{ mainController }
{
    minDistancesPacket.cmd = Esp32Comm::CMD_MIN_DISTANCES;
}

void Lidar::flush()
{
    tmp_lidar_scan_data_front = 0;
    tmp_lidar_scan_data_back = 0;

    resetDistances();
}

bool Lidar::processFrame( const LiDARFrame *frame, size_t size )
{
    bool ret = false;

    if(( frame->measure.ver_len == LIDAR_DATA_PKG_INFO ) && ( size == sizeof( LiDARMeasureDataType )) &&
       ( calCRC8( reinterpret_cast<const uint8_t *>( frame ), size - 1 ) == frame->measure.crc8 ))
        ret = storePacket( frame->measure );
    else
        Log::warning( "Invalid lidar frame: {:02x}, {}, wrong {}",
                      frame->measure.ver_len, size, ( size == sizeof( LiDARMeasureDataType )) ? "CRC" : "size" );

    return ret;
}

uint8_t Lidar::calCRC8( const uint8_t *p, uint8_t len )
{
    uint8_t crc = 0;

    while( len-- )
        crc = CrcTable[ ( crc ^ *p++ ) & 0xff ];

    return crc;
}

bool Lidar::storePacket( const LiDARMeasureDataType& dataPkg )
{
    uint32_t diff = ( static_cast<uint32_t>( dataPkg.end_angle ) + 36000 - static_cast<uint32_t>( dataPkg.start_angle )) % 36000;

    if( dataPkg.speed <= 0 ) {

        tmp_lidar_scan_data_back = tmp_lidar_scan_data_front;
        return false;

    } else if( diff <= (( dataPkg.speed + ( dataPkg.speed / 2 )) * LIDAR_POINT_PER_PACK * 100 / lidar_measure_freq_ )) {
        float step = static_cast<float>( diff ) / LIDAR_POINT_PER_PACK / 100.0f;
        float angle = static_cast<float>( dataPkg.start_angle ) / 100.0f;

        for( int i = 0; i < LIDAR_POINT_PER_PACK; angle += step, i++ )
            if( dataPkg.point[i].distance ) {
                constexpr float aperture = 60.0f / 2;
                int side;

                if( angle >= 360.0 )
                    angle -= 360.0;

                tmp_lidar_scan_data[ tmp_lidar_scan_data_back ].angle = angle;
                tmp_lidar_scan_data[ tmp_lidar_scan_data_back ].distance = dataPkg.point[i].distance;
                tmp_lidar_scan_data[ tmp_lidar_scan_data_back ].intensity = dataPkg.point[i].intensity;

                tmp_lidar_scan_data[ tmp_lidar_scan_data_back ].computeXY();

                tmp_lidar_scan_data_back = ++tmp_lidar_scan_data_back % tmp_lidar_scan_data_len;

                if(( angle < aperture ) || ( angle > 360 - aperture ))
                    side = 0;
                else if(( angle > 90 - aperture ) && ( angle < 90 + aperture ))
                    side = 1;
                else if(( angle > 180 - aperture ) && ( angle < 180 + aperture ))
                    side = 2;
                else if(( angle > 270 - aperture ) && ( angle < 270 + aperture ))
                    side = 3;
                else
                    side = -1;

                if(( side >= 0 ) && ( minDistancesPacket.minDistances[ side ] > dataPkg.point[i].distance )) {
                    minDistancesPacket.minDistances[ side ] = dataPkg.point[i].distance;
                    minDistances[ side ].store( minDistancesPacket.minDistances[ side ], std::memory_order_relaxed );
                }
            }
    }

    return assembleScan( dataPkg.speed );
}

bool Lidar::assembleScan( double speed )
{
    float last_angle = 0;
    int count = 0;

    for( size_t front = tmp_lidar_scan_data_front; front != tmp_lidar_scan_data_back; front = ++front % tmp_lidar_scan_data_len ) {
        double packetsTime = ++count * ( speed / 360. );
        const PointData& n = tmp_lidar_scan_data[ front ];

        // wait for enough data to show a circle
        if(( n.angle < 20.0 ) && ( last_angle > 340.0 )) {

            if( packetsTime > ( lidar_measure_freq_ * 1.4 )) {
                tmp_lidar_scan_data_front = ( tmp_lidar_scan_data_front + count ) % tmp_lidar_scan_data_len;
                return false;
            }

            if( count > 500 ) {

                fullScanPoints = 0;

				while( count-- > 0 ) {

                    fullScan[ fullScanPoints++ ] = tmp_lidar_scan_data[ tmp_lidar_scan_data_front++ ];

					tmp_lidar_scan_data_front %= tmp_lidar_scan_data_len;
				}

				std::sort( fullScan, &fullScan[ tmp_lidar_scan_data_len ],
                           [](const PointData& a, const PointData& b) { return a.angle < b.angle; } );

                tmp_lidar_scan_data_front = ( tmp_lidar_scan_data_front + count ) % tmp_lidar_scan_data_len;

                mainController.getEsp32Comm().writePacket( &minDistancesPacket, sizeof( minDistancesPacket ));
                resetDistances();

                return true;
            }
        }

        if( packetsTime > ( lidar_measure_freq_ * 2 )) {
            tmp_lidar_scan_data_front = ( tmp_lidar_scan_data_front + count ) % tmp_lidar_scan_data_len;
            return false;
        }

        last_angle = n.angle;
    }

    return false;
}

void Lidar::resetDistances()
{
    for( int i = 0; i < sizeof( minDistancesPacket.minDistances ) / sizeof( minDistancesPacket.minDistances[0] ); i++ ) {
        minDistancesPacket.minDistances[ i ] = 32000;
        minDistances[ i ].store( 32000, std::memory_order_relaxed );
    }
}
