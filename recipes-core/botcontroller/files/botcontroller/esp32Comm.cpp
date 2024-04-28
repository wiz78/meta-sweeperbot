//
// Created by simone on 2/25/24.
//

#include "esp32Comm.h"
#include "mainController.h"
#include "telemetry.h"
#include "lidar.h"
#include "log.h"

#include <iostream>

Esp32Comm::Esp32Comm( MainController& mainController ) : mainController{ mainController }, serial( "/dev/ttyAMA0", 460800 )
{
}

void Esp32Comm::run()
{
    if( serial.open() ) {

        Log::info( "Serial port open" );

        serial.put( CMD_READY );

        for(;;) {
            auto len = static_cast<ssize_t>( serial.readInt() );

            if(( len > 0 ) && ( len < 1024 * 1024 )) {
                std::vector<char> data = serial.read( len );
                auto type = static_cast<Telemetry::PacketType>( data[0] );

                switch( type ) {

                    case Telemetry::PacketType::LIDARFrame:
                        mainController.processLidarFrame( reinterpret_cast<LiDARFrame *>( data.data() ), data.size() );
                        break;

                    case Telemetry::PacketType::LIDARFlush:
                        mainController.getLidar().flush();
                        break;

                    case Telemetry::PacketType::Stopped:
                        mainController.updatePlannerWithReason( MovementPlanner::UpdateReason::Stopped );
                        break;

                    case Telemetry::PacketType::StoppedByObstacle:
                        mainController.updatePlannerWithReason( MovementPlanner::UpdateReason::StoppedByObstacle );
                        break;

                    case Telemetry::PacketType::IMU:
                        mainController.setIMUYaw( reinterpret_cast<IMUPacket *>( data.data() )->euler[0] );
                        break;

                    case Telemetry::PacketType::LOG: {
                        std::string logLine( &data[1], data.data() + len );

                        Log::info( "ESP32: {}", logLine );
                    }   break;

                    case Telemetry::PacketType::ESPFirmwareGotPacket:
                    case Telemetry::PacketType::ESPFirmwareStop: {
                        std::unique_lock<std::mutex> fl( firmwareMutex );

                        firmwarePacketType.store( type, std::memory_order_relaxed );
                        firmwareCV.notify_one();
                    }   break;

                    case Telemetry::PacketType::ESPReady:
                        serial.put( CMD_READY );
                        break;

                    default:
                        break;
                }

                switch( type ) {

                    case Telemetry::PacketType::IMU:
                    case Telemetry::PacketType::LOG:
                    case Telemetry::PacketType::Rotation:
                    case Telemetry::PacketType::Stopped:
                    case Telemetry::PacketType::StoppedByObstacle:
                        mainController.getTelemetry().pushPacket( data );
                        break;

                    default:
                        break;
                }
            }
        }

    } else
        Log::error( "Couldn't open serial port! ({})", errno );
}

void Esp32Comm::move( int32_t distance )
{
    char packet[] = {
        CMD_MOVE,
        static_cast<char>( distance & 0xFF ),
        static_cast<char>(( distance >> 8 ) & 0xFF ),
        static_cast<char>(( distance >> 16 ) & 0xFF ),
        static_cast<char>(( distance >> 24 ) & 0xFF ),
    };

    writePacket( packet, sizeof( packet ));
}

void Esp32Comm::rotate( int direction, float angle )
{
    auto *anglePtr = reinterpret_cast<unsigned int *>( &angle );
    char packet[] = {
        CMD_ROTATE,
        static_cast<char>( direction ),
        static_cast<char>( *anglePtr & 0xFF ),
        static_cast<char>(( *anglePtr >> 8 ) & 0xFF ),
        static_cast<char>(( *anglePtr >> 16 ) & 0xFF ),
        static_cast<char>(( *anglePtr >> 24 ) & 0xFF ),
    };

    writePacket( packet, sizeof( packet ));
}

void Esp32Comm::updateFirmware( const std::string& packet )
{
    std::lock_guard<std::recursive_mutex> lock( serial.getMutex() );
    size_t len = packet.size();
    const char *ptr = packet.data();
    bool eof = false;
    int i = 0;
    char header[] = {
        CMD_UPDATE,
        static_cast<char>( len & 0xFF ),
        static_cast<char>(( len >> 8 ) & 0xFF ),
        static_cast<char>(( len >> 16 ) & 0xFF ),
        static_cast<char>(( len >> 24 ) & 0xFF ),
    };

    firmwarePacketType.store( Telemetry::PacketType::ESPFirmwareGotPacket, std::memory_order_relaxed );

    serial.write( header, sizeof( header ));

    while( !eof && ( len > 0 )) {
        size_t num = std::min( len, 1024UL );

        if(!( i++ % 100 ))
            Log::debug( "{} firmware bytes to send, {} will be sent now", len, num );

        serial.write( ptr, num );

        ptr += num;
        len -= num;

        {
            std::unique_lock<std::mutex> fl( firmwareMutex );

            firmwareCV.wait( fl );
        }

        if( firmwarePacketType.load( std::memory_order_relaxed ) == Telemetry::PacketType::ESPFirmwareStop ) {
            Log::debug( "Got abort flag ({})", static_cast<int>( firmwarePacketType.load( std::memory_order_relaxed )));
            eof = true;
        }
    }

    Log::debug( "Firmware sent ({})", len );
}