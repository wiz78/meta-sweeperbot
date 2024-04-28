//
// Created by simone on 3/18/24.
//

#include "telemetry.h"
#include "binaryStream.h"
#include "mainController.h"
#include "occupancyGrid.h"
#include "lidarTypes.h"
#include "log.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>

#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

using namespace std::chrono_literals;

Telemetry::Telemetry( MainController& mainController ) : mainController{ mainController }
{
#if defined(TELEMETRY_HOST) && defined(TELEMETRY_PORT)
    std::thread( &Telemetry::telemetryThread, this ).detach();
    std::thread( &Telemetry::recvCommandsThread, this ).detach();
#endif
}

#if defined(TELEMETRY_HOST) && defined(TELEMETRY_PORT)
void Telemetry::telemetryThread()
{
    for(;;) {
        std::unique_lock lock( mutex );

        cv.wait_for( lock, 5s );
        lock.unlock();

        if( sock < 0 ) {
            auto now = std::chrono::steady_clock::now();

            if( now - lastConnectTime > 5s ) {
                lastConnectTime = now;
                connect();
            }
        }

        for( std::string packet = popPacket(); !packet.empty(); packet = popPacket() )
            if( sock >= 0 ) {
                auto size = static_cast<int32_t>( packet.size() );
                bool ok = sendData( &size, sizeof( size ));

                if( ok )
                    ok = sendData( packet.data(), packet.size() );

                if( !ok ) {
                    close( sock );
                    sock = -1;
                    break;
                }
            }
    }
}

void Telemetry::recvCommandsThread()
{
    for(;;) {

        if( sock < 0 )
            sleep( 5 );
        else {
            uint32_t size;

            if( recvData( &size, sizeof( size ))) {
                auto packet = reinterpret_cast<struct PacketHeader *>( malloc( size + sizeof( size )));

                packet->size = size;

                if( recvData( &packet->packetType, size )) {

                    switch( static_cast<PacketType>( packet->packetType )) {

                        case PacketType::SetTarget:
                            setTarget( reinterpret_cast<struct SetTargetPacket *>( packet ));
                            break;

                        default:
                            Log::warning( "Telemetry - unhandled incoming packet {}", packet->packetType );
                            break;
                    }
                }

                free( packet );
            }
        }
    }
}

void Telemetry::connect()
{
    struct hostent *host = gethostbyname( TELEMETRY_HOST );

    if( host ) {

        Log::debug( "Trying to connect to {}:{}", TELEMETRY_HOST, TELEMETRY_PORT );

        sock = socket( PF_INET, SOCK_STREAM, IPPROTO_TCP );

        if( sock >= 0 ) {
            struct sockaddr_in addr{
                .sin_family = AF_INET,
                .sin_port = htons( std::stoi( TELEMETRY_PORT ))
            };

            memcpy( &addr.sin_addr.s_addr, host->h_addr_list[0], sizeof( addr.sin_addr.s_addr ));

            if( ::connect( sock, reinterpret_cast<struct sockaddr *>( &addr ), sizeof( addr )) < 0 ) {

                close( sock );
                Log::error( "Connection to {}:{} failed", TELEMETRY_HOST, addr.sin_port );

                sock = -1;

            } else {
                int flag = 1;
                char s[20];

                setsockopt( sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof( flag ));
                inet_ntop( AF_INET, &addr.sin_addr, s, sizeof( s ));

                Log::info( "Connected to {}:{}", s, addr.sin_port );
            }
        }

    } else
        Log::error( "Cannot resolve {}", TELEMETRY_HOST );

    if( firstConnectionAttempt ) {

        if( sock < 0 )
            mainController.setExploring( true );

        firstConnectionAttempt = false;
    }
}
#endif

bool Telemetry::sendData( const void *buffer, size_t len ) const
{
    const char *ptr = reinterpret_cast<const char *>( buffer );

    while( len > 0 ) {
        ssize_t sentCount = send( sock, ptr, len, 0 );

        if( sentCount == -1 ) {
            Log::error( "Telemetry - send error {} {}", errno, strerror( errno ));
            return false;
        }

        ptr += sentCount;
        len -= sentCount;
    }

    return true;
}

bool Telemetry::recvData( void *buffer, size_t len ) const
{
    char *ptr = reinterpret_cast<char *>( buffer );

    while( len > 0 ) {
        ssize_t count = recv( sock, ptr, len, 0 );

        if( count == -1 ) {
            Log::error( "Telemetry - recv error {} {}", errno, strerror( errno ));
            return false;
        }

        ptr += count;
        len -= count;
    }

    return true;
}

void Telemetry::pushPacket( [[maybe_unused]] std::string packet )
{
#if defined(TELEMETRY_HOST) && defined(TELEMETRY_PORT)
    std::lock_guard<std::mutex> lock( packetsMutex );

    packets.emplace( std::move( packet ));
    cv.notify_one();
#endif
}

std::string Telemetry::popPacket()
{
    std::lock_guard<std::mutex> lock( packetsMutex );
    std::string ret;

    if( !packets.empty() ) {
        ret = packets.front();
        packets.pop();
    }

    return ret;
}

void Telemetry::sendLIDARPointCloud( const std::vector<PointData>& points )
{
    std::ostringstream stream;

    stream.put( static_cast<uint8_t>( PacketType::LIDAR ));
    stream.write( reinterpret_cast<const char *>( points.data() ), static_cast<std::streamsize>( points.size() * sizeof( PointData )));

    pushPacket( stream );
}

void Telemetry::sendGrid( const OccupancyGrid& grid )
{
    auto now = std::chrono::steady_clock::now();

    if( now - lastGridTime > 1s ) {
        BinaryStream<std::ostringstream> stream;

        stream << static_cast<uint8_t>( PacketType::OccupancyGrid );
        stream << grid;

        pushPacket( stream );

        lastGridTime = now;
    }
}

void Telemetry::sendTransformationMatrix( FloorMap::PM::TransformationParameters matrix )
{
    BinaryStream<std::ostringstream> stream;

    stream << static_cast<uint8_t>( PacketType::TransformationMatrix );

    for( int i = 0; i < matrix.rows(); i++ )
        for( int j = 0; j < matrix.cols(); j++ )
            stream << matrix( i, j );

    pushPacket( stream );
}

void Telemetry::sendYaw( float yaw )
{
    BinaryStream<std::ostringstream> stream;

    stream << static_cast<uint8_t>( PacketType::Yaw );
    stream << yaw;

    pushPacket( stream );
}

void Telemetry::setTarget( const struct SetTargetPacket *packet ) const
{
    mainController.setTarget( Eigen::Vector2i{ packet->x, packet->y } );
}