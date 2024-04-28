//
// Created by simone on 2/25/24.
//

#include "serial.h"

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <utility>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

Serial::Serial( std::string  device, int baud ) : device{std::move(device)}
{
    switch( baud ) {
        case      50:	this->baud =      B50; break;
        case      75:	this->baud =      B75; break;
        case     110:	this->baud =     B110; break;
        case     134:	this->baud =     B134; break;
        case     150:	this->baud =     B150; break;
        case     200:	this->baud =     B200; break;
        case     300:	this->baud =     B300; break;
        case     600:	this->baud =     B600; break;
        case    1200:	this->baud =    B1200; break;
        case    1800:	this->baud =    B1800; break;
        case    2400:	this->baud =    B2400; break;
        case    4800:	this->baud =    B4800; break;
        case    9600:	this->baud =    B9600; break;
        case   19200:	this->baud =   B19200; break;
        case   38400:	this->baud =   B38400; break;
        case   57600:	this->baud =   B57600; break;
        case  115200:	this->baud =  B115200; break;
        case  230400:	this->baud =  B230400; break;
        case  460800:	this->baud =  B460800; break;
        case  500000:	this->baud =  B500000; break;
        case  576000:	this->baud =  B576000; break;
        case  921600:	this->baud =  B921600; break;
        case 1000000:	this->baud = B1000000; break;
        case 1152000:	this->baud = B1152000; break;
        case 1500000:	this->baud = B1500000; break;
        case 2000000:	this->baud = B2000000; break;
        case 2500000:	this->baud = B2500000; break;
        case 3000000:	this->baud = B3000000; break;
        case 3500000:	this->baud = B3500000; break;
        case 4000000:	this->baud = B4000000; break;
    }
}

Serial::~Serial()
{
    close();
}

bool Serial::open()
{
    fd = ::open( device.c_str(), O_RDWR | O_NOCTTY );

    if( fd >= 0 ) {
        struct termios options;
        int status;

        fcntl( fd, F_SETFL, O_RDWR );

        tcgetattr( fd, &options );

        cfmakeraw( &options );
        cfsetispeed( &options, baud );
        cfsetospeed( &options, baud );

        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );
        options.c_oflag &= ~OPOST;

        options.c_cc[ VMIN ] = 0;
        options.c_cc[ VTIME ] = 100; // 10 seconds

        tcsetattr( fd, TCSANOW, &options );

        ioctl( fd, TIOCMGET, &status );

        status |= TIOCM_DTR | TIOCM_RTS;

        ioctl( fd, TIOCMSET, &status );

        usleep( 10000 ); // 10mS
    }

    return fd >= 0;
}

char Serial::readChar() const
{
    std::vector<char> data = read( 1 );

    return data[0];
}

int32_t Serial::readInt() const
{
    std::vector<char> data = read( 4 );
    const auto *bytes = reinterpret_cast<const unsigned char *>( data.data() );

    return bytes[0] | ( bytes[1] << 8 ) | ( bytes[2] << 16 ) | ( bytes[3] << 24 );
}

std::vector<char> Serial::read( ssize_t bytesToRead ) const
{
    std::vector<char> ret;
    size_t len = 0;

    ret.resize( bytesToRead );

    while( bytesToRead > 0 ) {
        ssize_t num = ::read( fd, ret.data() + len, bytesToRead );

        if( num <= 0 )
            break;

        bytesToRead -= num;
        len += num;
    }

    return ret;
}