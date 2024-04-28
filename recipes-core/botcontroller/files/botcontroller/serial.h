//
// Created by simone on 2/25/24.
//

#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <mutex>

class Serial
{
public:
    Serial( std::string device, int baud );
    ~Serial();

    bool open();
    void close() { ::close( fd ); fd = -1; }
    int put( unsigned char c ) { return write( &c, 1 ); }

    int write( const void *data, size_t len )
    {
        std::lock_guard<std::recursive_mutex> lock( mutex );

        return static_cast<int>( ::write( fd, data, len ));
    }

    [[nodiscard]] char readChar() const;
    [[nodiscard]] int32_t readInt() const;
    [[nodiscard]] std::vector<char> read( ssize_t bytesToRead ) const;

    [[nodiscard]] std::recursive_mutex& getMutex() { return mutex; }

private:
    std::string device;
    std::recursive_mutex mutex;
    speed_t baud;
    int fd = -1;
};


#endif //SERIAL_H
