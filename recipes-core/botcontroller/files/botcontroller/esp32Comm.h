//
// Created by simone on 2/25/24.
//

#ifndef BOTCONTROLLER_ESP32COMM_H
#define BOTCONTROLLER_ESP32COMM_H

#include "serial.h"
#include "telemetry.h"

#include <mutex>
#include <atomic>
#include <condition_variable>

class MainController;

class Esp32Comm final
{
public:
    static constexpr unsigned char CMD_READY = 0;
    static constexpr unsigned char CMD_ROTATE = 1;
    static constexpr unsigned char CMD_MOVE = 2;
    static constexpr unsigned char CMD_STOP = 3;
    static constexpr unsigned char CMD_UPDATE = 4;
    static constexpr unsigned char CMD_MIN_DISTANCES = 5;

    explicit Esp32Comm( MainController& mainController );

    [[noreturn]] void run();

    void writePacket( const void *data, size_t len ) { (void)serial.write( data, len ); }

    void cmd( unsigned char c ) { serial.put( c ); }

    void move( int32_t distance );
    void rotate( int direction, float angle );

    void updateFirmware( const std::string& packet );

private:
    typedef struct __attribute__((packed))
    {
        int8_t packetType;
        float euler[3];
        int16_t accelX;
        int16_t accelY;
        int16_t accelZ;
    } IMUPacket;

    MainController &mainController;
    Serial serial;
    std::atomic<Telemetry::PacketType> firmwarePacketType;
    std::mutex firmwareMutex;
    std::condition_variable firmwareCV;
};

#endif //BOTCONTROLLER_ESP32COMM_H
