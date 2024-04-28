//
// Created by simone on 3/24/24.
//

#ifndef BOTCONTROLLER_BINARYSTREAM_H
#define BOTCONTROLLER_BINARYSTREAM_H

#include <cstdint>
#include <ios>
#include <type_traits>

template <typename T>
class BinaryStream : public T
{
    static_assert( std::is_base_of<std::ios, T>::value, "T must derive from std::ios" );

public:
    BinaryStream<T>& operator<<( int32_t x ) { this->write( reinterpret_cast<const char *>( &x ), sizeof( x )); return *this; }
    BinaryStream<T>& operator>>( int32_t& x ) { this->read( reinterpret_cast<char *>( &x ), sizeof( x )); return *this; }

    BinaryStream<T>& operator<<( uint32_t x ) { this->write( reinterpret_cast<const char *>( &x ), sizeof( x )); return *this; }
    BinaryStream<T>& operator>>( uint32_t& x ) { this->read( reinterpret_cast<char *>( &x ), sizeof( x )); return *this; }

    BinaryStream<T>& operator<<( char x ) { this->put( &x ); return *this; }
    BinaryStream<T>& operator>>( char& x ) { x = this->get(); return *this; }

    BinaryStream<T>& operator<<( unsigned char x ) { this->write( reinterpret_cast<const char *>( &x ), sizeof( x )); return *this; }
    BinaryStream<T>& operator>>( unsigned char& x ) { this->read( reinterpret_cast<char *>( &x ), sizeof( x )); return *this; }

    BinaryStream<T>& operator<<( float x ) { this->write( reinterpret_cast<const char *>( &x ), sizeof( x )); return *this; }
    BinaryStream<T>& operator>>( float& x ) { this->read( reinterpret_cast<char *>( &x ), sizeof( x )); return *this; }
};

#endif //BOTCONTROLLER_BINARYSTREAM_H
