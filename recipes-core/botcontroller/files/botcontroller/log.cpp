//
// Created by simone on 3/4/24.
//

#include "log.h"

#include <string>
#include <iostream>

#include <fmt/chrono.h>

LogLevel Log::currentLevel = LogLevel::Debug;

void Log::vwrite( LogLevel level, fmt::string_view format, fmt::format_args args )
{
    if( level <= currentLevel ) {
        static const std::string levels[] = { "", "INFO", "WARN", "ERROR", "DEBUG", "VERBOSE" };
        static_assert(( sizeof( levels ) / sizeof( levels[0] )) == static_cast<int>( LogLevel::LevelsCount ));
        std::string prefix = fmt::format( "{} ", levels[ static_cast<int>( level ) ] );
        std::string message = fmt::vformat( format, args );
        std::lock_guard<std::mutex> lock( mutex );

        std::cout << prefix << message << std::endl;
    }
}
