//
// Created by simone on 3/4/24.
//

#ifndef BOTCONTROLLER_LOG_H
#define BOTCONTROLLER_LOG_H

#include <fmt/format.h>
#include <mutex>

enum class LogLevel
{
    None,
    Info,
    Warning,
    Error,
    Debug,
    Verbose,
    LevelsCount
};

class Log
{
public:
    static void setLevel( LogLevel x )
    {
        currentLevel = x;
    }

    static bool inLevel( LogLevel x )
    {
        return currentLevel >= x;
    }

    template <typename... T>
    static void info( fmt::format_string<T...> format, T&&... args )
    {
        vwrite( LogLevel::Info, format, fmt::make_format_args( args... ));
    }

    template <typename... T>
    static void warning( fmt::format_string<T...> format, T&&... args )
    {
        vwrite( LogLevel::Warning, format, fmt::make_format_args( args... ));
    }

    template <typename... T>
    static void error( fmt::format_string<T...> format, T&&... args )
    {
        vwrite( LogLevel::Error, format, fmt::make_format_args( args... ));
    }

    template <typename... T>
    static void debug( fmt::format_string<T...> format, T&&... args )
    {
        vwrite( LogLevel::Debug, format, fmt::make_format_args( args... ));
    }

    template <typename... T>
    static void verbose( fmt::format_string<T...> format, T&&... args )
    {
        vwrite( LogLevel::Verbose, format, fmt::make_format_args( args... ));
    }

    template <typename... T>
    static void write( LogLevel level, fmt::format_string<T...> format, T&&... args )
    {
        vwrite( level, format, fmt::make_format_args( args... ));
    }

private:
    static LogLevel currentLevel;
    inline static std::mutex mutex;
    inline static bool echoToStdOut = false;

    static void vwrite( LogLevel level, fmt::string_view format, fmt::format_args args );
};

#endif //BOTCONTROLLER_LOG_H
