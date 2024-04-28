//
// Created by simone on 2/25/24.
//

#include "mainController.h"
#include "esp32Comm.h"
#include "log.h"

#include "tinyhttp/http.hpp"
#include "multipart_parser.h"

#include <thread>
#include <sstream>
#include <algorithm>

static HttpResponse handleSetExploring( MainController& mainController, bool x );
static HttpResponse handleStop( MainController& mainController );
static HttpResponse handleMove( const HttpRequest& req, MainController& mainController );
static HttpResponse handleRotate( const HttpRequest& req, MainController& mainController );
static HttpResponse handleUpdate( const HttpRequest& req, MainController& mainController );

static int uploadData( multipart_parser *parser, const char *at, size_t length );
static int httpHeaderName( multipart_parser *parser, const char *at, size_t length );
static int httpHeaderValue( multipart_parser *parser, const char *at, size_t length );
static std::string getBoundary( const std::string& contentType );

class FormParseData
{
public:
    FormParseData() : buffer( std::ios::binary )
    {
    }

    std::unordered_map<std::string, std::string> headers;
    std::string currentHeader;
    std::ostringstream buffer;
};

int main()
{
    MainController mainController;
    HttpServer httpServer;

    httpServer.when( "/" )->serveFile( "/opt/sweeperbot/www/index.html" );
    httpServer.when( "/startExploring" )->posted( [&]( const HttpRequest& ) { return handleSetExploring( mainController, true ); } );
    httpServer.when( "/stopExploring" )->posted( [&]( const HttpRequest& ) { return handleSetExploring( mainController, false ); } );
    httpServer.when( "/stop" )->posted( [&]( const HttpRequest& ) { return handleStop( mainController ); } );
    httpServer.when( "/move" )->posted( [&]( const HttpRequest& req ) { return handleMove( req, mainController ); } );
    httpServer.when( "/rotate" )->posted( [&]( const HttpRequest& req ) { return handleRotate( req, mainController ); } );
    httpServer.when( "/update" )->posted( [&]( const HttpRequest& req ) { return handleUpdate( req, mainController ); } );

    std::thread( [&httpServer]() { httpServer.startListening( 8000 ); } ).detach();

#if !defined(TELEMETRY_HOST) || !defined(TELEMETRY_PORT)
    mainController.setExploring( true );
#endif

    mainController.getEsp32Comm().run();

    return 0;
}

HttpResponse handleSetExploring( MainController& mainController, bool x )
{
    mainController.setExploring( x );

    return HttpResponse{ 200 };
}

HttpResponse handleStop( MainController& mainController )
{
    mainController.getEsp32Comm().cmd( Esp32Comm::CMD_STOP );

    return HttpResponse{ 200 };
}

HttpResponse handleMove( const HttpRequest& req, MainController& mainController )
{
    const auto& data = req.json();
    int direction = 1;

    if( data.contains( "backwards" ))
        direction = -1;

    mainController.getEsp32Comm().move( static_cast<int32_t>( std::stoi( data.value( "distance", "" ))) * direction );

    return HttpResponse{ 200 };
}

HttpResponse handleRotate( const HttpRequest& req, MainController& mainController )
{
    const auto& data = req.json();
    int direction = 1;

    if( data.contains( "ccw" ))
        direction = -1;

    mainController.getEsp32Comm().rotate( direction, static_cast<float>( std::stoi( data.value( "angle", "0" ))) * M_PI / 180.f );

    return HttpResponse{ 200 };
}

HttpResponse handleUpdate( const HttpRequest& req, MainController& mainController )
{
    const auto& data = req.content();
    multipart_parser_settings callbacks {
        .on_header_field = httpHeaderName,
        .on_header_value = httpHeaderValue,
        .on_part_data = uploadData,
    };
    std::string contentType = req[ "Content-Type" ];
    std::string boundary = getBoundary( contentType );
    multipart_parser *parser = multipart_parser_init( boundary.c_str(), &callbacks );
    FormParseData parseData{};

    Log::debug( "Firmware upload - boundary: {}", boundary );

    multipart_parser_set_data( parser, &parseData );
    multipart_parser_execute( parser, data.data(), data.size() );
    multipart_parser_free( parser );

    if( parseData.headers.find( "content-disposition" ) != parseData.headers.end() ) {
        std::string content = parseData.buffer.str();
        size_t len = content.size();

        Log::debug( "Firmware upload - got {} content bytes", len );

        mainController.getEsp32Comm().updateFirmware( content );
    }

    return HttpResponse{ 200, "text/plain", "OK" };
}

int uploadData( multipart_parser *parser, const char *at, size_t length )
{
    std::ostringstream& buffer = reinterpret_cast<FormParseData *>( multipart_parser_get_data( parser ))->buffer;

    buffer.write( at, static_cast<std::streamsize>( length ));

    return 0;
}

int httpHeaderName( multipart_parser *parser, const char *at, size_t length )
{
    auto data = reinterpret_cast<FormParseData *>( multipart_parser_get_data( parser ));

    data->currentHeader = std::string( at, at + length );

    std::transform( data->currentHeader.begin(), data->currentHeader.end(), data->currentHeader.begin(),
                   []( unsigned char c ) { return std::tolower( c ); } );

    return 0;
}

int httpHeaderValue( multipart_parser *parser, const char *at, size_t length )
{
    auto data = reinterpret_cast<FormParseData *>( multipart_parser_get_data( parser ));

    data->headers[ data->currentHeader ] = std::string( at, at + length );

    Log::debug( "Firmware upload - header {}: {}", data->currentHeader, data->headers[ data->currentHeader ] );

    return 0;
}

std::string getBoundary( const std::string& contentType )
{
    static const std::string paramName = "boundary=";
    std::string::size_type start = contentType.find( paramName );
    std::string ret;

    if( start != std::string::npos ) {
        bool quoted;
        std::string delimiter;
        std::string::size_type end;

        start += paramName.length();
        quoted = contentType[ start ] == '"';

        if( quoted ) {
            start++;
            delimiter = "\"";
        } else
            delimiter = ";";

        end = contentType.find( delimiter, start );

        if( end == std::string::npos )
            end = contentType.length() + 1;

        ret = contentType.substr( start, end - start );
    }

    return "--" + ret;
}