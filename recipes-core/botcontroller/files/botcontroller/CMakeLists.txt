cmake_minimum_required(VERSION 3.27)

project(botcontroller)
set(PROJECT_NAME "Bot Controller")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG=1")

option(TELEMETRY_HOST "Telemetry server hostname")
option(TELEMETRY_PORT "Telemetry server port")

find_package( Boost REQUIRED )
find_package( libpointmatcher REQUIRED )
find_package( libnabo REQUIRED )
find_package( yaml-cpp REQUIRED )
find_package( fmt REQUIRED )

find_path(EIGEN_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
    HINTS ENV EIGEN3_INC_DIR
          ENV EIGEN3_DIR
    PATHS Eigen/Core
          /usr/local/include
          /usr/include
    PATH_SUFFIXES include eigen3 eigen
    DOC "Directory containing the Eigen3 header files"
)

set_target_properties(libnabo::nabo PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ""
)

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${libpointmatcher_INCLUDE_DIRS} )
include_directories( ${EIGEN_INCLUDE_DIR} )
include_directories( ${Fmt_INCLUDE_DIRS} )
include_directories( "MiniJson/Source/include" )

file(GLOB botcontroller_SRCS
    LIST_DIRECTORIES false
    "*.cpp"
    "*.c"
    "tinyhttp/http.cpp"
    "MiniJson/Source/src/*.cpp"
)

add_executable(botcontroller ${botcontroller_SRCS})

target_link_libraries(botcontroller ${Boost_LIBS} ${libpointmatcher_LIBRARIES} ${Fmt_LIBS})

target_compile_definitions(botcontroller
    PRIVATE
        FMT_HEADER_ONLY=1
        MAX_HTTP_CONTENT_SIZE=10485760
)

if(TELEMETRY_HOST)
    target_compile_definitions(botcontroller PRIVATE TELEMETRY_HOST="${TELEMETRY_HOST}")
endif()

if(TELEMETRY_PORT)
    target_compile_definitions(botcontroller PRIVATE TELEMETRY_PORT="${TELEMETRY_PORT}")
endif()
