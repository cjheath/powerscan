#
# powerscan project build instructions for CMake
#
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)

project(powerscan CXX C)

# search current directory first for #include <...>
include_directories(${PROJECT_SOURCE_DIR})

set(INSTALL_DEFAULT_BINDIR "bin" CACHE STRING "Appended to CMAKE_INSTALL_PREFIX")

#
# Dependencies
#

# Use the SoapySDR system:
find_package(SoapySDR "0.6" NO_MODULE)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()
include_directories(${SoapySDR_INCLUDE_DIRS})
list(APPEND TOOLS_LIBS ${SoapySDR_LIBRARIES})

# Use the fftw library:
find_library(
    FFTW3F_LIBRARIES NAMES fftw3f
    PATHS /usr/lib /usr/lib64 /usr/local/lib
) 
if (FFTW3F_LIBRARIES)
    list(APPEND TOOLS_LIBS ${FFTW3F_LIBRARIES})
endif ()

# Use the libm math library if it's available:
find_library(
    MATH_LIBRARIES NAMES m
    PATHS /usr/lib /usr/lib64
)
if (MATH_LIBRARIES)
    list(APPEND TOOLS_LIBS ${MATH_LIBRARIES})
endif ()

# Windows getopt compatibility:
if (WIN32)
    list(APPEND COMMON_SOURCES getopt.c)
endif ()

# httplib:
# Get https://github.com/yhirose/cpp-httplib, then:
# mkdir -p build; cd build; cmake -DCMAKE_BUILD_TYPE=Release .. && sudo cmake --build . --target install
# find_package(httplib COMPONENTS OpenSSL ZLIB Brotli)
find_package(httplib)
list(APPEND TOOLS_LIBS httplib::httplib)

# GNU compile options
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)
endif(CMAKE_C_COMPILER_ID STREQUAL "GNU")

#
# Build and install executables
#
SET(EXECUTABLES
        powerscan
)

list(APPEND COMMON_SOURCES webserver.cpp)

foreach(executable ${EXECUTABLES})
        add_executable(${executable} ${executable}.c ${COMMON_SOURCES})
        target_link_libraries(${executable} ${TOOLS_LIBS})
        install(TARGETS ${executable} RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})
endforeach(executable)
