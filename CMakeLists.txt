#
# powerscan project build instructions for CMake
#
cmake_minimum_required(VERSION 2.8.0)
project(powerscan C)
set(CMAKE_C_STANDARD 99)

# include some cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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
if(MSVC)
	LIST(APPEND TOOLS_LINK_LIBS ${FFTW_LIBRARIES})
else()
	LIST(APPEND TOOLS_LINK_LIBS fftw3f)
endif()

# Use the libm math library if it's available
find_library(
    MATH_LIBRARIES NAMES m
    PATHS /usr/lib /usr/lib64
)
if (MATH_LIBRARIES)
    list(APPEND TOOLS_LIBS ${MATH_LIBRARIES})
endif ()

# Windows getopt compatibility
if (WIN32)
    list(APPEND COMMON_SOURCES getopt.c)
endif ()

# GNU compile options
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)
endif(CMAKE_C_COMPILER_ID STREQUAL "GNU")

#
# Helper library
#
#set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
#list(APPEND COMMON_SOURCES convenience.c)
#add_library(common STATIC ${COMMON_SOURCES})
#list(APPEND TOOLS_LIBS common)

#
# Build and install executables
#
SET(EXECUTABLES
        powerscan
)

foreach(executable ${EXECUTABLES})
        add_executable(${executable} ${executable}.c)
        target_link_libraries(${executable} ${TOOLS_LIBS})
        install(TARGETS ${executable} RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})
endforeach(executable)
