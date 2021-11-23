cmake_minimum_required(VERSION 3.8)

# todo: check if the options are necessary

if ( CMAKE_COMPILER_IS_GNUCXX )
   set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive" )
endif ()

# set compiler support for C++11 standard
option(USE_CXX11_STD "Product should be build with C++11 compiler option enabled" ON)

if(USE_CXX11_STD)
    set(CMAKE_CXX_STANDARD 11)
endif()

if(MSVC)
    add_definitions(-D_CONSOLE)
else()
    # GCC or Clang
endif()

# BUILD_SHARED_LIBS: Global flag to cause add_library to create shared libraries
# if on. If present and true, this will cause all libraries to be built shared
# unless the library was explicitly added as a static library. This variable is
# often added to projects as an OPTION so that each user of a project can decide
# if they want to build the project using shared or static libraries.
if ( NOT DEFINED BUILD_SHARED_LIBS )
   set ( BUILD_SHARED_LIBS FALSE )
endif ()

