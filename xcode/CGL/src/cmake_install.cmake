# Install script for directory: /Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

set(CMAKE_BINARY_DIR "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode")

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/Debug/libCGL.a")
    if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
    endif()
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/Release/libCGL.a")
    if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
    endif()
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/MinSizeRel/libCGL.a")
    if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
    endif()
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/RelWithDebInfo/libCGL.a")
    if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    include("/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/CMakeFiles/CGL.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    include("/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/CMakeFiles/CGL.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    include("/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/CMakeFiles/CGL.dir/install-cxx-module-bmi-MinSizeRel.cmake" OPTIONAL)
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    include("/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/xcode/CGL/src/CMakeFiles/CGL.dir/install-cxx-module-bmi-RelWithDebInfo.cmake" OPTIONAL)
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/CGL" TYPE FILE FILES
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/CGL.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/vector2D.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/vector3D.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/vector4D.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/matrix3x3.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/matrix4x4.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/quaternion.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/complex.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/color.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/osdtext.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/viewer.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/base64.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/tinyxml2.h"
    "/Users/carolynwang/desktop/cs184/hw2-meshedit-sp24-tiny-turtle/CGL/src/renderer.h"
    )
endif()

