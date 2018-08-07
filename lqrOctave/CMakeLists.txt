cmake_minimum_required(VERSION 2.6)

PROJECT(lqr)

INCLUDE_DIRECTORIES(
#/usr/include/
#/usr/local/include/
/usr/include/octave-4.0.0/
)

LINK_DIRECTORIES(
#/usr/lib /usr/local/lib
#/usr/lib/octave-3.0.2/
#/usr/lib/octave-3.0.1/
/usr/lib/octave-4.0.0/
)

ADD_EXECUTABLE(lqr lqr.cpp)

TARGET_LINK_LIBRARIES(lqr octave octinterp )
