CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

PROJECT(test1 C)

INCLUDE_DIRECTORIES(../../include .)

FIND_LIBRARY(MR_LIB NAMES mrapi PATHS ../../lib  NO_DEFAULT_PATH)  #libpcan.so

ADD_EXECUTABLE(test1 position_profile_test.c MC_PositionProfile)
TARGET_LINK_LIBRARIES(test1  ${MR_LIB} -lm)
TARGET_LINK_LIBRARIES(test1 -lrt -lpthread -lm)

