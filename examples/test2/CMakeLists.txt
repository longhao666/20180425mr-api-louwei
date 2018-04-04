CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

PROJECT(test2 C)

INCLUDE_DIRECTORIES(../../include .)

FIND_LIBRARY(MR_LIB NAMES mrapi PATHS /usr/local/lib/MRAPI  NO_DEFAULT_PATH)  #libpcan.so

ADD_EXECUTABLE(test2 position_profile_test.c MC_PositionProfile)
TARGET_LINK_LIBRARIES(test2  ${MR_LIB} -lm)
#TARGET_LINK_LIBRARIES(${LIB_NAME} -lrt -lpthread -lm)

