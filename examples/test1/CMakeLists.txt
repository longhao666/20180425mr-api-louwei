CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(test1 LANGUAGES C)
SET(CMAKE_BUILD_TYPE Debug)

#INCLUDE_DIRECTORIES(/usr/local/include/MRAPI .)

FIND_LIBRARY(MR_LIB NAMES mrapi PATHS /home/louw/build-mr-api-Desktop-Default  NO_DEFAULT_PATH)  #libpcan.so
MESSAGE("MR_LIB: " ${MR_LIB})


ADD_EXECUTABLE(test1 position_profile_test.c MC_PositionProfile)
TARGET_LINK_LIBRARIES(test1  ${MR_LIB} -lm)
TARGET_LINK_LIBRARIES(test1 -lrt -lpthread -lm)

