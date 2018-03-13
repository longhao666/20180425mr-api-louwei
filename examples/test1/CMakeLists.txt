CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(test1 LANGUAGES C)

INCLUDE_DIRECTORIES(/usr/local/include/MRAPI .)

FIND_LIBRARY(MR_LIB NAMES mrapi PATHS /usr/local/lib/MRAPI  NO_DEFAULT_PATH)  #libpcan.so

ADD_EXECUTABLE(test1 position_profile_test.c MC_PositionProfile)
TARGET_LINK_LIBRARIES(test1  ${MR_LIB} -lm)
TARGET_LINK_LIBRARIES(test1 -lrt -lpthread -lm)

