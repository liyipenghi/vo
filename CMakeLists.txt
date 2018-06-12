SET(PROJECT_NAME eval_vo)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmakes")

SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -03 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

FIND_PACKAGE(OpenCV 3.2 REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(fast REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)
#FIND_PACKAGE(svo REQUIRED)
#FIND_PACKAGE(vikit_common REQUIRED)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Include dirs
INCLUDE_DIRECTORIES(
    include
    ${Eigen_INCLUDE_DIRS}
 #   ${svo_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
 #   ${vikit_common_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${fast_INCLUDE_DIRS}
    )

# Set link libraries
LIST(APPEND LINK_LIBS
    ${OpenCV_LIBS}
    #opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs
#    ${svo_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${Boost_LIBRARIES}
#    ${vikit_common_LIBRARIES}
    ${fast_LIBRARIES}
    )

LIST(APPEND SOURCEFILES   
    src/utils.cpp
    src/Frame.cpp
    src/Point3d.cpp
    src/Camera.cpp
    src/Param.cpp
    src/FeatureDetector.cpp
    src/Map.cpp
    src/RobustCost.cpp
#    src/FeatureAlignment.cpp
    src/ImageAlignment.cpp
    src/VisualOdometry.cpp
    )

ADD_LIBRARY(vo SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(vo ${LINK_LIB})

ADD_EXECUTABLE(test test/test.cpp)
TARGET_LINK_LIBRARIES(test ${LINK_LIBS} vo)

ADD_EXECUTABLE(test_vo test/test_vo.cpp)
TARGET_LINK_LIBRARIES(test_vo ${LINK_LIBS} vo)

ADD_EXECUTABLE(test_nls test/test_nls.cpp)
TARGET_LINK_LIBRARIES(test_nls ${LINK_LIBS} vo)

#ADD_EXECUTABLE(test_feat_align test/test_feature_align.cpp)
#TARGET_LINK_LIBRARIES(test_feat_align ${LINK_LIBS} vo)

#ADD_EXECUTABLE(runSVO src/run_svo.cpp)
#TARGET_LINK_LIBRARIES(runSVO ${LINK_LIBS})

#ADD_EXECUTABLE(test src/test_read.cpp)
#TARGET_LINK_LIBRARIES(test ${LINK_LIBS})
