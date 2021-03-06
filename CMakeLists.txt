cmake_minimum_required(VERSION 2.8)
project(BlendShapesViewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenGL REQUIRED)

# Compile external dependencies
add_subdirectory(external)

# On Visual 2005 and above, this module can set the debug working directory
cmake_policy(SET CMP0026 OLD)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/rpavlik-cmake-modules-fe2273")
include(CreateLaunchers)
include(MSVCMultipleProcessCompile) # /MP

#set(BOOST_LIBRARYDIR /usr/include/boost)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED COMPONENTS system filesystem regex )
message(status "** Boost Include: ${Boost_INCLUDE_DIR}")
message(status "** Boost Libraries: ${Boost_LIBRARY_DIRS}")
message(status "** Boost Libraries: ${Boost_LIBRARIES}")

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


#OpenCV on Rodrigo's computer
#include_directories(/home/rodrigo/opencv-2.4_location/include)
#set(OPENCV24_LIB /home/rodrigo/opencv-2.4_location/lib)

#FaceTracker On Rodrigo's computer
#include_directories(/home/rodrigo/face-analysis-sdk-master/src)
#include_directories(/home/rodrigo/face-analysis-sdk-master/build/src)
#set(FACE_ANALYSIS_LIB /home/rodrigo/face-analysis-sdk-master/build/lib)

#OpenCV on Juarez's computer
include_directories(/opt/opencv/opencv-2.4.13/release/include)
set(OPENCV24_LIB /opt/opencv/opencv-2.4.13/release/lib)

#FaceTracker on Juarez's computer
include_directories(/opt/face-analysis-sdk-master/src)
include_directories(/opt/face-analysis-sdk-master/build/src)
set(FACE_ANALYSIS_LIB /opt/face-analysis-sdk-master/build/lib)


include_directories(
        external/AntTweakBar-1.16/include/
        external/glfw-3.1.2/include/GLFW/
        external/glm-0.9.7.1/
        external/glew-1.13.0/include/
        external/assimp-3.0.1270/include/
        external/bullet-2.81-rev2613/src/
        .
)

set(ALL_LIBS
        ${OPENGL_LIBRARY}
        glfw
        GLEW_1130
        )

add_definitions(
        -DTW_STATIC
        -DTW_NO_LIB_PRAGMA
        -DTW_NO_DIRECT3D
        -DGLEW_STATIC
        -D_CRT_SECURE_NO_WARNINGS
)


set(SOURCE_FILES
        src/main.cpp
        src/windowdump.cpp
        src/trackball.cc
        src/display.h
        src/display.cpp
        src/global.h
        src/global.cpp
        src/DrawObject.cpp
        src/DrawObject.h
        src/structs.h
        src/Filter.cpp
        src/Filter.h
        src/FIR.h
        src/FIR.cpp
        src/util.h)
add_executable(BlendShapesViewer ${SOURCE_FILES})
target_link_libraries(BlendShapesViewer
        ${ALL_LIBS}
        ${OPENCV24_LIB}/libopencv_highgui.so
        ${OPENCV24_LIB}/libopencv_imgproc.so
        ${OPENCV24_LIB}/libopencv_objdetect.so
        ${OPENCV24_LIB}/libopencv_core.so
        ${FACE_ANALYSIS_LIB}/libutilities.so
        ${FACE_ANALYSIS_LIB}/libclmTracker.so
        ${FACE_ANALYSIS_LIB}/libavatarAnim.so
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_REGEX_LIBRARY}
        ${Boost_FILESYSTEM_LIBRARY}
        )


add_executable(TrackAndShow
        src/applyTrackerAndShow.cpp)
target_link_libraries(TrackAndShow
        ${OPENCV24_LIB}/libopencv_highgui.so
        ${OPENCV24_LIB}/libopencv_imgproc.so
        ${OPENCV24_LIB}/libopencv_objdetect.so
        ${OPENCV24_LIB}/libopencv_core.so
        ${FACE_ANALYSIS_LIB}/libutilities.so
        ${FACE_ANALYSIS_LIB}/libclmTracker.so
        ${FACE_ANALYSIS_LIB}/libavatarAnim.so
        )



set(SOURCE_FILES2
        src/BlendShapeDemo.cpp
        src/trackball.cc
        src/display.h
        src/display.cpp
        src/global.h
        src/global.cpp
        src/DrawObject.cpp
        src/DrawObject.h
        src/structs.h
        src/Filter.cpp
        src/Filter.h
        src/util.h)
add_executable(BlendShapesDemo ${SOURCE_FILES2})
target_link_libraries(BlendShapesDemo
        ${ALL_LIBS}
        ${OPENCV24_LIB}/libopencv_highgui.so
        ${OPENCV24_LIB}/libopencv_imgproc.so
        ${OPENCV24_LIB}/libopencv_objdetect.so
        ${OPENCV24_LIB}/libopencv_core.so
        ${FACE_ANALYSIS_LIB}/libutilities.so
        ${FACE_ANALYSIS_LIB}/libclmTracker.so
        ${FACE_ANALYSIS_LIB}/libavatarAnim.so
        )

set(SOURCE_FILES3
        src/TrackAnd3DEstimate.cpp
        src/trackball.cc
        src/display.h
        src/display.cpp
        src/global.h
        src/global.cpp
        src/DrawObject.cpp
        src/DrawObject.h
        src/structs.h
        src/Filter.cpp
        src/Filter.h
        src/util.h)
add_executable(TrackAnd3DEstimate ${SOURCE_FILES3})
target_link_libraries(TrackAnd3DEstimate
        ${ALL_LIBS}
        ${OPENCV24_LIB}/libopencv_highgui.so
        ${OPENCV24_LIB}/libopencv_imgproc.so
        ${OPENCV24_LIB}/libopencv_objdetect.so
        ${OPENCV24_LIB}/libopencv_core.so
        ${FACE_ANALYSIS_LIB}/libutilities.so
        ${FACE_ANALYSIS_LIB}/libclmTracker.so
        ${FACE_ANALYSIS_LIB}/libavatarAnim.so
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_REGEX_LIBRARY}
        ${Boost_FILESYSTEM_LIBRARY}
        )

set(SOURCE_FILES4
        src/RecordVideo.cpp
        src/trackball.cc
        src/display.h
        src/display.cpp
        src/global.h
        src/global.cpp
        src/DrawObject.cpp
        src/DrawObject.h
        src/structs.h
        src/Filter.cpp
        src/Filter.h
        src/util.h)
add_executable(RecordVideo ${SOURCE_FILES4})
target_link_libraries(RecordVideo
        ${ALL_LIBS}
        ${OPENCV24_LIB}/libopencv_highgui.so
        ${OPENCV24_LIB}/libopencv_imgproc.so
        ${OPENCV24_LIB}/libopencv_objdetect.so
        ${OPENCV24_LIB}/libopencv_core.so
        ${FACE_ANALYSIS_LIB}/libutilities.so
        ${FACE_ANALYSIS_LIB}/libclmTracker.so
        ${FACE_ANALYSIS_LIB}/libavatarAnim.so
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_REGEX_LIBRARY}
        ${Boost_FILESYSTEM_LIBRARY}
        )
