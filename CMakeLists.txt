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

#OpenCV
include_directories(/home/rodrigo/opencv-2.4_location/include)
set(OPENCV24_LIB /home/rodrigo/opencv-2.4_location/lib)

#FaceTracker
include_directories(/home/rodrigo/face-analysis-sdk-master/src)
include_directories(/home/rodrigo/face-analysis-sdk-master/build/src)
set(FACE_ANALYSIS_LIB /home/rodrigo/face-analysis-sdk-master/build/lib)


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
        )
