cmake_minimum_required(VERSION 3.18.1)
project("clustering_android")
set(CMAKE_CXX_STANDARD 17)

add_library(clustering_android SHARED bindings.cpp clustering/clustering.h clustering/clustering.cpp)

set(OpenCV_DIR C:/Users/krist/Desktop/OpenCV-android-sdk/sdk/native)
include_directories(${OpenCV_DIR}/jni/include)
include_directories(libs)
add_subdirectory(libs/fmt)
add_library(lib_opencv SHARED IMPORTED)
set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION ${OpenCV_DIR}/libs/${ANDROID_ABI}/libopencv_java4.so)

find_library(log-lib log)
target_link_libraries(clustering_android fmt::fmt)
target_link_libraries(clustering_android ${log-lib})
target_link_libraries(clustering_android lib_opencv)
