find_package(OpenCV REQUIRED)
include_directories("/usr/local/include/opencv4")
# if you want to use opencv 3.4.10, you should set the OpenCV_DIR to the path of opencv 3.4.10
#if (${OpenCV_VERSION} VERSION_GREATER "4.0.0")
#    set(OpenCV_DIR  /home/qzj/Downloads/tool/opencv-3.4.10/build)
#    FIND_PACKAGE(OpenCV 	REQUIRED )
#    set(OpenCV_LIB_DIR ${OpenCV_DIR}/lib)
#    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
#    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
#    message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")
#    message(STATUS "OpenCV library path: ${OpenCV_LIB_DIR}")
#    include_directories(${OpenCV_INCLUDE_DIRS})
#    link_libraries(${OpenCV_LIB_DIR}/libopencv_core.so ${OpenCV_LIB_DIR}/libopencv_imgproc.so ${OpenCV_LIB_DIR}/libopencv_highgui.so
#            ${OpenCV_LIB_DIR}/libopencv_imgcodecs.so ${OpenCV_LIB_DIR}/libopencv_videoio.so ${OpenCV_LIB_DIR}/libopencv_video.so
#            ${OpenCV_LIB_DIR}/libopencv_calib3d.so ${OpenCV_LIB_DIR}/libopencv_features2d.so ${OpenCV_LIB_DIR}/libopencv_flann.so
#            ${OpenCV_LIB_DIR}/libopencv_ml.so ${OpenCV_LIB_DIR}/libopencv_objdetect.so ${OpenCV_LIB_DIR}/libopencv_photo.so
#            ${OpenCV_LIB_DIR}/libopencv_stitching.so ${OpenCV_LIB_DIR}/libopencv_superres.so ${OpenCV_LIB_DIR}/libopencv_videostab.so)
#endif()

include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})

