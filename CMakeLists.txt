cmake_minimum_required(VERSION 2.8)
project( StuCam-vo)
find_package( OpenCV 4.0 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PkgConfig) # pkg-config를 사용하여 라이브러리 검색
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})
add_definitions(${GSTREAMER_CFLAGS_OTHER})

file(GLOB viso
    "src/*.h"
    "src/*.cpp"
)
add_executable( vo ${viso} )
target_link_libraries( vo ${OpenCV_LIBS} ${GSTREAMER_LIBRARIES})
