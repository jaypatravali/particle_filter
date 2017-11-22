cmake_minimum_required(VERSION 2.8)
project( DisplayImage )

find_package( OpenCV REQUIRED )




add_executable( stereo_rectify_all.cpp src/stereo_rectify_all.cpp )
target_link_libraries( stereo_rectify_all.cpp ${OpenCV_LIBS} )


add_executable( disparity_opencv.cpp src/disparity_opencv.cpp )
target_link_libraries( disparity_opencv.cpp ${OpenCV_LIBS} )


