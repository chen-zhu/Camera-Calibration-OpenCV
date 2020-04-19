PROJECT( opengl_cv )

cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

FIND_PACKAGE( OpenCV REQUIRED)
FIND_PACKAGE( OpenGL REQUIRED)
FIND_PACKAGE( GLU REQUIRED)
FIND_PACKAGE( GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

SET(MySources opengl_cv_skeleton.cpp )
ADD_EXECUTABLE( opengl_cv_skeleton ${MySources} )
TARGET_LINK_LIBRARIES( opengl_cv_skeleton ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )


