cmake_minimum_required(VERSION 3.10)

# set the project name
project(LZRoverEngine VERSION 1.0)
enable_language(C)
enable_language(CXX)

# add the executable
add_executable(${PROJECT_NAME} src/lunarEmulation/glad.c src/external/Worley.cpp src/external/Perlin.cpp src/main.cpp)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(assimp REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} "${CMAKE_SOURCE_DIR}/src/external" )
target_link_libraries(${PROJECT_NAME} glfw assimp ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${CMAKE_DL_LIBS})

