cmake_minimum_required(VERSION 3.21)

project(QtWallShooter
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_PREFIX_PATH "/usr/lib64/cmake")

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets OpenGL)

qt_standard_project_setup()

qt_add_executable(qt-wall-shooter
    src/GameWidget.cpp
    src/GameWidget.h
    src/main.cpp
)

target_link_libraries(qt-wall-shooter
    PRIVATE
        Qt6::Widgets
        Qt6::OpenGLWidgets
        Qt6::OpenGL
)
