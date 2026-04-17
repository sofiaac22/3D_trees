cmake_minimum_required(VERSION 3.10)
project(PointCloudTest)

# Incluir headers
include_directories(include)

# Buscar librerías requeridas
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

# Ejecutable principal
add_executable(main
    main.cpp
    src/BasicLidarInCamera.cpp
    src/BinaryPcd.cpp
    src/Sync.cpp
)

# Enlazar librerías necesarias
target_link_libraries(main
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)