cmake_minimum_required(VERSION 3.5.1)

project(localization VERSION 1.0)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(PCL REQUIRED COMPONENTS common filters)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
include(GNUInstallDirs)

IF (NOT CMAKE_BUILD_TYPE)
 #SET(CMAKE_BUILD_TYPE "DEBUG")
 SET(CMAKE_BUILD_TYPE "RELEASE") 
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

add_library(localization SHARED
    src/interface_ap.cpp
    src/latlong_utm.cpp
    src/association_problem.cpp
    src/optimization_process.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${EIGEN_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_FILTER_LIBRARY})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES})

set_target_properties(localization PROPERTIES VERSION ${PROJECT_VERSION})

target_include_directories(localization PRIVATE includes)
target_include_directories(localization PRIVATE src)

install(TARGETS localization EXPORT localizationConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME})

install(DIRECTORY includes/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

install(EXPORT localizationConfig DESTINATION share/localization/cmake)


export(TARGETS ${PROJECT_NAME} FILE localizationConfig.cmake)
