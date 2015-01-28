cmake_minimum_required(VERSION 2.8.3)
project(screenrun)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(${PROJECT_NAME} src/screenrun.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

