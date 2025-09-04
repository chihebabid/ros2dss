cmake_minimum_required(VERSION 3.8)
project(ros2dss_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)





find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/Command.msg" "srv/SyncTransition.srv" "msg/Firing.msg" "msg/Response.msg" "srv/FiringSyncTransitionSrv.srv"
            "srv/FiringInfo.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target  ${PROJECT_NAME} rosidl_typesupport_cpp)

set(dependencies
        rclcpp std_msgs
)



# Inclure les fichiers sources du sous-dossier
file(GLOB_RECURSE SOURCES
        dss/*.cpp
)

#message(WARNING ${SOURCES})
add_executable(ros2dss src/main.cpp src/SyncTransitionService.cpp
        src/FiringSyncTransitionService.cpp src/AckManage.cpp src/BaseNode.cpp src/MasterNode.cpp src/SlaveNode.cpp ${SOURCES})
target_include_directories(ros2dss PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/ros2dss dss)
ament_target_dependencies(ros2dss ${dependencies})

target_link_libraries(ros2dss "${cpp_typesupport_target}")


install(TARGETS
        ros2dss
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Installer les fichiers de données
install(DIRECTORY models/
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
