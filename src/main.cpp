//
// Created by chiheb on 12/12/24.
//
#include "rclcpp/rclcpp.hpp"
#include "../include/ros2dss/DSSPublisher.h"
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simple_node");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::cout<<"Running..\n";
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout<<"Hello   kok\n";
    return 0;
}