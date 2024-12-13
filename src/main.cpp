//
// Created by chiheb on 12/12/24.
//
#include "rclcpp/rclcpp.hpp"
#include "DSSPublisher.h"
#include "DSSSubscriber.h"
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto  pubNode {std::make_shared<DSSPublisher>()};
    auto  subNode {std::make_shared<DSSSubscriber>()};
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pubNode);
    executor.add_node(subNode);
    std::cout<<"Running..\n";
    executor.spin();
    rclcpp::shutdown();
    std::cout<<"Hello   kok\n";
    return 0;
}