//
// Created by chiheb on 12/12/24.
//
#include "rclcpp/rclcpp.hpp"
#include "DSSPublisher.h"
#include "DSSSubscriber.h"
#include "misc.h"

int main(int argc, char * argv[]) {
    dss::BuildPetri build;
    if (argc!=2) {
        std::cout<<"Usage: "<<argv[0]<<" <filename>\n";
        return -1;
    }
    build.setFileName(argv[1]);
    auto petri {build.getPetriNet()};
    rclcpp::init(argc, argv);
    auto  pubNode {std::make_shared<DSSPublisher>(petri)};
    auto  subNode {std::make_shared<DSSSubscriber>()};
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pubNode);
    executor.add_node(subNode);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}