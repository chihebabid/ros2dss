//
// Created by chiheb on 12/12/24.
//

#include "rclcpp/rclcpp.hpp"
#include "misc.h"
#include "DSSPublisher.h"
#include "DSSSubscriber.h"
#include "SMBuilder.h"
#include "gmisc.h"

using namespace chrono_literals;

int main(int argc, char * argv[]) {
    dss::BuildPetri build;
    if (argc!=2) {
        std::cout<<"Usage: "<<argv[0]<<" <filename>\n";
        return -1;
    }
    build.setFileName(argv[1]);
    auto petri {build.getPetriNet()};
    rclcpp::init(argc, argv);

    auto syncNode {std::make_shared<SyncTransitionService>(petri)};


    auto  pubNode {std::make_shared<DSSPublisher>(petri)};
    auto  subNode {std::make_shared<DSSSubscriber>(petri)};
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pubNode);
    executor.add_node(subNode);


    rclcpp::WallRate loop_rate(500ms);
    SMBuilder sm_builder {petri,pubNode};
    while (rclcpp::ok())
    {
        try {
            sm_builder.run();
            executor.spin_all(0ns);
        } catch (const rclcpp::exceptions::RCLError & e)
        {

        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    delete petri;
    return 0;
}