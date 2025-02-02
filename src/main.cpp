//
// Created by chiheb on 12/12/24.
//


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
    rclcpp::WallRate loop_rate(500ms);

    auto syncNode {std::make_shared<SyncTransitionService>(petri)};
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(syncNode);
    while (rclcpp::ok()  && !syncNode->shouldShutdown())
    {
        try {
            executor.spin_all(0ns);
        } catch (const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SyncTransitionService"), "Error in spin: %s", e.what());
        }
        loop_rate.sleep();
    }

    executor.remove_node(syncNode);
    syncNode.reset();

    auto  pubNode {std::make_shared<DSSPublisher>(petri)};
    auto  subNode {std::make_shared<DSSSubscriber>(petri)};
    auto fireSyncTransitionNode {std::make_shared<FiringSyncTransitionService>(petri)};

    executor.add_node(pubNode);
    executor.add_node(subNode);
    if (petri->getPetriID()) {
        executor.add_node(fireSyncTransitionNode);
    }

    SMBuilder sm_builder {petri,pubNode,fireSyncTransitionNode};
    while (rclcpp::ok() )
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