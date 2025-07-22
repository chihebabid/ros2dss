//
// Created by chiheb on 12/12/24.
//


#include "gmisc.h"

using namespace chrono_literals;

int main(int argc, char * argv[]) {
    dss::BuildPetri build;
    if (argc<2) {
        std::cout<<"Usage: "<<argv[0]<<" <filename> [-r] \n";
        std::cout<<"-r: enable reduction of metastates\n";
        return -1;
    }
    build.setFileName(argv[1]);
    if (argc>2 and strcmp(argv[2],"-r")==0) {
        _enabled_reduction=true;
        RCLCPP_INFO(rclcpp::get_logger("Main"), "Reduction enabled");
    }
    else {
        _enabled_reduction=false;
        RCLCPP_INFO(rclcpp::get_logger("Main"), "Reduction disabled");
    }
    auto petri {build.getPetriNet()};
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(500ms);

    auto syncNode {std::make_shared<SyncTransitionService>(petri)};
    auto executor {std::make_shared<rclcpp::executors::MultiThreadedExecutor>()};
    executor->add_node(syncNode);
    while (rclcpp::ok()  and !syncNode->shouldShutdown())
    {
        try {
            executor->spin_all(0ns);
        } catch (const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Main"), "Error in spin: %s", e.what());
        }


        loop_rate.sleep();
    }

    executor->remove_node(syncNode);
    syncNode.reset();

    /*auto  pubNode {std::make_shared<DSSPublisher>(petri)};
    auto  subNode {std::make_shared<DSSSubscriber>(petri)};*/

    std::shared_ptr<BaseNode> base_node{};
    if (petri->getPetriID()==0) {
        base_node=std::make_shared<MasterNode>(petri,executor);
    }
    else {
        auto fireSyncTransitionNodeService {std::make_shared<FiringSyncTransitionService>(petri)};
        executor->add_node(fireSyncTransitionNodeService);
        base_node=std::make_shared<SlaveNode>(petri,fireSyncTransitionNodeService);
    }
    executor->add_node(base_node);

    while (rclcpp::ok() and !base_node->shouldShutdown() )    {
        try {
            base_node->run();
            executor->spin_all(0ms);
        } catch (const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Main"), "Error in spin: %s", e.what());
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    delete petri;
    return 0;
}