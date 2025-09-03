//
// Created by chiheb on 12/12/24.
//

#define ENABLE_LOGGING 1  // Mettre à 0 pour désactiver, 1 pour activer

#if ENABLE_LOGGING
#define LOG_INFO RCLCPP_INFO
#define LOG_ERROR RCLCPP_ERROR
#else
#define LOG_INFO(...)
#define LOG_ERROR(...)
#endif

#include "gmisc.h"
#include <chrono>

using namespace chrono_literals;


int main(int argc, char * argv[]) {
    dss::BuildPetri build;
    if (argc<2) {
        std::cout<<"Usage: "<<argv[0]<<" <filename> [-r] \n";
        std::cout<<"-r: enable reduction of metastates\n";
        return -1;
    }

    LOG_INFO(rclcpp::get_logger("Main"), "File: %s", argv[1]);
    build.setFileName(argv[1]);
    if (argc>2 and strcmp(argv[2],"-r")==0) {
        _enabled_reduction=true;
        LOG_INFO(rclcpp::get_logger("Main"), "Reduction enabled");
    }
    else {
        _enabled_reduction=false;
        LOG_INFO(rclcpp::get_logger("Main"), "Reduction disabled");
    }
    auto petri {build.getPetriNet()};
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(100us);

    auto syncNode {std::make_shared<SyncTransitionService>(petri)};
    auto executor {std::make_shared<rclcpp::executors::MultiThreadedExecutor>()};
    executor->add_node(syncNode);

    while (rclcpp::ok()  and !syncNode->shouldShutdown())    {
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
    LOG_INFO(rclcpp::get_logger("Main"), "Starting main node...\n");
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
    // Main loop for the building of DSS
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() and !base_node->shouldShutdown() )    {
            base_node->run();
            executor->spin_all(0ms);
    }
    auto end = std::chrono::steady_clock::now();
    if (petri->getPetriID()==0) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        RCLCPP_INFO(rclcpp::get_logger("Main"), "DSS building time: %ld ms\n", duration);
    }
    rclcpp::shutdown();
    delete petri;
    return 0;
}