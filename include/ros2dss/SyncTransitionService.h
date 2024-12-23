//
// Created by chiheb on 22/12/24.
//

#ifndef GETSYNCTRANSITIONS_H
#define GETSYNCTRANSITIONS_H

/*
 * This file contains the functions to get the synchronization transitions
 *
 */

struct transitionfusionset_t {
    std::string name;
    std::map<uint32_t, bool> enabled;
    uint32_t module_count {};
};

class SyncTransitionService : public rclcpp::Node {
public:
    SyncTransitionService(dss::PetriNet  *);
    bool shouldShutdown() const;

private:
    void updateSyncTransitions(const dss::PetriNet *petri );
    void syncTransitionsService(const std::shared_ptr<ros2dss::SyncTransition::Request>,std::shared_ptr<ros2dss::SyncTransition::Response> );
    dss::PetriNet *m_petri;
    rclcpp::Service<ros2dss::SyncTransition>::SharedPtr m_server {};
    bool m_should_shutdown {false};
    uint32_t m_request_count {};

};



#endif //GETSYNCTRANSITIONS_H
