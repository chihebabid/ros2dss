//
// Created by chiheb on 04/02/25.
//

#ifndef SLAVENODE_H
#define SLAVENODE_H

#include "gmisc.h"

class SlaveNode : public BaseNode {
public:
    SlaveNode(dss::PetriNet  *petri,std::shared_ptr<FiringSyncTransitionService> firing_service);
    auto run() -> void override;
    auto getCurrentMetaState() const -> dss::MetaState *;
private:
    auto command_receiver(const ros2dss::Command & msg) -> void;

    rclcpp::Publisher<ros2dss::Response>::SharedPtr m_response_pub;
    ros2dss_project::msg::Response m_response;
    shared_ptr<FiringSyncTransitionService> m_firing_sync_transition_service;

    rclcpp::Subscription<ros2dss::Command>::SharedPtr m_command_sub;
};



#endif //SLAVENODE_H
