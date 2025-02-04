//
// Created by chiheb on 04/02/25.
//

#ifndef MASTERNODE_H
#define MASTERNODE_H
#include "gmisc.h"


class MasterNode : public BaseNode {
    enum class state_t {GET_SYNC_FUSION, INIT,BUILD_INITIAL_META_STATE,BUILD_META_STATE,POP_METASTATE,PREPARE_COMPUTE_SYNC,COMPUTE_SYNC, FIRE_SYNC,TERMINATE_BUILDING};
public:
    MasterNode(dss::PetriNet  *petri,std::shared_ptr<FiringSyncTransitionService> firing_service);
    auto run() -> void override;
private:
    auto response_receiver(const ros2dss::Response & msg) ->void;

    auto buildInitialMetaState() -> void;

    rclcpp::Publisher<ros2dss::Command>::SharedPtr m_command_pub;
    ros2dss_project::msg::Command m_command;

    rclcpp::Subscription<ros2dss::Response>::SharedPtr m_response_sub;
    state_t m_current_state {state_t::INIT};

    std::shared_ptr<FiringSyncTransitionService> m_firing_sync_transition_service;

    dss::MarkingArray m_ack_modules; // Used to check that all ACK are received
};



#endif //MASTERNODE_H
