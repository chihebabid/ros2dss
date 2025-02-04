//
// Created by chiheb on 04/02/25.
//

#ifndef MASTERNODE_H
#define MASTERNODE_H
#include "gmisc.h"


class MasterNode : public BaseNode {
    enum class state_t {GET_SYNC_FUSION, INIT,BUILD_INITIAL_META_STATE,BUILD_META_STATE,POP_METASTATE,PREPARE_COMPUTE_SYNC,COMPUTE_SYNC, FIRE_SYNC,TERMINATE_BUILDING};
public:
    MasterNode(dss::PetriNet  *petri);
private:
    void run();
    void response_receiver(const ros2dss::Response & msg) const;


    rclcpp::Publisher<ros2dss::Command>::SharedPtr m_command_pub;
    ros2dss_project::msg::Command m_command;

    rclcpp::Subscription<ros2dss::Response>::SharedPtr m_response_sub;
    state_t m_current_state {state_t::INIT};

};



#endif //MASTERNODE_H
