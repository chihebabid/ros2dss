//
// Created by chiheb on 04/02/25.
//

#ifndef MASTERNODE_H
#define MASTERNODE_H
#include "gmisc.h"


class MasterNode : public rclcpp::Node {
    enum class state_t {GET_SYNC_FUSION, INIT,BUILD_INITIAL_META_STATE,BUILD_META_STATE,POP_METASTATE,PREPARE_COMPUTE_SYNC,COMPUTE_SYNC, FIRE_SYNC,TERMINATE_BUILDING};
public:
    MasterNode(dss::PetriNet  *petri);
private:
    void run();
    dss::PetriNet  *m_petri_net;
    rclcpp::Publisher<ros2dss::Command>::SharedPtr m_command_pub;
    state_t m_current_state {state_t::INIT};
};



#endif //MASTERNODE_H
