//
// Created by chiheb on 04/02/25.
//

#ifndef MASTERNODE_H
#define MASTERNODE_H
#include "gmisc.h"


class MasterNode : public BaseNode {
    enum class state_t {
        GET_SYNC_FUSION, INIT,BUILD_INITIAL_META_STATE,BUILD_META_STATE,SEND_METASTATE_NAME,POP_METASTATE,PREPARE_COMPUTE_SYNC,
        COMPUTE_SYNC, FIRE_SYNC, TERMINATE_BUILDING
    };
public:
    MasterNode(dss::PetriNet  *petri,std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> _executor);
    auto run() -> void override;


private:
    auto response_receiver(const ros2dss::Response & msg) ->void;

    auto buildInitialMetaState() -> void;
    auto computeEnabledSyncTransitions() -> void;
    auto statemachineMoveToState(const state_t state) -> void;
    auto fireSyncTransition() -> bool ;
    auto executeFireSyncTransitionRequest(const uint32_t id_server, const string &transition) -> std::vector<dss::firing_sync_t> ;

    rclcpp::Publisher<ros2dss::Command>::SharedPtr m_command_pub;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;

    ros2dss_project::msg::Command m_command;

    rclcpp::Subscription<ros2dss::Response>::SharedPtr m_response_sub;
    state_t m_current_state {state_t::INIT};


    AckManage m_ack_modules; // Used to check that all ACK are received

    dss::ArrayModel<std::string> m_metastate_building_name; // Used to store the name of the metastate being built
    std::stack<dss::MetaState*> m_meta_states_stack; // Used to store the metastates
    std::vector<std::string> ml_enabled_fusion_sets; // Used to store the enabled fusion sets


};



#endif //MASTERNODE_H
