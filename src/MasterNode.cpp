//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

MasterNode::MasterNode(dss::PetriNet  *petri,std::shared_ptr<FiringSyncTransitionService> firing_service):BaseNode(petri,"dss_master"),m_ack_modules(petri->getModulesCount()) {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

     // Create 'dss/command' topic
    m_command_pub=create_publisher<ros2dss::Command>("dss/command",qos);

    // Subscribe to 'dss/response' topic
    m_response_sub = this->create_subscription<ros2dss::Response>(
            "/dss/response", qos, [this](const ros2dss::Response & msg) {
                response_receiver(msg);
            });
    for (uint32_t i{}; i < m_petri->getModulesCount(); ++i) {
        m_ack_modules[i] = 0;
    }
    m_ack_modules[m_petri->getPetriID()] = 1;
}


auto MasterNode::run()->void {
    switch (m_current_state) {
        case state_t::GET_SYNC_FUSION :
            RCLCPP_INFO(get_logger(), "Current SM: INIT");
            m_current_state=state_t::INIT;
        break;

        case state_t::INIT:
            RCLCPP_INFO(get_logger(), "Current SM: INIT");
            if (m_command_pub->get_subscription_count() == m_petri->getModulesCount()-1) {
                m_current_state = state_t::BUILD_INITIAL_META_STATE;
                m_command.cmd = "INIT";
                m_command.param = m_petri->getPetriID();
                m_command_pub->publish(m_command);
                for (uint32_t i{}; i < m_petri->getModulesCount(); ++i) {
                    if (m_ack_modules[i] != 1) {
                        m_current_state = state_t::INIT;
                        break;
                    }
                }
            }
        break;

        case state_t::BUILD_INITIAL_META_STATE:
            RCLCPP_INFO(get_logger(), "Current SM: BUILD_INITIAL_META_STATE");
            buildInitialMetaState();
            m_current_state=state_t::BUILD_META_STATE;
            break;
    }

}

auto MasterNode::response_receiver(const ros2dss::Response & msg) -> void {
    if (msg.cmd=="ACK") {
        m_ack_modules[msg.param]=1;
    }
}

auto MasterNode::buildInitialMetaState() -> void {
    m_current_meta_state = m_petri->getMetaState(m_petri->getMarquage());
    // _ptr_metastate_name = std::make_unique<dss::ArrayModel<std::string> >(m_petri->getModulesCount());
    // _ptr_metastate_name->operator[](m_petri->getPetriID()) = m_petri->getSCCName(m_current_meta_state->getInitialSCC());
}