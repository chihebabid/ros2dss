//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

MasterNode::MasterNode(dss::PetriNet  *petri):BaseNode(petri,"dss_master") {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

     // Create 'dss/command' topic
    m_command_pub=create_publisher<ros2dss::Command>("dss/command",qos);

    // Subscribe to 'dss/response' topic
    m_response_sub = this->create_subscription<ros2dss::Response>(
            "/dss/response", qos, [this](const ros2dss::Response & msg) {
                response_receiver(msg);
            });


}


auto MasterNode::run()->void {
    switch (m_current_state) {
        case state_t::GET_SYNC_FUSION :
            m_current_state=state_t::INIT;
        break;

        case state_t::INIT:
            RCLCPP_INFO(get_logger(), "Current SM: INIT");
            if (m_command_pub->get_subscription_count() == m_petri->getModulesCount()-1) {
                m_current_state = state_t::BUILD_INITIAL_META_STATE;
                m_command.cmd = "INIT";
                m_command.param = m_petri->getPetriID();
                m_command_pub->publish(m_command);
                /*for (uint32_t i{}; i < m_petri->getModulesCount(); ++i) {
                    if ((*_ptr_modules)[i] != 1) {
                        m_current_state = state_t::INIT;
                        break;
                    }
                }*/
            }
        break;
    }

}

auto MasterNode::response_receiver(const ros2dss::Response & msg) const -> void{
}