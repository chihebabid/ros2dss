//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

SlaveNode::SlaveNode(dss::PetriNet  *petri):BaseNode(petri, "dss_slave") {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Create 'dss/response' topic
    m_response_pub=create_publisher<ros2dss::Response>("dss/response",qos);

    // Subscribe to 'dss/command' topic
    m_command_sub = this->create_subscription<ros2dss::Command>(
            "/dss/command", qos, [this](const ros2dss::Command & msg) {
                command_receiver(msg);
            });

}


auto SlaveNode::run() -> void {
}

auto SlaveNode::command_receiver(const ros2dss::Command & msg) -> void {
  	RCLCPP_INFO(get_logger(), "Received: %s",msg.cmd.c_str());
    if (msg.cmd=="INIT") {
        m_response.msg="ACK";
        m_response.id=m_petri->getPetriID();
        m_response_pub->publish(m_response);
    }
    else if (msg.cmd=="GET_METASTATE") {
      	m_current_meta_state = m_petri->getMetaState(m_petri->getMarquage());
        m_response.msg="ACK_GET_METASTATE";
        m_response.id=m_petri->getPetriID();
        m_response.scc=m_petri->getSCCName(m_current_meta_state->getInitialSCC());
        m_response_pub->publish(m_response);
        RCLCPP_INFO(get_logger(), "Send ACK_GET_METASTATE: %s",m_response.scc.c_str());
    }

}