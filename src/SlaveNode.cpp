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
    else if (msg.cmd=="SET_METASTATE_NAME") {
        m_current_meta_state->setName(msg.sync);
        m_module_ss->insertMS(m_current_meta_state);
        m_response.msg="ACK_SET_METASTATE_NAME";
        m_response.id=m_petri->getPetriID();
        m_response_pub->publish(m_response);
        RCLCPP_INFO(get_logger(), "Send ACK_SET_METASTATE_NAME");
    }
    else if (msg.cmd=="MOVE_TO_METASTATE") {
        m_current_meta_state = m_module_ss->findMetaState(msg.scc);
        if (!m_current_meta_state) {
            RCLCPP_ERROR(get_logger(), "Didn\'t find requested metastate %s !",msg.scc.c_str());
        }
        else {
            m_response.msg="ACK_MOVE_TO_METASTATE";
            m_response.id=m_petri->getPetriID();
            m_response_pub->publish(m_response);
            RCLCPP_INFO(get_logger(), "Send ACK_MOVE_TO_METASTATE");
        }
    }
}