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


void SlaveNode::run() {
}

void SlaveNode::command_receiver(const ros2dss::Command & msg) const {

}