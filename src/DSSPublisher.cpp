//
// Created by chiheb on 13/12/24.
//

#include "gmisc.h"

DSSPublisher::DSSPublisher(dss::PetriNet  *petri):Node("dss_publisher"),m_petri_net(petri) {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));  // Keep the last message
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    m_command_pub=create_publisher<ros2dss_project::msg::Command>("dss/command",qos);

    //init();
}

void DSSPublisher::init() {

}


void DSSPublisher::publishCommand(const ros2dss_project::msg::Command &msg) {
    m_command_pub->publish(msg);
}

uint32_t DSSPublisher::getCommandSubscribersCount() const {
    return m_command_pub->get_subscription_count();
}
