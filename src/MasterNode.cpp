//
// Created by chiheb on 04/02/25.
//

#include "MasterNode.h"

MasterNode::MasterNode(dss::PetriNet  *petri):Node("dss_master"),m_petri_net(petri) {
      // Create 'dss/command' topic
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));  // Keep the last message
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    m_command_pub=create_publisher<ros2dss::Command>("dss/command",qos);
}


auto MasterNode::run()->void {

}