//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

SlaveNode::SlaveNode(dss::PetriNet  *petri):BaseNode(petri, "dss_slave") {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

}


void SlaveNode::run() {
}

void SlaveNode::command_receiver(const ros2dss::Command & msg) const {

}