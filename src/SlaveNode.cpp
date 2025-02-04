//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

SlaveNode::SlaveNode(dss::PetriNet  *petri):BaseNode(petri, "dss_slave") {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);



    m_module_ss = new dss::ModuleSS(m_petri->getModulesCount());
}
