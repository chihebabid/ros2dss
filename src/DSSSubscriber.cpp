//
// Created by chiheb on 13/12/24.
//
#include "misc.h"
#include "DSSSubscriber.h"
#include "gmisc.h"

DSSSubscriber::DSSSubscriber(dss::PetriNet  *petri): Node("dss_receiver"),m_petri_net(petri) {
    rclcpp::QoS qos(rclcpp::KeepLast(m_petri_net->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    m_command_sub = this->create_subscription<ros2dss_project::msg::Command>(
            "/dss/command", qos, [this](const ros2dss_project::msg::Command & msg) {
                command_receiver(msg);
            });
}

void DSSSubscriber::receiveMarking() {

}


void DSSSubscriber::command_receiver(const ros2dss_project::msg::Command & msg) const {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%s', param: %d", msg.cmd.c_str(),msg.param);
   if (msg.cmd=="INIT") {
       (*_ptr_modules)[static_cast<int>(msg.param)-1]=1;
   }
   else
   if (msg.cmd=="METASTATE") {

   }
}

