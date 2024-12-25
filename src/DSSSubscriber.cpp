//
// Created by chiheb on 13/12/24.
//

#include "gmisc.h"

DSSSubscriber::DSSSubscriber(dss::PetriNet  *petri): Node("dss_receiver"),m_petri_net(petri) {
    rclcpp::QoS qos(rclcpp::KeepLast(m_petri_net->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    m_command_sub = this->create_subscription<ros2dss::Command>(
            "/dss/command", qos, [this](const ros2dss::Command & msg) {
                command_receiver(msg);
            });
}

void DSSSubscriber::receiveMarking() {

}


void DSSSubscriber::command_receiver(const ros2dss::Command & msg) const {
   if (msg.cmd=="INIT") {
       (*_ptr_modules)[static_cast<int>(msg.param)]=1;
   }
   else
   if (msg.cmd=="METASTATE") {
       (*_ptr_metastate_name)[msg.param]=msg.scc;
   }
   else if (msg.cmd=="MOVE_TO_METASTATE") {
       _current_meta_state_name=msg.scc;
   }
    else if (msg.cmd=="SYNC_TRANSITION" && m_petri_net->getPetriID()==0) {
        ++_received_sync_count;
        std::set<std::string> enabled_sync_trans {msg.sync.begin(),msg.sync.end()};
        m_petri_net->getManageTransitionFusionSet()->enableSetFusion(enabled_sync_trans,msg.param);
    }
}

