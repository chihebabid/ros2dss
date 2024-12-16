//
// Created by chiheb on 13/12/24.
//

#include "DSSPublisher.h"

DSSPublisher::DSSPublisher(dss::PetriNet  *petri):Node("dss_publisher"),m_petri_net(petri) {
    m_command_pub=create_publisher<std_msgs::msg::String>("dss/command",10);
    //init();
}

void DSSPublisher::init() {
    auto msg {std_msgs::msg::String{}};
    msg.data="START:"+std::to_string(m_petri_net->getNumero());
    m_command_pub->publish(msg);
}


void DSSPublisher::publishCommand(const string& command) {
    auto msg {std_msgs::msg::String{}};
    msg.data=command;
    m_command_pub->publish(msg);
}
