//
// Created by chiheb on 13/12/24.
//

#include "DSSPublisher.h"

DSSPublisher::DSSPublisher(dss::PetriNet  *petri):Node("dss_publisher"),m_petri_net(petri) {
    m_command_pub=create_publisher<ros2dss_project::msg::Command>("dss/command",10);
    //init();
}

void DSSPublisher::init() {

}


void DSSPublisher::publishCommand(const ros2dss_project::msg::Command &msg) {
    m_command_pub->publish(msg);
}
