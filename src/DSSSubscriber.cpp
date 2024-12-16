//
// Created by chiheb on 13/12/24.
//

#include "DSSSubscriber.h"


DSSSubscriber::DSSSubscriber(): Node("dss_receiver") {
    m_command_sub = this->create_subscription<ros2dss_project::msg::Command>(
            "/dss/command", 10, [this](const ros2dss_project::msg::Command & msg) {
                command_receiver(msg);
            });
}

void DSSSubscriber::receiveMarking() {

}


void DSSSubscriber::command_receiver(const ros2dss_project::msg::Command & msg) const {
    std::cout<<"Message: received: "<<msg.cmd<<std::endl;
}

