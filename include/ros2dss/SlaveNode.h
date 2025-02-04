//
// Created by chiheb on 04/02/25.
//

#ifndef SLAVENODE_H
#define SLAVENODE_H

#include "gmisc.h"

class SlaveNode : public BaseNode {
public:
    SlaveNode(dss::PetriNet  *petri);
private:
    void run() override;
    void command_receiver(const ros2dss::Command & msg) const;

    rclcpp::Publisher<ros2dss::Response>::SharedPtr m_response_pub;
    ros2dss_project::msg::Response m_response;

    rclcpp::Subscription<ros2dss::Command>::SharedPtr m_command_sub;
};



#endif //SLAVENODE_H
