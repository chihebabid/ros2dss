//
// Created by chiheb on 13/12/24.
//

#ifndef DSSPUBLISHER_H
#define DSSPUBLISHER_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2dss_project/msg/command.hpp"
#include "misc.h"

class DSSPublisher : public rclcpp::Node {
public:
    DSSPublisher(dss::PetriNet  *);
    void publishCommand(const ros2dss_project::msg::Command&);
    uint32_t getCommandSubscribersCount() const;
private:
    void init();
    rclcpp::Publisher<ros2dss_project::msg::Command>::SharedPtr m_command_pub;
    dss::PetriNet  *m_petri_net;
};



#endif //DSSPUBLISHER_H
