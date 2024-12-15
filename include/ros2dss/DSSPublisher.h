//
// Created by chiheb on 13/12/24.
//

#ifndef DSSPUBLISHER_H
#define DSSPUBLISHER_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "misc.h"

class DSSPublisher : public rclcpp::Node {
public:
    DSSPublisher(dss::PetriNet  *);
private:
    void init();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_command_pub;
    dss::PetriNet  *m_petri_net;
};



#endif //DSSPUBLISHER_H
