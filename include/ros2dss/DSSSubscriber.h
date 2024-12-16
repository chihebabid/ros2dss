//
// Created by chiheb on 13/12/24.
//

#ifndef DSSSUBSCRIBER_H
#define DSSSUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2dss_project/msg/command.hpp"

class DSSSubscriber : public rclcpp::Node {
public:
	DSSSubscriber();
private:
	void receiveMarking();
	void command_receiver(const ros2dss_project::msg::Command & msg) const;
	rclcpp::Subscription<ros2dss_project::msg::Command>::SharedPtr m_command_sub;
};



#endif //DSSSUBSCRIBER_H
