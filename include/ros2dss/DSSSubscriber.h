//
// Created by chiheb on 13/12/24.
//

#ifndef DSSSUBSCRIBER_H
#define DSSSUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DSSSubscriber : public rclcpp::Node {
public:
	DSSSubscriber();
private:
	void receiveMarking();
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_scommand_sub;
};



#endif //DSSSUBSCRIBER_H
