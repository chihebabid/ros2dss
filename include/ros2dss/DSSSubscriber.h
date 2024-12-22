//
// Created by chiheb on 13/12/24.
//

#ifndef DSSSUBSCRIBER_H
#define DSSSUBSCRIBER_H



class DSSSubscriber : public rclcpp::Node {
public:
	DSSSubscriber(dss::PetriNet  *);
private:
	void receiveMarking();
	void command_receiver(const ros2dss::Command & msg) const;
	rclcpp::Subscription<ros2dss::Command>::SharedPtr m_command_sub;
	dss::PetriNet  *m_petri_net;
};



#endif //DSSSUBSCRIBER_H
