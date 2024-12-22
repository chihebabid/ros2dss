//
// Created by chiheb on 13/12/24.
//

#ifndef DSSPUBLISHER_H
#define DSSPUBLISHER_H
#include "gmisc.h"

class DSSPublisher : public rclcpp::Node {
public:
    DSSPublisher(dss::PetriNet  *);
    void publishCommand(const ros2dss::Command&);
    uint32_t getCommandSubscribersCount() const;
private:
    void init();
    rclcpp::Publisher<ros2dss::Command>::SharedPtr m_command_pub;
    dss::PetriNet  *m_petri_net;
};



#endif //DSSPUBLISHER_H
