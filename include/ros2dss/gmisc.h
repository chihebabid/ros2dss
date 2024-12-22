//
// Created by chiheb on 16/12/24.
//

#ifndef GMISC_H
#define GMISC_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2dss_project/msg/command.hpp"
#include "ros2dss_project/srv/sync_transition.hpp"

#include "misc.h"




#include "SyncTransitionService.h"
#include "DSSSubscriber.h"
#include "DSSPublisher.h"
#include "SMBuilder.h"


template class dss::ArrayModel<std::string>;
inline std::unique_ptr<dss::MarkingArray> _ptr_modules {};
inline std::unique_ptr<dss::ArrayModel<std::string>> _ptr_metastate_name;
inline std::string _current_meta_state_name;




#endif //MISC_H
