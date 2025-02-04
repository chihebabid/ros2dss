//
// Created by chiheb on 16/12/24.
//

#ifndef GMISC_H
#define GMISC_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2dss_project/msg/command.hpp"
#include "ros2dss_project/msg/response.hpp"
#include "ros2dss_project/srv/sync_transition.hpp"
#include "ros2dss_project/msg/firing.hpp"
#include "ros2dss_project/srv/firing_sync_transition_srv.hpp"

namespace ros2dss {
    using SyncTransition=ros2dss_project::srv::SyncTransition;
    using FiringSyncTransitionSrv=ros2dss_project::srv::FiringSyncTransitionSrv;
    using Command=ros2dss_project::msg::Command;
    using Response=ros2dss_project::msg::Response;
    using Firing=ros2dss_project::msg::Firing;
}


#include "misc.h"
#include "FiringSyncTransitionService.h"
#include "SyncTransitionService.h"
#include "DSSSubscriber.h"
#include "DSSPublisher.h"
#include "AckManage.h"
#include "BaseNode.h"
#include "MasterNode.h"
#include "SlaveNode.h"
#include "SMBuilder.h"



template class dss::ArrayModel<std::string>;
inline std::mutex _mutex;
inline std::unique_ptr<dss::MarkingArray> _ptr_modules {};
inline std::unique_ptr<dss::ArrayModel<std::string>> _ptr_metastate_name;
inline std::string _current_meta_state_name;
inline std::atomic<uint32_t> _received_sync_count {};



#endif //MISC_H
