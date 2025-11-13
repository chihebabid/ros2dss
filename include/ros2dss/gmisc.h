//
// Created by chiheb on 16/12/24.
//

#ifndef GMISC_H
#define GMISC_H

#define ENABLE_LOGGING 1  // 0: disable loggin, 1: enable

#if ENABLE_LOGGING
#define LOG_INFO RCLCPP_INFO
#define LOG_ERROR RCLCPP_ERROR
#else
#define LOG_INFO(...)
#define LOG_ERROR(...)
#endif


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2dss_project/msg/command.hpp"
#include "ros2dss_project/msg/response.hpp"
#include "ros2dss_project/srv/sync_transition.hpp"
#include "ros2dss_project/msg/firing.hpp"
#include "ros2dss_project/srv/firing_sync_transition_srv.hpp"
#include "ros2dss_project/srv/firing_info.hpp"

namespace ros2dss {
    using SyncTransition=ros2dss_project::srv::SyncTransition;
    using FiringSyncTransitionSrv=ros2dss_project::srv::FiringSyncTransitionSrv;
    using Command=ros2dss_project::msg::Command;
    using Response=ros2dss_project::msg::Response;
    using Firing=ros2dss_project::msg::Firing;
    using InfoFiring=ros2dss_project::srv::FiringInfo;
}


#include "misc.h"
#include "FiringSyncTransitionService.h"
#include "SyncTransitionService.h"

#include "AckManage.h"
#include "BaseNode.h"
#include "MasterNode.h"
#include "SlaveNode.h"




template class dss::ArrayModel<std::string>;
inline bool _enabled_reduction {false};
inline std::mutex _mutex;
inline std::unique_ptr<dss::MarkingArray> _ptr_modules {};
inline std::unique_ptr<dss::ArrayModel<std::string>> _ptr_metastate_name;
inline std::string _current_meta_state_name;
inline std::atomic<uint32_t> _received_sync_count {};



#endif //MISC_H
