//
// Created by chiheb on 22/12/24.
//
#include "rclcpp/rclcpp.hpp"
#include "ros2dss_project/srv/sync_transition.hpp"
#include "misc.h"
#include "NodeSyncTransitionService.h"

namespace ros2dss {


 void syncTransitionsService(const std::shared_ptr<ros2dss_project::srv::SyncTransition::Request> request,
 std::shared_ptr<ros2dss_project::srv::SyncTransition::Response> response)  {
   }


    void updateSyncTransitions(const dss::PetriNet *petri ) {
      if (petri->getPetriID()==0) {
          auto node {std::make_shared<rclcpp::Node>("sync_transitions")};
          auto servic {node->create_service<ros2dss_project::srv::SyncTransition>("sync_transitions",&syncTransitionsService)};
        }
        else {
          }
     }
}