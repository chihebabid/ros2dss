//
// Created by chiheb on 22/12/24.
//
#include "gmisc.h"



SyncTransitionService::SyncTransitionService(dss::PetriNet  *petri):Node("sync_transitions"),m_petri(petri) {
    auto node {std::make_shared<rclcpp::Node>("sync_transitions")};
    if (petri->getPetriID()==0) {
       // auto servic {node->create_service<ros2dss::SyncTransition>("sync_transitions",&syncTransitionsService)};
    }
    else {
    }
 }

 void SyncTransitionService::syncTransitionsService(const std::shared_ptr<ros2dss_project::srv::SyncTransition::Request> request,
 std::shared_ptr<ros2dss_project::srv::SyncTransition::Response> response)  {
   for (auto &t : request->transitions) {
     //auto transition {petri->getTransition(t.name)};
   }

   }


