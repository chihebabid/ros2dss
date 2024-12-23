//
// Created by chiheb on 22/12/24.
//
#include "gmisc.h"

using std::placeholders::_1;
using std::placeholders::_2;

SyncTransitionService::SyncTransitionService(dss::PetriNet  *petri):Node("sync_transitions"),m_petri(petri) {
    auto node {std::make_shared<rclcpp::Node>("sync_transitions")};
    if (petri->getPetriID()==0) {
       m_server=create_service<ros2dss::SyncTransition>("sync_transitions",std::bind(&SyncTransitionService::syncTransitionsService,this,_1,_2));
    }
    else {
        // Création du client
        auto client {node->create_client<ros2dss::SyncTransition>("sync_transitions")};
        auto request {std::make_shared<ros2dss::SyncTransition::Request>()};
        request->id=m_petri->getPetriID();
        request->transitions=m_petri->getSyncTransitions();
        while (!client->wait_for_service(1s)) {
             if (!rclcpp::ok()) {
                  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting...");
                  return;
             }
        }

        auto result {client->async_send_request(request)};
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
 			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Synced transitions\n");
 		} else {
 			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to sync transitions!\n");
        }
    }
 }

 void SyncTransitionService::syncTransitionsService(const std::shared_ptr<ros2dss::SyncTransition::Request> request,
 std::shared_ptr<ros2dss::SyncTransition::Response> response)  {
   for (auto &t : request->transitions) {
     //auto transition {petri->getTransition(t.name)};
   }

   }


