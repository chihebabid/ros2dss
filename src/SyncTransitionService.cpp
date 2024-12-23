//
// Created by chiheb on 22/12/24.
//
#include "gmisc.h"

using std::placeholders::_1;
using std::placeholders::_2;

SyncTransitionService::SyncTransitionService(dss::PetriNet  *petri):Node("sync_transitions"),m_petri(petri) {

    if (petri->getPetriID()==0) {
       m_server=create_service<ros2dss::SyncTransition>("sync_transitions",std::bind(&SyncTransitionService::syncTransitionsService,this,_1,_2));
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Server service for syncing transitions is created...\n");
    }
    else {
        // Création du client
        auto client {create_client<ros2dss::SyncTransition>("sync_transitions")};
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
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),result) == rclcpp::FutureReturnCode::SUCCESS) {
 			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Synced transitions\n");
 		} else {
 			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to sync transitions!\n");
        }
        m_should_shutdown=true;
    }
 }

void SyncTransitionService::syncTransitionsService(const std::shared_ptr<ros2dss::SyncTransition::Request> request,
 std::shared_ptr<ros2dss::SyncTransition::Response> response)  {
	++m_request_count;
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received transitions from %d:\n",request->id);
   	for (auto &t : request->transitions) {
    	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), t.c_str());
   	}
    if (m_request_count==m_petri->getModulesCount()-1) {
    	m_should_shutdown=true;
    }
}

bool SyncTransitionService::shouldShutdown() const {
   	return m_should_shutdown;
}



