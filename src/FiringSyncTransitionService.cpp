//
// Created by chiheb on 01/02/25.
//
#include "gmisc.h"

using std::placeholders::_1;
using std::placeholders::_2;

FiringSyncTransitionService::FiringSyncTransitionService(dss::PetriNet  *petri):Node("firing_sync_transitions"),m_petri(petri) {
     if (m_petri->getPetriID()!=0) {
        std::string service_name {"firing_sync_transitions_service"+std::to_string(m_petri->getPetriID())};
        m_server_firing_service=create_service<ros2dss::FiringSyncTransitionSrv>(std::move(service_name),std::bind(&FiringSyncTransitionService::firingSyncTransitionsService,this,_1,_2));
    }
}

void FiringSyncTransitionService::firingSyncTransitionsService(const std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Request> req,
                                  std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Response> resp) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request firing transition %s\n",req->transition.c_str());

    // auto res = m_petri->fireSync(req->transition,m_sm_builder->getCurrentMetaState());
    // if (res.empty()) RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"res: is empty");
//    else RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"res: is not empty");
//    for (const auto & t : res) {
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Source metastate: %s",t.getSCCSource()->getMetaState()->toString().c_str());
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Transition nfusion name: %s",t.getTransition().c_str());
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Dest metastate %s",t.getDestSCC()->getMetaState()->toString().c_str());
//    }
//
//    for (const auto & t : res) {
//      ros2dss::Firing f;
//      f.source=t.getSCCSource()->getName(m_petri);
//      f.target=t.getDestSCC()->getName(m_petri);
//      resp->lfiring.emplace_back(f);
//    }
}



void FiringSyncTransitionService::setNode(MasterNode *node) {
    m_master_node=node;
}