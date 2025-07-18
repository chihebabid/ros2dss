//
// Created by chiheb on 01/02/25.
//
#include "gmisc.h"

using std::placeholders::_1;
using std::placeholders::_2;

FiringSyncTransitionService::FiringSyncTransitionService(dss::PetriNet  *petri):Node("firing_sync_transitions"),m_petri(petri) {
        std::string service_name {"firing_sync_transitions_service"+std::to_string(m_petri->getPetriID())};
        m_server_firing_service=create_service<ros2dss::FiringSyncTransitionSrv>(std::move(service_name),std::bind(&FiringSyncTransitionService::firingSyncTransitionsService,this,_1,_2));
}

void FiringSyncTransitionService::firingSyncTransitionsService(const std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Request> req,
                                  std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Response> resp) {
    RCLCPP_INFO(rclcpp::get_logger("Service firing sync: "), "Service request firing transition %s",req->transition.c_str());
    auto _result {m_petri->fireSync(req->transition, m_slave_node->getCurrentMetaState())};
    ml_firing_sync_transitions.insert(
    _result.begin(),
    _result.end()
);
    /*
     if (ml_firing_sync_transitions.empty()) RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"res: is empty");
    else RCLCPP_INFO(rclcpp::get_logger("Service firing sync: "),"res: is not empty");
    for (const auto & t : ml_firing_sync_transitions) {
        RCLCPP_INFO(rclcpp::get_logger("Service firing sync: "),"Source metastate: %s",t.getSCCSource()->getMetaState()->toString().c_str());
        RCLCPP_INFO(rclcpp::get_logger("Service firing sync: "),"Transition fusion name: %s",t.getTransition().c_str());
        RCLCPP_INFO(rclcpp::get_logger("Service firing sync: "),"Dest metastate: %s",t.getDestSCC()->getMetaState()->toString().c_str());
    }
    */

    for (const auto & t : ml_firing_sync_transitions) {
      ros2dss::Firing f;
      f.source=t.getSCCSource()->getName(m_petri);
      f.target=t.getDestSCC()->getName(m_petri);
      resp->lfiring.emplace_back(f);
    }
}


void FiringSyncTransitionService::setNode(SlaveNode *node) {
    m_slave_node=node;
}

const std::set<dss::FiringSyncTransition>& FiringSyncTransitionService::getFiringSyncTransitions() const {
    return ml_firing_sync_transitions;
}

void FiringSyncTransitionService::cleanSCCs() {
    for (auto & elt : ml_firing_sync_transitions) {
        delete elt.getDestSCC()->getMetaState();
        delete elt.getDestSCC();
    }
    ml_firing_sync_transitions.clear();
}


