//
// Created by chiheb on 01/02/25.
//
#include "gmisc.h"

using std::placeholders::_1;
using std::placeholders::_2;

FiringSyncTransitionService::FiringSyncTransitionService(dss::PetriNet  *petri):Node("firing_sync_transitions"),m_petri(petri) {
  if (m_petri->getPetriID()==0) {
    m_client_firing_service=create_client<ros2dss::FiringSyncTransition>("firing_sync_transitions_service_client");
    }
    else {
        m_server_firing_service=create_service<ros2dss::FiringSyncTransition>("firing_sync_transitions_service",std::bind(&FiringSyncTransitionService::firingSyncTransitionsService,this,_1,_2));
    }
}

void FiringSyncTransitionService::firingSyncTransitionsService(const std::shared_ptr<ros2dss::FiringSyncTransition::Request> req,
                                  std::shared_ptr<ros2dss::FiringSyncTransition::Response> resp) {


}