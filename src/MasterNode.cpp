//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

MasterNode::MasterNode(dss::PetriNet  *petri,std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> _executor):BaseNode(petri,"dss_master"),m_executor(_executor),m_ack_modules(petri->getModulesCount()),m_metastate_building_name(petri->getModulesCount()) {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

     // Create 'dss/command' topic
    m_command_pub=create_publisher<ros2dss::Command>("dss/command",qos);

    // Subscribe to 'dss/response' topic
    m_response_sub = this->create_subscription<ros2dss::Response>(
            "/dss/response", qos, [this](const ros2dss::Response & msg) {
                response_receiver(msg);
            });
    m_ack_modules.reset();
    m_ack_modules[m_petri->getPetriID()] = 1;
    RCLCPP_INFO(get_logger(), "Modules count: %d",m_petri->getModulesCount());
}


auto MasterNode::run()->void {
    switch (m_current_state) {
        case state_t::GET_SYNC_FUSION :
            RCLCPP_INFO(get_logger(), "Current SM: GET_SYNC_FUSION");
            m_current_state=state_t::INIT;
        break;

        case state_t::INIT:
            RCLCPP_INFO(get_logger(), "Current SM: INIT");
            if (m_command_pub->get_subscription_count() == m_petri->getModulesCount()-1) {
                m_command.cmd = "INIT";
                m_command_pub->publish(m_command);
                statemachineMoveToState(state_t::BUILD_INITIAL_META_STATE);
            }
        break;

        case state_t::BUILD_INITIAL_META_STATE:
            RCLCPP_INFO(get_logger(), "Current SM: BUILD_INITIAL_META_STATE");
            buildInitialMetaState();
            m_current_state=state_t::BUILD_META_STATE;
            break;

        case state_t::BUILD_META_STATE:
          	RCLCPP_INFO(get_logger(), "Current SM: BUILD_META_STATE");
            if (m_ack_modules.all()) {
              	m_current_meta_state->setName(m_metastate_building_name);
                RCLCPP_INFO(get_logger(),"Built Metastate: %s",m_current_meta_state->toString().c_str());
                m_module_ss->insertMS(m_current_meta_state);
                m_meta_states_stack.push(m_current_meta_state);
            	m_current_state = state_t::SEND_METASTATE_NAME;
            }
            break;

        case state_t::SEND_METASTATE_NAME:
            RCLCPP_INFO(get_logger(), "Current SM: SEND_METASTATE_NAME");
          	m_command.cmd = "SET_METASTATE_NAME";
            m_command.sync = m_metastate_building_name;
            m_command_pub->publish(m_command);
            statemachineMoveToState(state_t::POP_METASTATE);
            break;

        case state_t::POP_METASTATE:
            RCLCPP_INFO(get_logger(), "Current SM: POP_METASTATE");
            if (m_meta_states_stack.empty()) {
                    m_current_state = state_t::TERMINATE_BUILDING;
            } else {
                    m_current_meta_state = m_meta_states_stack.top();
                    m_meta_states_stack.pop();
                    auto manageFusion {m_petri->getManageTransitionFusionSet()};
    				manageFusion->reset();
                    m_command.cmd = "MOVE_TO_METASTATE";
                    m_command.scc = m_current_meta_state->toString();
                    m_command_pub->publish(m_command);
                    m_current_state = state_t::PREPARE_COMPUTE_SYNC;
            }
            break;

        case state_t::PREPARE_COMPUTE_SYNC:
            RCLCPP_INFO(get_logger(), "Current SM: PREPARE_COMPUTE_SYNC");
            computeEnabledSyncTransitions();
            statemachineMoveToState(state_t::COMPUTE_SYNC);
            break;

	    case state_t::COMPUTE_SYNC:
            RCLCPP_INFO(get_logger(), "Current SM: COMPUTE_SYNC");
            {
              	auto manage {m_petri->getManageTransitionFusionSet()};
            	ml_enabled_fusion_sets=manage->getEnabledFusionSets();
            	for (auto & elt : ml_enabled_fusion_sets) {
                	RCLCPP_INFO(get_logger(),"enabled fusion %s\n",elt.c_str());
            	}
            	manage->display();
			}
            m_current_state=state_t::FIRE_SYNC;
            break;

        case state_t::FIRE_SYNC:
            RCLCPP_INFO(get_logger(), "Current SM: FIRE_SYNC");
            if (!fireSyncTransition()) {
                // No more fusion set is enabled
                m_current_state=state_t::POP_METASTATE;
            }
            else {
                // There are still non processed fusion sets
            }
            break;
          
        case state_t::TERMINATE_BUILDING:
          RCLCPP_INFO(get_logger(), "Current SM: TERMINATE_BUILDING");
          break;
    }

}

auto MasterNode::response_receiver(const ros2dss::Response & resp) -> void {
    RCLCPP_INFO(get_logger(), "Received: %s",resp.msg.c_str());
    if (resp.msg=="ACK" and m_current_state==state_t::INIT) {
        m_ack_modules[resp.id]=1;
    }
    else if (resp.msg=="ACK_GET_METASTATE" and m_current_state==state_t::BUILD_META_STATE) {
        RCLCPP_INFO(get_logger(), "Received: %s %s",resp.msg.c_str(),resp.scc.c_str());
      	m_metastate_building_name[resp.id]=resp.scc;
        m_ack_modules[resp.id]=1;
    }
    else if (resp.msg=="ACK_SET_METASTATE_NAME" and m_current_state==state_t::SEND_METASTATE_NAME) {
        RCLCPP_INFO(get_logger(), "Received: %s",resp.msg.c_str());
        m_ack_modules[resp.id]=1;
    }
    else if (resp.msg=="ACK_MOVE_TO_METASTATE" ) {
        RCLCPP_INFO(get_logger(), "Received: %s",resp.msg.c_str());
        m_ack_modules[resp.id]=1;
        std::set<std::string> enabled_sync_trans {resp.sync.begin(),resp.sync.end()};
        for (auto & elt : enabled_sync_trans) {
                	RCLCPP_INFO(get_logger(),"ACK_MOVE_TO_METASTAT received %s from %d\n",elt.c_str(),resp.id);
        }
        m_petri->getManageTransitionFusionSet()->enableSetFusion(enabled_sync_trans,resp.id);
    }


}

auto MasterNode::buildInitialMetaState() -> void {
    m_current_meta_state = m_petri->getMetaState(m_petri->getMarquage());
    m_metastate_building_name[m_petri->getPetriID()] = m_petri->getSCCName(m_current_meta_state->getInitialSCC());
    m_command.cmd = "GET_METASTATE";
    m_command_pub->publish(m_command);
}

auto MasterNode::computeEnabledSyncTransitions() -> void {
    auto enabled_sync_trans {m_petri->getSyncEnabled(m_current_meta_state)};
    for (auto & elt : enabled_sync_trans) {
        RCLCPP_INFO(get_logger(),"locally enabled sync %s\n",elt.c_str());
    }
    auto manageFusion {m_petri->getManageTransitionFusionSet()};
    //manageFusion->reset();
    manageFusion->enableSetFusion(enabled_sync_trans,m_petri->getPetriID());
}

auto MasterNode::statemachineMoveToState(const state_t state) -> void {
    if (m_ack_modules.all()) {
        m_ack_modules.reset();
        m_current_state = state;
    }
}

auto MasterNode::fireSyncTransition() -> bool {
     if (ml_enabled_fusion_sets.empty()) {
        RCLCPP_INFO(get_logger(),"No enabled fusion set...");
        return false;
     }
    // Pop a transition fusion set
    string transition {ml_enabled_fusion_sets[ml_enabled_fusion_sets.size()-1]};
    ml_enabled_fusion_sets.pop_back();
    RCLCPP_INFO(get_logger(),"Transition to sync fire: %s",transition.c_str());
    auto res = m_petri->fireSync(transition,m_current_meta_state);
    if (res.empty()) RCLCPP_INFO(get_logger(),"res: is empty");
    else RCLCPP_INFO(get_logger(),"res: is not empty");
    for (const auto & t : res) {
        RCLCPP_INFO(get_logger(),"Source metastate: %s",t.getSCCSource()->getMetaState()->toString().c_str());
        RCLCPP_INFO(get_logger(),"Transition nfusion name: %s",t.getTransition().c_str());
        RCLCPP_INFO(get_logger(),"Dest metastate %s",t.getDestSCC()->getMetaState()->toString().c_str());
    }
    for (uint32_t i {1};i<m_petri->getModulesCount();++i) {
        // Send request only to modules synced on transition
        if (m_petri->getManageTransitionFusionSet()->isFusionSetSyncedOnModule(transition,i)) {
            RCLCPP_INFO(get_logger(),"Received ");
            auto res {executeFireSyncTransitionRequest(i,transition)};
            for (auto e : res) {
                RCLCPP_INFO(get_logger(),"Received (%s,%s) ",e.source.c_str(),e.target.c_str());
            }
            RCLCPP_INFO(get_logger(),"\n");
        }
    }
    return true;
}

auto MasterNode::executeFireSyncTransitionRequest(const uint32_t id_server, const string &transition) -> std::vector<dss::firing_sync_t> {
    std::vector<dss::firing_sync_t> res;
    RCLCPP_INFO(get_logger(), "Execute fire sync service request...");
    static auto client_firing_service {create_client<ros2dss::FiringSyncTransitionSrv>("firing_sync_transitions_service"+std::to_string(id_server))};
    auto request {std::make_shared<ros2dss::FiringSyncTransitionSrv::Request>()};
    request->transition=transition;
    while (!client_firing_service->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting...");
            return res;
        }
    }
    RCLCPP_INFO(get_logger(), "Ok1....");
    auto future {client_firing_service->async_send_request(request)};
				/*,
                                                [this](rclcpp::Client<ros2dss::FiringSyncTransitionSrv>::SharedFuture response){
                                                  RCLCPP_INFO(this->get_logger(), "Service response: %ld", response.get()->lfiring.size());
                                                })};*/


    /*while (1) {
        if (future.wait_for(1s) == std::future_status::ready) {

            break;
        }
        RCLCPP_INFO(get_logger(), "Waiting for response...");
    }*/
	while (1) {
		m_executor->spin_some();
		if(future.wait_for(1s) == std::future_status::ready) {
            break;
        }
	}
	RCLCPP_INFO(get_logger(), "Ok2....");





//    if (spin_until_future_complete(result) == rclcpp::FutureReturnCode::SUCCESS) {
//        auto response {result.get()};
//        for (const auto & f : response->lfiring) {
//            res.push_back({f.source,f.target});
//        }
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Firing transition %s\n",transition.c_str());
//    } else {
//        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to fire transition %s\n",transition.c_str());
//    }
    RCLCPP_INFO(get_logger(), "Ok2....");
    return res;
}