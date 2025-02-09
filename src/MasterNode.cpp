//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

MasterNode::MasterNode(dss::PetriNet  *petri,std::shared_ptr<FiringSyncTransitionService> firing_service):BaseNode(petri,"dss_master"),
m_firing_sync_transition_service(firing_service),m_ack_modules(petri->getModulesCount()),m_metastate_building_name(petri->getModulesCount()) {
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
                m_ack_modules.reset();
            	m_current_state = state_t::SEND_METASTATE_NAME;
            }
            break;
        case state_t::SEND_METASTATE_NAME:
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
    else if (resp.msg=="ACK_MOVE_TO_METASTATE"  and m_current_state==state_t::PREPARE_COMPUTE_SYNC) {
        RCLCPP_INFO(get_logger(), "Received: %s",resp.msg.c_str());
        m_ack_modules[resp.id]=1;
        std::set<std::string> enabled_sync_trans {resp.sync.begin(),resp.sync.end()};
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
    auto manageFusion {m_petri->getManageTransitionFusionSet()};
    manageFusion->reset();
    manageFusion->enableSetFusion(enabled_sync_trans,m_petri->getPetriID());
}

auto MasterNode::statemachineMoveToState(const state_t state) -> void {
    if (m_ack_modules.all()) {
        m_ack_modules.reset();
        m_current_state = state;
    }
}