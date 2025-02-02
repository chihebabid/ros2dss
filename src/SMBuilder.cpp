//
// Created by chiheb on 16/12/24.
//
#include "gmisc.h"
#include "SMBuilder.h"
#include "ros2dss_project/msg/command.hpp"
#include "ros2dss_project/msg/firing.hpp"


SMBuilder::SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> publisher,std::shared_ptr<FiringSyncTransitionService> firing_service): m_petri(petri),
    m_publisher(publisher),m_firing_sync_transition_service(firing_service) {
    m_firing_sync_transition_service->setSMBuilder(this);
    _ptr_modules = std::make_unique<dss::MarkingArray>(m_petri->getModulesCount());
    for (uint32_t i{}; i < m_petri->getModulesCount(); ++i) {
        (*_ptr_modules)[i] = 0;
    }
    (*_ptr_modules)[m_petri->getPetriID()] = 1;
    m_module_ss = new dss::ModuleSS(m_petri->getModulesCount());

}

void SMBuilder::run() {

    switch (m_current_state) {
        case state_t::GET_SYNC_FUSION :
            m_current_state=state_t::INIT;
            break;

        case state_t::INIT:
            if (m_publisher->getCommandSubscribersCount() == m_petri->getModulesCount()) {
                m_current_state = state_t::BUILD_INITIAL_META_STATE;
                m_command.cmd = "INIT";
                m_command.param = m_petri->getPetriID();
                m_publisher->publishCommand(m_command);
                for (uint32_t i{}; i < m_petri->getModulesCount(); ++i) {
                    if ((*_ptr_modules)[i] != 1) {
                        m_current_state = state_t::INIT;
                        break;
                    }
                }
            }
            RCLCPP_INFO(m_publisher->get_logger(), "INIT STATE\n");
            break;

        case state_t::BUILD_INITIAL_META_STATE:
            buildInitialMetaState();
            m_current_state=state_t::BUILD_META_STATE;
            break;

        case state_t::BUILD_META_STATE:
            m_command.cmd = "METASTATE";
            m_command.param = m_petri->getPetriID();
            m_command.scc = (*_ptr_metastate_name)[m_petri->getPetriID()];
            m_publisher->publishCommand(m_command);
            m_current_state = state_t::POP_METASTATE;
            for (size_t i{}; i < _ptr_metastate_name->size(); ++i) {
                if (_ptr_metastate_name->operator[](i).empty()) {
                    m_current_state = state_t::BUILD_META_STATE;
                    break;
                }
            }
            RCLCPP_INFO(m_publisher->get_logger(), "BUILD_META_STATE : %d\n", m_petri->getModulesCount());
            if (m_current_state == state_t::POP_METASTATE) {
                m_current_meta_state->setName(*_ptr_metastate_name);
                if (m_module_ss->insertMS(m_current_meta_state)) {
                    if (!m_petri->getPetriID()) {
                        m_meta_states_stack.push(m_current_meta_state);
                    }
                }

            }
            break;

        case state_t::POP_METASTATE: // Pop a metastate from the stack
            if (m_petri->getPetriID() == 0) {
                if (m_meta_states_stack.empty()) {
                    m_current_state = state_t::TERMINATE_BUILDING;
                } else {
                    m_current_meta_state = m_meta_states_stack.top();
                    m_meta_states_stack.pop();
                    m_command.cmd = "MOVE_TO_METASTATE";
                    m_command.scc = m_current_meta_state->toString();
                    m_publisher->publishCommand(m_command);
                    RCLCPP_INFO(m_publisher->get_logger(), "POP_METASTATE\n");
                    m_current_state = state_t::PREPARE_COMPUTE_SYNC;
                }
            } else {
                if (!_current_meta_state_name.empty()) {
                    m_current_meta_state = m_module_ss->findMetaState(_current_meta_state_name);
                    m_current_state = state_t::PREPARE_COMPUTE_SYNC;
                }
            }
            RCLCPP_INFO(m_publisher->get_logger(), "#Metastates %ld\nListe of metastates\n",
                        m_module_ss->getMetaStateCount());
            for (size_t i{}; i < m_module_ss->getMetaStateCount(); ++i) {
                RCLCPP_INFO(m_publisher->get_logger(), "#Metastates name %s\n",
                            m_module_ss->getMetaState(i)->toString().c_str());
            }
            break;

        case state_t::PREPARE_COMPUTE_SYNC:
            computeEnabledSyncTransitions();
            m_current_state=state_t::COMPUTE_SYNC;
            break ;

        case state_t::COMPUTE_SYNC: // Determine enabled fusion sets
            if (m_petri->getPetriID()==0  && _received_sync_count==m_petri->getModulesCount()-1) {
                auto manage {m_petri->getManageTransitionFusionSet()};
                ml_enabled_fusion_sets=manage->getEnabledFusionSets();
                for (auto & elt : ml_enabled_fusion_sets) {
                    RCLCPP_INFO(m_publisher->get_logger(),"enabled fusion %s\n",elt.c_str());
                }

                m_current_state=state_t::FIRE_SYNC;
                manage->display();
            }
            RCLCPP_INFO(m_publisher->get_logger(),"Current state : COMPUTE_SYNC %d\n",_received_sync_count.load());
            break;

        case state_t::FIRE_SYNC:
            if (m_petri->getPetriID()==0) {
               if (!fireSyncTransition()) {
                     // No more fusion set is enabled
               }
               else {
                   // There are still non processed fusion sets
               }

            }
            m_current_state=state_t::TERMINATE_BUILDING;
            break;

        case state_t::TERMINATE_BUILDING:
            RCLCPP_INFO(m_publisher->get_logger(), "TERMINATE_BUILDING\n");
            break;
    }
}

void SMBuilder::buildInitialMetaState() {
    m_current_meta_state = m_petri->getMetaState(m_petri->getMarquage());
    _ptr_metastate_name = std::make_unique<dss::ArrayModel<std::string> >(m_petri->getModulesCount());
    _ptr_metastate_name->operator[](m_petri->getPetriID()) = m_petri->getSCCName(
        m_current_meta_state->getInitialSCC());
}

void SMBuilder::computeEnabledSyncTransitions() {
    auto enabled_sync_trans {m_petri->getSyncEnabled(m_current_meta_state)};
    auto manageFusion {m_petri->getManageTransitionFusionSet()};
    if (m_petri->getPetriID()==0) {
        manageFusion->reset();
        manageFusion->enableSetFusion(enabled_sync_trans,m_petri->getPetriID());
    }
    else {
        m_command.cmd = "SYNC_TRANSITION";
        m_command.param=m_petri->getPetriID();
        std::vector <std::string> _vec {enabled_sync_trans.begin(),enabled_sync_trans.end()};
        m_command.sync=_vec;
        RCLCPP_INFO(m_publisher->get_logger(), "COMPUTE_SYNC: Send enabled sync transitions");
        m_publisher->publishCommand(m_command);
    }
}

bool SMBuilder::fireSyncTransition() {
     if (ml_enabled_fusion_sets.empty()) {
        RCLCPP_INFO(m_publisher->get_logger(),"No enabled fusion set...");
        return false;
     }
    // Participate in the fusion
    string transition {ml_enabled_fusion_sets[ml_enabled_fusion_sets.size()-1]};
    ml_enabled_fusion_sets.pop_back();
    RCLCPP_INFO(m_publisher->get_logger(),"Transition to sync fire: %s",transition.c_str());
    auto res = m_petri->fireSync(transition,m_current_meta_state);
    if (res.empty()) RCLCPP_INFO(m_publisher->get_logger(),"res: is empty");
    else RCLCPP_INFO(m_publisher->get_logger(),"res: is not empty");
    for (const auto & t : res) {
        RCLCPP_INFO(m_publisher->get_logger(),"Source metastate: %s",t.getSCCSource()->getMetaState()->toString().c_str());
        RCLCPP_INFO(m_publisher->get_logger(),"Transition nfusion name: %s",t.getTransition().c_str());
        RCLCPP_INFO(m_publisher->get_logger(),"Dest metastate %s",t.getDestSCC()->getMetaState()->toString().c_str());
    }
    for (uint32_t i {1};i<m_petri->getModulesCount();++i) {
        // Send request only to modules synced on transition
        if (m_petri->getManageTransitionFusionSet()->isFusionSetSyncedOnModule(transition,i)) {
            RCLCPP_INFO(m_publisher->get_logger(),"Received ");
            auto res {m_firing_sync_transition_service->executeRequest(i,transition)};
            for (auto e : res) {
                RCLCPP_INFO(m_publisher->get_logger(),"Received (%s,%s) ",e.source.c_str(),e.target.c_str());
            }
            RCLCPP_INFO(m_publisher->get_logger(),"\n");
        }
    }
    return true;
}
