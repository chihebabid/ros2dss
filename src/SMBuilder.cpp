//
// Created by chiheb on 16/12/24.
//

#include "SMBuilder.h"
#include "ros2dss_project/msg/command.hpp"
#include "gmisc.h"

SMBuilder::SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> &publisher): m_petri(petri),
    m_publisher(publisher) {
    _ptr_modules = std::make_unique<dss::MarkingArray>(m_petri->getModulesCount());
    for (uint32_t i{}; i < m_petri->getModulesCount(); ++i) {
        (*_ptr_modules)[i] = 0;
    }
    (*_ptr_modules)[m_petri->getPetriID()] = 1;
    m_module_ss = new dss::ModuleSS(m_petri->getModulesCount());
}

void SMBuilder::run() {
    auto command{ros2dss_project::msg::Command{}};
    static bool once_execution{false};

    switch (m_current_state) {
        case state_t::GET_SYNC_FUSION :
            m_current_state=state_t::INIT;
            break;
        case state_t::INIT:
            if (m_publisher->getCommandSubscribersCount() == m_petri->getModulesCount()) {
                m_current_state = state_t::BUILD_META_STATE;
                command.cmd = "INIT";
                command.param = m_petri->getPetriID();
                m_publisher->publishCommand(command);
                for (uint32_t i{}; i < m_petri->getModulesCount(); ++i) {
                    // std::cout<<static_cast<int>((*_ptr_modules)[i])<<" ";
                    if ((*_ptr_modules)[i] != 1) {
                        m_current_state = state_t::INIT;
                        break;
                    }
                }
            }
            RCLCPP_INFO(m_publisher->get_logger(), "INIT STATE\n");
            break;

        case state_t::BUILD_META_STATE:
            if (!once_execution) {
                RCLCPP_INFO(m_publisher->get_logger(), "Enter\n");
                once_execution = true;
                m_current_meta_state = m_petri->getMetaState(m_petri->getMarquage());
                _ptr_metastate_name = std::make_unique<dss::ArrayModel<std::string> >(m_petri->getModulesCount());
                _ptr_metastate_name->operator[](m_petri->getPetriID()) = m_petri->getSCCName(
                    m_current_meta_state->getInitialSCC());
            }
            command.cmd = "METASTATE";
            command.param = m_petri->getPetriID();
            command.scc = (*_ptr_metastate_name)[m_petri->getPetriID()];
            m_publisher->publishCommand(command);
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
                once_execution = false;
            }
            break;

        case state_t::POP_METASTATE:
            if (m_petri->getPetriID() == 0) {
                if (m_meta_states_stack.empty()) {
                    m_current_state = state_t::TERMINATE_BUILDING;
                } else {
                    m_current_meta_state = m_meta_states_stack.top();
                    m_meta_states_stack.pop();
                    command.cmd = "MOVE_TO_METASTATE";
                    command.scc = m_current_meta_state->toString();
                    m_publisher->publishCommand(command);
                    RCLCPP_INFO(m_publisher->get_logger(), "POP_METASTATE\n");
                    m_current_state = state_t::COMPUTE_SYNC;
                }
            } else {
                if (!_current_meta_state_name.empty()) {
                    m_current_meta_state = m_module_ss->findMetaState(_current_meta_state_name);
                    m_current_state = state_t::COMPUTE_SYNC;
                }
            }
            RCLCPP_INFO(m_publisher->get_logger(), "#Metastates %ld\nListe of metastates\n",
                        m_module_ss->getMetaStateCount());
            for (size_t i{}; i < m_module_ss->getMetaStateCount(); ++i) {
                RCLCPP_INFO(m_publisher->get_logger(), "#Metastates name %s\n",
                            m_module_ss->getMetaState(i)->toString().c_str());
            }
            break;

        case state_t::COMPUTE_SYNC: // Determine enabled sync transitions
            if (!once_execution) {
                once_execution=true;
                auto enabled_sync_trans {m_petri->getSyncEnabled(m_current_meta_state)};
                auto manageFusion {m_petri->getManageTransitionFusionSet()};
                if (m_petri->getPetriID()==0) {
                    manageFusion->reset();
                    manageFusion->enableSetFusion(enabled_sync_trans,m_petri->getPetriID());
                }
                else {
                    command.cmd = "SYNC_TRANSITION";
                    command.param=m_petri->getPetriID();
                    std::vector <std::string> _vec {enabled_sync_trans.begin(),enabled_sync_trans.end()};
                    command.sync=_vec;
                    RCLCPP_INFO(m_publisher->get_logger(), "COMPUTE_SYNC: Send enabled sync transitions");
                    m_publisher->publishCommand(command);
                    state_t::TERMINATE_BUILDING;
                }
            }
            if (m_petri->getPetriID()==0  && _received_sync_count==m_petri->getModulesCount()-1) {
                m_current_state=state_t::TERMINATE_BUILDING;
            }
            RCLCPP_INFO(m_publisher->get_logger(),"Current state : COMPUTE_SYNC %d\n",_received_sync_count);
            break;

        case state_t::TERMINATE_BUILDING:
            if (m_petri->getPetriID()==0) {
                auto manage {m_petri->getManageTransitionFusionSet()};
                auto enabled_sync_trans {manage->getEnabledFusionSets()};
                for (const auto & elt : enabled_sync_trans) {
                    RCLCPP_INFO(m_publisher->get_logger(), "Enabled sync transition : %s\n", elt.c_str());
                }
            }
            break;
    }
}
