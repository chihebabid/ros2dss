//
// Created by chiheb on 16/12/24.
//

#include "SMBuilder.h"
#include "ros2dss_project/msg/command.hpp"
#include "gmisc.h"

SMBuilder::SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> &publisher):m_petri(petri),m_publisher(publisher) {
    _ptr_modules=std::make_unique<dss::MarkingArray>(m_petri->getModulesCount());
    for (uint32_t i{};i<m_petri->getModulesCount();++i) {
        (*_ptr_modules)[i]=0;
    }
    (*_ptr_modules)[m_petri->getPetriID()-1]=1;
    m_module_ss=new dss::ModuleSS(m_petri->getPetriID()-1);
}

void SMBuilder::run() {
    auto command {ros2dss_project::msg::Command{}};
    static bool once_execution {false};
    dss::MetaState *ptrMS {};
    switch (m_current_state) {
        case state_t::INIT:
            if (m_publisher->getCommandSubCount()==m_petri->getModulesCount()) {
                m_current_state=state_t::BUILD_META_STATE;
                command.cmd="INIT";
                command.param=m_petri->getPetriID();
                m_publisher->publishCommand(command);
                for (uint32_t i{};i<m_petri->getModulesCount();++i) {
                    // std::cout<<static_cast<int>((*_ptr_modules)[i])<<" ";
                    if ((*_ptr_modules)[i]!=1) {
                        m_current_state=state_t::INIT;
                        break;
                    }
                }
            }
            RCLCPP_INFO(m_publisher->get_logger(), "INIT STATE\n");
            break;
        case state_t::BUILD_META_STATE:
            if (!once_execution) {
                RCLCPP_INFO(m_publisher->get_logger(),"Enter\n");
                once_execution=true;
                ptrMS=m_petri->getMetaState(m_petri->getMarquage());
                _ptr_metastate_name=std::make_unique<dss::ArrayModel<std::string>>(m_petri->getModulesCount());
                _ptr_metastate_name->operator[](m_petri->getPetriID()-1)=m_petri->getSCCName(ptrMS->getInitialSCC());

            }
            command.cmd="METASTATE";
            command.param=m_petri->getPetriID();
            command.scc=(*_ptr_metastate_name)[m_petri->getPetriID()];
            m_publisher->publishCommand(command);
            m_current_state=state_t::COMPUTE_ENABLED_SYNC;
            for (size_t i{};i<_ptr_metastate_name->size();++i) {
                if (_ptr_metastate_name->operator[](i).empty()) {
                    m_current_state=state_t::BUILD_META_STATE;
                    break;
                }
            }
            RCLCPP_INFO(m_publisher->get_logger(), "BUILD_META_STATE : %d\n",m_petri->getModulesCount());

            break;
        case state_t::COMPUTE_ENABLED_SYNC:
            std::cout<<"Build_local\n";
            break;
    }
}