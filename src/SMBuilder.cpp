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
}

void SMBuilder::run() {
    auto command {ros2dss_project::msg::Command{}};

    switch (m_current_state) {
        case state_t::INIT:
            if (m_publisher->getCommandSubCount()==m_petri->getModulesCount()) {
                m_current_state=state_t::WAIT_FOR_ALL;
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

            break;

        case state_t::WAIT_FOR_ALL:
            std::cout<<"WAIT_FOR_ALL: "<<m_petri->getModulesCount()<<"\n";

            break;
        case state_t::BUILD_LOCAL:
            std::cout<<"Build_local\n";
            break;
    }
}