//
// Created by chiheb on 16/12/24.
//

#include "SMBuilder.h"
#include "ros2dss_project/msg/command.hpp"
#include "gmisc.h"

SMBuilder::SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> &publisher):m_petri(petri),m_publisher(publisher) {

}

void SMBuilder::run() {
    auto command {ros2dss_project::msg::Command{}};
    switch (m_current_state) {
        case state_t::INIT:
            command.cmd="INIT";
            command.param=m_petri->getPetriID();
            m_publisher->publishCommand(command);
            _ptr_modules=std::make_unique<dss::MarkingArray>(m_petri->getModulesCount());
            (*_ptr_modules)[m_petri->getPetriID()]=1;
            m_current_state=state_t::WAIT_FOR_ALL;
            break;

        case state_t::WAIT_FOR_ALL:
            m_current_state=state_t::BUILD_LOCAL;
            for (uint32_t i{};i<m_petri->getModulesCount();++i) {
                if ((*_ptr_modules)[i]!=1) {
                    m_current_state=state_t::WAIT_FOR_ALL;
                    break;
                }
            }
            break;
        case state_t::BUILD_LOCAL:
            break;
    }
}