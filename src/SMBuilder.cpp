//
// Created by chiheb on 16/12/24.
//

#include "SMBuilder.h"
#include "ros2dss_project/msg/command.hpp"

SMBuilder::SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> &publisher):m_petri(petri),m_publisher(publisher) {

}

void SMBuilder::run() {
    auto command {ros2dss_project::msg::Command{}};
    switch (m_current_state) {
        case state_t::INIT:
            //m_publisher->publishCommand("START:"+std::to_string(m_petri->getNumero()));
            command.cmd="INIT";
            command.param=m_petri->getNumero();
            m_publisher->publishCommand(command);
            m_current_state=state_t::WAIT_FOR_ALL;
            break;

        case state_t::WAIT_FOR_ALL:
            break;
    }
}