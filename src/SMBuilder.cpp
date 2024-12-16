//
// Created by chiheb on 16/12/24.
//

#include "SMBuilder.h"


SMBuilder::SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> &publisher):m_petri(petri),m_publisher(publisher) {

}

void SMBuilder::run() {
    switch (m_current_state) {
        case state_t::INIT:
            m_publisher->publishCommand("START:"+std::to_string(m_petri->getNumero()));
            m_current_state=state_t::WAIT_FOR_ALL;
            break;
        case state_t::WAIT_FOR_ALL:
            break;
    }
}