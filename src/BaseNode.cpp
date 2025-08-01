//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"


BaseNode::BaseNode(dss::PetriNet  *petri,const string &name):Node(name),m_petri(petri) {
    m_module_ss = new dss::ModuleSS(m_petri->getModulesCount());
}


auto BaseNode::shouldShutdown() const -> bool {
    return m_should_shutdown;
}

auto BaseNode::requestShutdown()  -> void {
    m_should_shutdown = true;
}