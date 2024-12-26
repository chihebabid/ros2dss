//
// Created by chiheb on 27/12/24.
//

#include "misc.h"

namespace dss {
    FiringSyncTransition::FiringSyncTransition(SCC* source,const std::string &transition,SCC *dest):m_source(source),m_transition(transition),m_dest(dest) {
    }

    bool FiringSyncTransition::operator==(const FiringSyncTransition &o) const {
        return *m_source==*(o.m_source) && *m_dest==*(o.m_dest) && m_transition==o.m_transition;
    }
}