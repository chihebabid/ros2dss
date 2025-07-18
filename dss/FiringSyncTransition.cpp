//
// Created by chiheb on 27/12/24.
//

#include "misc.h"

namespace dss {
    FiringSyncTransition::FiringSyncTransition(SCC *source, const std::string &transition, SCC *dest): m_source(source),
        m_transition(transition), m_dest(dest) {
    }

    bool FiringSyncTransition::operator==(const FiringSyncTransition &o) const {
        return *m_source == *(o.m_source) && *m_dest == *(o.m_dest) && m_transition == o.m_transition;
    }

    SCC *FiringSyncTransition::getDestSCC() const {
        return m_dest;
    }

    bool FiringSyncTransition::operator<(const FiringSyncTransition &o) const {
        if (!(*m_source == *(o.m_source) and *m_dest == *(o.m_dest) and m_transition == o.m_transition)) {
            if (m_source < o.m_source) return true;
            if (m_source > o.m_source) return false;
            if (m_dest < o.m_dest) return true;
            if (m_dest > o.m_dest) return false;
            return m_transition < o.m_transition;
        }
        return false;
    }

    SCC *FiringSyncTransition::getSCCSource() const {
        return m_source;
    }

    std::string FiringSyncTransition::getTransition() const {
        return m_transition;
    }
}
