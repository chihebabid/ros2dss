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

    SCC* FiringSyncTransition::getDestSCC() const {
        return m_dest;
    }

    bool FiringSyncTransition::operator<(const FiringSyncTransition &o) const {
      if (!(*m_source==*(o.m_source) and *m_dest==*(o.m_dest) and m_transition==o.m_transition)) {
        return m_source<o.m_source or m_dest<o.m_dest;
      }
      return false;
    }

    SCC * FiringSyncTransition::getSCCSource() const {
        return m_source;
    }

    std::string FiringSyncTransition::getTransition() const {
        return m_transition;
    }

    FiringSyncTransition::operator firing_sync_t() const {
        // TODO: Implement the conversion
        return {"ok1","ok2"};
    }
}