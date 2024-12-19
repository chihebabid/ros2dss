//
// Created by chiheb on 16/12/24.
//

#ifndef SMBUILER_H
#define SMBUILER_H
#include "DSSPublisher.h"
#include "misc.h"

class SMBuilder {
    enum class state_t {INIT,BUILD_META_STATE,BUILD_LOCAL};
public:
    SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> &publisher);
    ~SMBuilder() = default;
    void run();
private:
    dss::PetriNet *m_petri;
    state_t m_current_state {state_t::INIT};
    std::shared_ptr<DSSPublisher> m_publisher;

};



#endif //SMBUILER_H
