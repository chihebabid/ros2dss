//
// Created by chiheb on 16/12/24.
//

#ifndef SMBUILER_H
#define SMBUILER_H
#include "DSSPublisher.h"
#include "misc.h"

class SMBuilder {
    enum class state_t {GET_SYNC_FUSION, INIT,BUILD_META_STATE,POP_METASTATE,COMPUTE_SYNC, FIRE_SYNC,TERMINATE_BUILDING};
public:
    SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> &publisher);
    ~SMBuilder() = default;
    void run();
private:
    dss::PetriNet *m_petri;
    state_t m_current_state {state_t::INIT};
    std::shared_ptr<DSSPublisher> m_publisher;
    dss::ModuleSS  *m_module_ss {};
    dss::MetaState* m_current_meta_state {};
    std::stack<dss::MetaState*> m_meta_states_stack;
    std::vector<std::string> ml_enabled_fusion_sets;
};



#endif //SMBUILER_H
