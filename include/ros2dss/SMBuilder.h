//
// Created by chiheb on 16/12/24.
//

#ifndef SMBUILER_H
#define SMBUILER_H
#include "DSSPublisher.h"
#include "misc.h"
#include "ros2dss_project/srv/firing_sync_transition.hpp"
class SMBuilder {
    enum class state_t {GET_SYNC_FUSION, INIT,BUILD_INITIAL_META_STATE,BUILD_META_STATE,POP_METASTATE,PREPARE_COMPUTE_SYNC,COMPUTE_SYNC, FIRE_SYNC,TERMINATE_BUILDING};
public:
    SMBuilder(dss::PetriNet *petri, std::shared_ptr<DSSPublisher> publisher,std::shared_ptr<FiringSyncTransitionService>);
    ~SMBuilder() = default;
    void run();
    dss::MetaState* getCurrentMetaState() const {return m_current_meta_state;}
private:
    dss::PetriNet *m_petri;
    state_t m_current_state {state_t::INIT};
    std::shared_ptr<DSSPublisher> m_publisher;
    std::shared_ptr<FiringSyncTransitionService> m_firing_sync_transition_service;
    dss::ModuleSS  *m_module_ss {};
    dss::MetaState* m_current_meta_state {};
    std::stack<dss::MetaState*> m_meta_states_stack;
    std::vector<std::string> ml_enabled_fusion_sets;
    ros2dss_project::msg::Command m_command;

    void buildInitialMetaState();
    void computeEnabledSyncTransitions();
    bool fireSyncTransition();
};



#endif //SMBUILER_H
