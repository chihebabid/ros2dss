//
// Created by chiheb on 01/02/25.
//

#ifndef FIRINGSYNCTRANSITIONSERVICE_H
#define FIRINGSYNCTRANSITIONSERVICE_H
#include "gmisc.h"
class SlaveNode;
class FiringSyncTransitionService : public rclcpp::Node {
public:
    FiringSyncTransitionService(dss::PetriNet  *);
    void setNode(SlaveNode *);
    const std::set<dss::FiringSyncTransition>& getFiringSyncTransitions() const;
    void cleanSCCs();
private:
    void firingSyncTransitionsService(const std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Request>,std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Response> );
    bool m_should_shutdown {false};
    dss::PetriNet *m_petri;
    rclcpp::Service<ros2dss::FiringSyncTransitionSrv>::SharedPtr m_server_firing_service {};
    SlaveNode *m_slave_node {};
    std::set<dss::FiringSyncTransition> ml_firing_sync_transitions;
};
#endif //FIRINGSYNCTRANSITIONSERVICE_H
