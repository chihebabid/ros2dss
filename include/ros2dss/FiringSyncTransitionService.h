//
// Created by chiheb on 01/02/25.
//

#ifndef FIRINGSYNCTRANSITIONSERVICE_H
#define FIRINGSYNCTRANSITIONSERVICE_H
class SMBuilder;
class FiringSyncTransitionService : public rclcpp::Node {
public:
    FiringSyncTransitionService(dss::PetriNet  *);
    void executeRequest(const uint32_t id_server,const string &transition);
    void setSMBuilder(SMBuilder *);
private:
    void firingSyncTransitionsService(const std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Request>,std::shared_ptr<ros2dss::FiringSyncTransitionSrv::Response> );
    bool m_should_shutdown {false};
    dss::PetriNet *m_petri;
    rclcpp::Service<ros2dss::FiringSyncTransitionSrv>::SharedPtr m_server_firing_service {};
    SMBuilder *m_sm_builder {};
};
#endif //FIRINGSYNCTRANSITIONSERVICE_H
