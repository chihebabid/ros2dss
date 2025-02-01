//
// Created by chiheb on 01/02/25.
//

#ifndef FIRINGSYNCTRANSITIONSERVICE_H
#define FIRINGSYNCTRANSITIONSERVICE_H
class FiringSyncTransitionService : public rclcpp::Node {
public:
    FiringSyncTransitionService(dss::PetriNet  *);


private:
    void firingSyncTransitionsService(const std::shared_ptr<ros2dss::FiringSyncTransition::Request>,std::shared_ptr<ros2dss::FiringSyncTransition::Response> );
    bool m_should_shutdown {false};
    dss::PetriNet *m_petri;
    rclcpp::Service<ros2dss::FiringSyncTransition>::SharedPtr m_server_firing_service {};
    rclcpp::Client<ros2dss::FiringSyncTransition>::SharedPtr m_client_firing_service {};

};
#endif //FIRINGSYNCTRANSITIONSERVICE_H
