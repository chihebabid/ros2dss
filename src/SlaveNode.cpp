//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

SlaveNode::SlaveNode(dss::PetriNet  *petri,std::shared_ptr<FiringSyncTransitionService> firing_service):BaseNode(petri, "dss_slave"),m_firing_sync_transition_service(firing_service) {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Create 'dss/response' topic
    m_response_pub=create_publisher<ros2dss::Response>("dss/response",qos);

    // Subscribe to 'dss/command' topic
    m_command_sub = this->create_subscription<ros2dss::Command>(
            "/dss/command", qos, [this](const ros2dss::Command & msg) {
                command_receiver(msg);
            });

    m_firing_sync_transition_service->setNode(this);
}


auto SlaveNode::run() -> void {
}

auto SlaveNode::command_receiver(const ros2dss::Command & msg) -> void {
  	RCLCPP_INFO(get_logger(), "Received: %s",msg.cmd.c_str());
    if (msg.cmd=="INIT") {
        m_response.msg="ACK";
        m_response.id=m_petri->getPetriID();
        m_response_pub->publish(m_response);
    }
    else if (msg.cmd=="GET_METASTATE") {
      	m_current_meta_state = m_petri->getMetaState(m_petri->getMarquage());
        m_response.msg="ACK_GET_METASTATE";
        m_response.id=m_petri->getPetriID();
        m_response.scc=m_petri->getSCCName(m_current_meta_state->getInitialSCC());
        m_response_pub->publish(m_response);
        RCLCPP_INFO(get_logger(), "Send ACK_GET_METASTATE: %s",m_response.scc.c_str());
    }
    else if (msg.cmd=="SET_METASTATE_NAME") {
        m_current_meta_state->setName(msg.sync);
        m_module_ss->insertMS(m_current_meta_state);
        m_response.msg="ACK_SET_METASTATE_NAME";
        m_response.id=m_petri->getPetriID();
        m_response_pub->publish(m_response);
        RCLCPP_INFO(get_logger(), "Send ACK_SET_METASTATE_NAME");
    }
    else if (msg.cmd=="MOVE_TO_METASTATE") {
        m_firing_sync_transition_service->cleanSCCs();
        m_current_meta_state = m_module_ss->findMetaState(msg.scc);
        if (!m_current_meta_state) {
            RCLCPP_ERROR(get_logger(), "Didn\'t find requested metastate %s !",msg.scc.c_str());
        }
        else {
            auto enabled_sync_trans {m_petri->getSyncEnabled(m_current_meta_state)};
            /*for (auto & elt : enabled_sync_trans) {
        		RCLCPP_INFO(get_logger(),"locally enabled sync %s\n",elt.c_str());
    		}*/
            m_response.msg = "ACK_MOVE_TO_METASTATE";
            m_response.id=m_petri->getPetriID();
            std::vector <std::string> _vec {enabled_sync_trans.begin(),enabled_sync_trans.end()};
            m_response.sync=std::move(_vec);
            m_response_pub->publish(m_response);
        }
    }
    else if (msg.cmd=="ADD_NEW_METASTATE") {

        //msg.target_ms
        RCLCPP_INFO(get_logger(), "Received command to add new metastate: %s ===%s==> %s", dss::vectorToStdString(msg.source_product).c_str(),msg.transition.c_str(),dss::vectorToStdString(msg.target_ms).c_str());
        std::string _chaine;
        /*for (const auto e: m_firing_sync_transition_service->getFiringSyncTransitions()) {
            _chaine+="("+e.getSCCSource()->getName(m_petri)+","+e.getTransition()+","+e.getDestSCC()->getName(m_petri)+") ";
        }
        RCLCPP_WARN(get_logger(), "Set of firings: %s",_chaine.c_str());*/
        auto& scc_name {msg.target_ms[m_petri->getPetriID()]};
        RCLCPP_INFO(get_logger(), "Searching for SCC: %s",scc_name.c_str());
        auto find_ms {std::find_if(m_firing_sync_transition_service->getFiringSyncTransitions().begin(),
                                 m_firing_sync_transition_service->getFiringSyncTransitions().end(),
                                 [this,&scc_name](const dss::FiringSyncTransition & elt)-> bool {
                                     RCLCPP_INFO(get_logger(), "item for SCC: %s",elt.getDestSCC()->getName(m_petri).c_str());
                                     if (elt.getDestSCC()->getName(m_petri)==scc_name) {
                                         return true;
                                     }
                                     return false;
                                 })};

        // Check if the transition is found (the module is synchronized on it)
        if (m_petri->getTransitionPtr(msg.transition)) {

            if (find_ms!=m_firing_sync_transition_service->getFiringSyncTransitions().end()) {
                //RCLCPP_INFO(get_logger(), "The module is synchronized on transition %s",msg.transition.c_str());
                dss::Marking *p_marking {find_ms->getDestSCC()->getMetaState()->getInitialMarking()};
                dss::MetaState* new_ms {m_petri->getMetaState(*p_marking)};
                new_ms->setName(msg.target_ms);
                m_module_ss->insertMS(new_ms);
            }
            else {
                RCLCPP_ERROR(get_logger(), "Destination SCC not found %s",scc_name.c_str());
            }
        }
        else {
            // RCLCPP_INFO(get_logger(), "The module is not synchronized on transition %s",msg.transition.c_str());
            dss::Marking *p_marking {m_current_meta_state->getInitialMarking()};
            dss::MetaState* new_ms {m_petri->getMetaState(*p_marking)};
            new_ms->setName(msg.target_ms);
            m_module_ss->insertMS(new_ms);
        }
    }
}

auto SlaveNode::getCurrentMetaState() const -> dss::MetaState * {
    return m_current_meta_state;
}