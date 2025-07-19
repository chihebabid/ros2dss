//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"

MasterNode::MasterNode(dss::PetriNet *petri,
                       std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
                       _executor): BaseNode(petri, "dss_master"), m_executor(_executor),
                                   m_ack_modules(petri->getModulesCount()),
                                   m_metastate_building_name(petri->getModulesCount()) {
    rclcpp::QoS qos(rclcpp::KeepLast(petri->getModulesCount()));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Create 'dss/command' topic
    m_command_pub = create_publisher<ros2dss::Command>("dss/command", qos);

    // Subscribe to 'dss/response' topic
    m_response_sub = this->create_subscription<ros2dss::Response>(
        "/dss/response", qos, [this](const ros2dss::Response &msg) {
            response_receiver(msg);
        });
    m_ack_modules.reset();
    m_ack_modules[m_petri->getPetriID()] = 1;
    RCLCPP_INFO(get_logger(), "Modules count: %d", m_petri->getModulesCount());

    ml_clients_firing_info.resize(m_petri->getModulesCount());
    for (size_t i {1}; i < m_petri->getModulesCount(); ++i) {
        std::string client_name = "adding_info_sync_service" + std::to_string(i);
        ml_clients_firing_info[i] = create_client<ros2dss::InfoFiring>(client_name);
        RCLCPP_INFO(get_logger(), "Created client for firing info: %s", client_name.c_str());
        while (!ml_clients_firing_info[i]->wait_for_service(10ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Service Info: interrupted while waiting...");
                // return res;
            }
        }
    }
}


auto MasterNode::run() -> void {
    switch (m_current_state) {
        case state_t::GET_SYNC_FUSION:
            RCLCPP_INFO(get_logger(), "Current SM: GET_SYNC_FUSION");
            m_current_state = state_t::INIT;
            break;

        case state_t::INIT:
            RCLCPP_INFO(get_logger(), "Current SM: INIT");
            if (m_command_pub->get_subscription_count() == m_petri->getModulesCount() - 1) {
                m_command.cmd = "INIT";
                m_command_pub->publish(m_command);
                statemachineMoveToState(state_t::BUILD_INITIAL_META_STATE);
            }
            break;

        case state_t::BUILD_INITIAL_META_STATE:
            RCLCPP_INFO(get_logger(), "Current SM: BUILD_INITIAL_META_STATE");
            buildInitialMetaState();
            m_current_state = state_t::BUILD_META_STATE;
            break;

        case state_t::BUILD_META_STATE:
            RCLCPP_INFO(get_logger(), "Current SM: BUILD_META_STATE");
            if (m_ack_modules.all()) {
                m_current_meta_state->setName(m_metastate_building_name);
                RCLCPP_INFO(get_logger(), "Built Metastate: %s", m_current_meta_state->toString().c_str());
                m_module_ss->insertMS(m_current_meta_state);
                m_meta_states_stack.push(m_current_meta_state);
                m_current_state = state_t::SEND_METASTATE_NAME;
            }
            break;

        case state_t::SEND_METASTATE_NAME:
            RCLCPP_INFO(get_logger(), "Current SM: SEND_METASTATE_NAME");
            m_command.cmd = "SET_METASTATE_NAME";
            m_command.sync = m_metastate_building_name;
            m_command_pub->publish(m_command);
            statemachineMoveToState(state_t::POP_METASTATE);
            break;

        case state_t::POP_METASTATE:
            RCLCPP_INFO(get_logger(), "POP_METASTATE: Stack size: %ld",m_meta_states_stack.size());
            if (m_meta_states_stack.empty()) {
                m_current_state = state_t::TERMINATE_BUILDING;
            } else {
                m_current_meta_state = m_meta_states_stack.top();
                RCLCPP_INFO(get_logger(), "POP_METASTATE: Popped MS: %s",m_current_meta_state->toString().c_str());
                m_meta_states_stack.pop();
                auto manageFusion{m_petri->getManageTransitionFusionSet()};
                manageFusion->reset();
                m_command.cmd = "MOVE_TO_METASTATE";
                m_command.scc = m_current_meta_state->toString();
                m_command.transition="";
                m_command.target_ms.clear();
                m_command.source_product.clear();
                m_command_pub->publish(m_command);
                m_current_state = state_t::PREPARE_COMPUTE_SYNC;
            }
            break;

        case state_t::PREPARE_COMPUTE_SYNC:
            RCLCPP_INFO(get_logger(), "Current SM: PREPARE_COMPUTE_SYNC");
            computeEnabledSyncTransitions();
            statemachineMoveToState(state_t::COMPUTE_SYNC);
            break;

        case state_t::COMPUTE_SYNC:
            RCLCPP_INFO(get_logger(), "Current SM: COMPUTE_SYNC"); {
            auto manage{m_petri->getManageTransitionFusionSet()};
            ml_enabled_fusion_sets = manage->getEnabledFusionSets();
            }
            m_current_state = state_t::FIRE_SYNC;
            break;

        case state_t::FIRE_SYNC:
            RCLCPP_INFO(get_logger(), "Current SM: FIRE_SYNC");
            if (!fireSyncTransition()) {
                m_command.cmd= "PROCESS_FIRE_SYNC_FINISH";
                m_command.target_ms.clear();
                m_command.source_product.clear();
                m_command.transition.clear();
                m_command_pub->publish(m_command);
                m_current_state = state_t::PROCESS_FIRE_SYNC_FINISH;
            }
            break;
        case state_t::PROCESS_FIRE_SYNC_FINISH:
            RCLCPP_INFO(get_logger(), "Current SM: PROCESS_FIRE_SYNC_FINISH");
            if (m_ack_modules.all()) {
                m_ack_modules.reset();
                m_current_state = state_t::POP_METASTATE;
            }
            break;

        case state_t::TERMINATE_BUILDING:
            RCLCPP_INFO(get_logger(), "Current SM: TERMINATE_BUILDING");
            m_command.cmd = "TERMINATE";
            RCLCPP_INFO(get_logger(), "#Metastates: %d",m_module_ss->getMetaStateCount());
            for (size_t i{};i<m_module_ss->getMetaStateCount();++i) {
                dss::MetaState *ms {(m_module_ss->getLMetaState())[i]};
                RCLCPP_INFO(get_logger(), "#Metastates: %s",ms->toString().c_str());
                for (const auto &arc: ms->getSyncSucc()) {
                    RCLCPP_INFO(get_logger(), "Arc: %s -> %s (%s)", dss::arrayModelToStdString(*(arc->getStartProduct())).c_str(),
                                dss::arrayModelToStdString((arc->getMetaStateDest()->getName())).c_str(), arc->getTransitionName().c_str());
                }
            }

            requestShutdown();
            break;
    }
}

auto MasterNode::response_receiver(const ros2dss::Response &resp) -> void {
    RCLCPP_INFO(get_logger(), "Received: %s", resp.msg.c_str());
    if (resp.msg == "ACK" and m_current_state == state_t::INIT) {
        m_ack_modules[resp.id] = 1;
    } else if (resp.msg == "ACK_GET_METASTATE" and m_current_state == state_t::BUILD_META_STATE) {
        // RCLCPP_INFO(get_logger(), "Received: %s %s", resp.msg.c_str(), resp.scc.c_str());
        m_metastate_building_name[resp.id] = resp.scc;
        m_ack_modules[resp.id] = 1;
    } else if (resp.msg == "ACK_SET_METASTATE_NAME" and m_current_state == state_t::SEND_METASTATE_NAME) {
        //RCLCPP_INFO(get_logger(), "Received: %s", resp.msg.c_str());
        m_ack_modules[resp.id] = 1;
    } else if (resp.msg == "ACK_MOVE_TO_METASTATE") {
        // RCLCPP_INFO(get_logger(), "Received: %s", resp.msg.c_str());
        m_ack_modules[resp.id] = 1;
        std::set<std::string> enabled_sync_trans{resp.sync.begin(), resp.sync.end()};
        m_petri->getManageTransitionFusionSet()->enableSetFusion(enabled_sync_trans, resp.id);
    }
    else if (resp.msg == "ACK_PROCESS_NODE") {
        m_ack_modules[resp.id] = 1;
    }
}

auto MasterNode::buildInitialMetaState() -> void {
    m_current_meta_state = m_petri->getMetaState(m_petri->getMarquage());
    m_metastate_building_name[m_petri->getPetriID()] = m_petri->getSCCName(m_current_meta_state->getInitialSCC());
    m_command.cmd = "GET_METASTATE";
    m_command_pub->publish(m_command);
}

auto MasterNode::computeEnabledSyncTransitions() -> void {

    auto enabled_sync_trans{m_petri->getSyncEnabled(m_current_meta_state)};
    auto manageFusion{m_petri->getManageTransitionFusionSet()};
    manageFusion->enableSetFusion(enabled_sync_trans, m_petri->getPetriID());

}

auto MasterNode::statemachineMoveToState(const state_t state) -> void {
    if (m_ack_modules.all()) {
        m_ack_modules.reset();
        m_current_state = state;
    }
}

auto MasterNode::fireSyncTransition() -> bool {
    if (ml_enabled_fusion_sets.empty()) {
        RCLCPP_INFO(get_logger(), "No enabled fusion set...");
        return false;
    }
    // Pop a transition fusion set
    string transition{ml_enabled_fusion_sets[ml_enabled_fusion_sets.size() - 1]};
    ml_enabled_fusion_sets.pop_back();

    auto res = m_petri->fireSync(transition, m_current_meta_state);

    vector<vector<dss::firing_sync_t> > received_scc;
    received_scc.resize(m_petri->getModulesCount());
    for (const auto &t: res) {
        received_scc[0].push_back(dss::firing_sync_t{
            t.getSCCSource()->getName(m_petri), t.getDestSCC()->getName(m_petri), t.getDestSCC()
        });
    }
    // SCCs of master module

    for (uint32_t i{1}; i < m_petri->getModulesCount(); ++i) {
        // Send request only to modules synced on transition
        if (m_petri->getManageTransitionFusionSet()->isFusionSetSyncedOnModule(transition, i)) {
            auto res{executeFireSyncTransitionRequest(i, transition)};
            for (auto e: res) {
                RCLCPP_INFO(get_logger(), "Received (%s,%s) ", e.source.c_str(), e.target.c_str());
                received_scc[i].push_back(dss::firing_sync_t{e.source, e.target});
            }
        } else {
            // We have to put component of source metastate, as destination as well
            received_scc[i].push_back({m_current_meta_state->getSCCName(i), m_current_meta_state->getSCCName(i)});
        }
    }
    // Build edges and destination metastate for transition t
    auto l_pair_source_ms{dss::buildMetaStatesNames(received_scc)};
    for (auto &e_tuple: l_pair_source_ms) {
        RCLCPP_INFO(get_logger(), "Source product: %s, Destination metastate: %s",
                    dss::arrayModelToStdString(std::get<0>(e_tuple)).c_str(), dss::arrayModelToStdString(std::get<1>(e_tuple)).c_str());
        if (auto dest_ms{m_module_ss->findMetaState(std::get<1>(e_tuple))}; dest_ms) {
            RCLCPP_INFO(get_logger(), "Meta state: %s is already inserted!",
                        dss::arrayModelToStdString(std::get<1>(e_tuple)).c_str());
            // Insert just the edge
            m_current_meta_state->addSyncArc(new dss::ArcSync{std::get<0>(e_tuple), dest_ms,transition});
        } else {
            // Insert the new metastate
            auto new_ms {new dss::MetaState(*std::get<2>(e_tuple)->getMetaState())};
            new_ms->setName(std::get<1>(e_tuple));
            m_module_ss->insertMS(new_ms);
            m_current_meta_state->addSyncArc(new dss::ArcSync{std::get<0>(e_tuple), new_ms,transition});
            m_meta_states_stack.push(new_ms);
            // Publish the command to add new metastate
           /* m_command.cmd = "ADD_NEW_METASTATE";
            m_command.scc.clear();
            m_command.source_product=std::get<0>(e_tuple);
            m_command.target_ms=std::get<1>(e_tuple);
            m_command.transition=transition;
            m_command_pub->publish(m_command);*/
            RCLCPP_INFO(get_logger(), "Meta state: %s is new!", dss::arrayModelToStdString(new_ms->getName()).c_str());
            addFiringInfoRequest(std::get<0>(e_tuple), std::get<1>(e_tuple), transition);
        }
    }

    return true;
}

auto MasterNode::executeFireSyncTransitionRequest(const uint32_t id_server,
                                                  const string &transition) -> std::vector<dss::firing_sync_t> {
    std::vector<dss::firing_sync_t> res;

    auto client_firing_service{
        create_client<ros2dss::FiringSyncTransitionSrv>("firing_sync_transitions_service" + std::to_string(id_server))
    };
    auto request{std::make_shared<ros2dss::FiringSyncTransitionSrv::Request>()};
    request->transition = transition;
    while (!client_firing_service->wait_for_service(10ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting...");
            return res;
        }
    }

    auto future{client_firing_service->async_send_request(request)};
    while (1) {
        m_executor->spin_some();
        if (future.wait_for(10ms) == std::future_status::ready) {
            break;
        }
    }

    auto response{future.get()};
    for (const auto &f: response->lfiring) {
        res.push_back({f.source, f.target});
    }
    RCLCPP_INFO(get_logger(), "Firing transition %s\n", transition.c_str());
    return res;
}

/*
 * Execute service request to ask modules to add firing info for a transition
 */
auto MasterNode::addFiringInfoRequest(const std::vector<std::string>& startProduct,const std::vector<std::string> &targetMS,const string &transition) -> void {
    auto request{std::make_shared<ros2dss::InfoFiring::Request>()};
    request->source_product = startProduct;
    request->target_ms = targetMS;
    request->transition = transition;
    for (size_t i {1};i<m_petri->getModulesCount();++i) {
        auto future{ml_clients_firing_info[i]->async_send_request(request)};
        while (1) {
            m_executor->spin_some();
            if (future.wait_for(10ms) == std::future_status::ready) {
                break;
            }
        }
    }
}