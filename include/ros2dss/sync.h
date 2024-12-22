//
// Created by chiheb on 22/12/24.
//

#ifndef GETSYNCTRANSITIONS_H
#define GETSYNCTRANSITIONS_H



namespace ros2dss {
    struct transitionfusionset_t {
        std::string name;
        std::map<uint32_t, bool> enabled;
        uint32_t module_count {};
    };

    void updateSyncTransitions(const dss::PetriNet *petri );

};



#endif //GETSYNCTRANSITIONS_H
