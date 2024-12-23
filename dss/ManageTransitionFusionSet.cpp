//
// Created by chiheb on 23/12/24.
//

#include "misc.h"

namespace dss {
    void ManageTransitionFusionSet::add_fusion_set(const std::string& fusion_set_name,const uint32_t module) {
        ml_fusion_sets[fusion_set_name].insert(module);
    }
}