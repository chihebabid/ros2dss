//
// Created by chiheb on 23/12/24.
//

#include "misc.h"

namespace dss {
    void ManageTransitionFusionSet::add_fusion_set(const std::string& fusion_set_name,const uint32_t module) {
        ml_fusion_sets[fusion_set_name].push_back({module,false});
    }

    uint32_t ManageTransitionFusionSet::getFusionSetsCount() const {
        return ml_fusion_sets.size();
    }

    /*
	 * @brief checks whether a fusion set is enabled or not
     */
    bool ManageTransitionFusionSet::isFusionEnabled(const std::string &name) const {
        auto it {ml_fusion_sets.find(name)};
		auto & _set {it->second};
        auto it_res {std::find_if(_set.begin(),_set.end(),[](const fusion_set_t &fusion_set){return  !fusion_set.enabled;})};
        return it_res==_set.end();
    }

    void ManageTransitionFusionSet::reset() {
      for (auto &elt: ml_fusion_sets) {
			for (auto &elt: elt.second) {
                elt.enabled=false;
			}

      }
    }

    void ManageTransitionFusionSet::enableFusion(const std::string &name,uint32_t module) {
        auto it {ml_fusion_sets.find(name)};
        auto & _set {it->second};
        auto it_res {std::find_if(_set.begin(),_set.end(),[module](const fusion_set_t &fusion_set){return  fusion_set.module==module;})};
        it_res->enabled=true;
    }
}