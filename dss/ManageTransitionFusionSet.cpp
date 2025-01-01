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

    std::vector<std::string> ManageTransitionFusionSet::getEnabledFusionSets() const {
        std::vector<std::string> _vec;
        for (auto &elt: ml_fusion_sets) {
            if (isFusionEnabled(elt.first)) {
                _vec.push_back(elt.first);
            }
        }
        return _vec;
    }

     /*
	 * @brief disable all fusion sets
     */
    void ManageTransitionFusionSet::reset() {
      for (auto &fusion: ml_fusion_sets) {
			for (auto &elt: fusion.second) {
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

    void ManageTransitionFusionSet::enableSetFusion(const set<string> &_set,uint32_t module) {
      for(const auto &elt:_set) {
        enableFusion(elt,module);
      }
    }

    void ManageTransitionFusionSet::display() {
      	for (auto & [name,l] : ml_fusion_sets) {
        	printf("Transition: %s\n",name.c_str());
            for (auto &elt: l) {
              printf("module: %d, enabled: %d\n",elt.module,elt.enabled);
            }
        }
    }
}