//
// Created by chiheb on 13/07/25.
//
#include "misc.h"
namespace dss {
    std::vector<ArrayModel<std::string>> buildMetaStatesNames(std::vector<std::vector<firing_sync_t>> &l_scc) {
        std::vector<ArrayModel<std::string>> l_metastates_names;
        std::vector<int> _counters(l_scc.size(),0),_counters_max(l_scc.size());
        for (size_t i{};i<_counters_max.size();++i) {
            _counters_max[i]=l_scc[i].size();
        }


        auto isLittleThan = [](const std::vector<int>& a, const std::vector<int>& b) -> int {

            // Compare digits from left to right
            for (size_t i {}; i < a.size(); ++i) {
                if (a[i] < b[i]) {
                    return true; // a is less than b
                } else if (a[i] > b[i]) {
                    return false;  // a is greater than b
                }
            }

            return true;
        };

        auto add1=[](std::vector<int> &v,const std::vector<int> &v_max) -> void {
            for (size_t i{v.size()-1};i=0;--i) {
                if (v[i] < v_max[i]-1) {
                    ++v[i];
                    return;
                } else {
                    v[i]=0; // Reset the current counter and carry over to the next
                }
            }
        };
        // Compute the product
        while (isLittleThan(_counters,_counters_max)) {
            // source_name : product enabling transition, ms_name : product destination metastate
            ArrayModel<std::string> source_name(l_scc.size()),ms_name(l_scc.size());
            for (size_t i{};i<l_scc.size();++i) {
                source_name[i]= l_scc[i][_counters[i]].source;
                ms_name[i]= l_scc[i][_counters[i]].target;
            }
            add1(_counters,_counters_max);
        }
        return l_metastates_names;
    }
}