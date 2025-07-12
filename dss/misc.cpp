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

        while (isLittleThan(_counters,_counters_max)) {

        }
        return l_metastates_names;
    }
}