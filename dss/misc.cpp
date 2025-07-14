//
// Created by chiheb on 13/07/25.
//
#include "misc.h"
namespace dss {
    /*
     * @brief This function builds the metastates names from the firing sync transitions
     * @return a pair of vectors containing the source and destination metastates names
     */
    std::vector<std::pair<ArrayModel<std::string>,ArrayModel<std::string>>>  buildMetaStatesNames(std::vector<std::vector<firing_sync_t>> &l_scc) {
        std::vector<std::pair<ArrayModel<std::string>,ArrayModel<std::string>>> l_source_metastates_names;
        std::vector<int> _counters(l_scc.size(),0),_counters_max(l_scc.size());
        for (size_t i{};i<_counters_max.size();++i) {
            _counters_max[i]=l_scc[i].size();
            std::cout<<"Module i: "<<i<<" has "<<_counters_max[i]<<" SCCs\n";
        }


        auto isLittleThan = [](const std::vector<int>& a, const std::vector<int>& b) -> int {

            // Compare digits from left to right
            for (size_t i {}; i < a.size(); ++i) {
                if (a[i] < b[i]-1) {
                    return false;
                }
            }

            return true;
        };

        auto add1=[](std::vector<int> &v,const std::vector<int> &v_max) -> void {
            for (int i = static_cast<int>(v.size()) - 1;i>=0;--i) {
                if (v[i] < v_max[i]-1) {
                    ++v[i];
                    return;
                } else {
                    v[i]=0; // Reset the current counter and carry over to the next
                }
            }
        };
        // Compute the product
        while (!isLittleThan(_counters,_counters_max)) {
            // source_name : product enabling transition, ms_name : product destination metastate
            ArrayModel<std::string> source_name(l_scc.size()),ms_name(l_scc.size());
            for (size_t i{};i<l_scc.size();++i) {
                source_name[i]= l_scc[i][_counters[i]].source;
                ms_name[i]= l_scc[i][_counters[i]].target;
            }
            l_source_metastates_names.push_back({source_name,ms_name});
            add1(_counters,_counters_max);
        }
        ArrayModel<std::string> source_name(l_scc.size()),ms_name(l_scc.size());
        for (size_t i{};i<l_scc.size();++i) {
            source_name[i]= l_scc[i][_counters[i]].source;
            ms_name[i]= l_scc[i][_counters[i]].target;
        }
        l_source_metastates_names.push_back({source_name,ms_name});
        return l_source_metastates_names;
    }

    std::string arrayModelToStdString(ArrayModel<std::string>& model, const std::string& delimiter ) {
        if (model.size() == 0) {
            return "";
        }
        std::string result;
        for (size_t i = 0; i < model.size(); ++i) {
            result += model[i];
            if (i < model.size() - 1) {
                result += delimiter;
            }
        }
        return result;
    }
}