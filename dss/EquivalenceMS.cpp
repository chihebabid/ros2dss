//
// Created by chiheb on 21/07/25.
//

#include "EquivalenceMS.h"

namespace dss {
    void EquivalenceMS::insertMetaState(const ArrayModel<std::string> &ms) {
        if (std::find(ml_metastates.begin(), ml_metastates.end(), ms) == ml_metastates.end()) {
            ml_metastates.push_back(ms);
        }
    }

    bool EquivalenceMS::findMetaState(const ArrayModel<std::string> &ms) const {
        return std::find(ml_metastates.begin(), ml_metastates.end(), ms) != ml_metastates.end();
    }

    void EquivalenceMS::mergeMetaStates(const EquivalenceMS &other) {
        for (const auto &ms : other.ml_metastates) {
                ml_metastates.push_back(ms);
        }
    }
}

