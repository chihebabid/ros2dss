//
// Created by chiheb on 21/07/25.
//

#ifndef EQUIVALENCEMS_H
#define EQUIVALENCEMS_H
#include "ArrayModel.h"
#include <stdbool.h>
#include <string>
#include <vector>


namespace dss {
    using std::string;
    class EquivalenceMS {
    public:
        void insertMetaState(const ArrayModel<string> &ms);
        bool findMetaState(const ArrayModel<string> &ms) const;

        void mergeMetaStates(const EquivalenceMS &other) ;
    private:
        std::vector<ArrayModel<string>> ml_metastates;
    };
}


#endif //EQUIVALENCEMS_H
