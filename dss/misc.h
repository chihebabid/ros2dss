//
// Created by chiheb on 14/12/24.
//

#ifndef MISC_H
#define MISC_H
#include <string>
#include <vector>

#include <cstring>
#include <vector>
#include <iostream>
#include <cstdint>
#include <set>
#include <algorithm>
#include <sstream>
#include <stack>
#include <map>
#include <memory>
#include <list>
#include <unordered_set>

#include "ArrayModel.h"
#include "MarkingArray.h"
#include "Marking.h"
#include "Node.h"
#include "Place.h"
#include "Transition.h"
#include "ManageTransitionFusionSet.h"
#include "MetaState.h"

#include "FiringSyncTransition.h"
#include "PetriNet.h"
#include "BuildPetri.h"
#include "SCC.h"

#include "ArcSync.h"
#include "MetaStateName.h"
#include "ModuleSS.h"


namespace dss {

    struct firing_sync_t {
        std::string source;
        std::string target;
        SCC* scc_target {};
    };


    struct element_t {
        Marking marquage;
        vector<Transition *> liste_transitions;
    };


    struct PElement {
        Marking *marquage;
        vector<Transition *> liste_transitions;
    };

    std::vector<std::tuple<ArrayModel<std::string>,ArrayModel<std::string>,SCC*>>  buildMetaStatesNames(std::vector<std::vector<firing_sync_t>> &l_scc);


    std::string arrayModelToStdString(const ArrayModel<std::string>& model, const std::string& delimiter = " ");

}

#endif //MISC_H
