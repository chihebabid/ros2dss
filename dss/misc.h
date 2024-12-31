//
// Created by chiheb on 14/12/24.
//

#ifndef MISC_H
#define MISC_H
#include <string>
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
#include "SSBuilder.h"
namespace dss {
    struct element_t {
        Marking marquage;
        vector<Transition *> liste_transitions;
    };


    struct PElement {
        Marking *marquage;
        vector<Transition *> liste_transitions;
    };


}

#endif //MISC_H
