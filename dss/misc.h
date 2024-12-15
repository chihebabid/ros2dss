//
// Created by chiheb on 14/12/24.
//

#ifndef MISC_H
#define MISC_H
#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <set>
#include <algorithm>
#include "MarkingArray.h"
#include "Marking.h"
#include "Node.h"
#include "Place.h"

#include "Transition.h"
#include "TransitionFusionSet.h"
#include "MetaState.h"
#include "PetriNet.h"
#include "BuildPetri.h"
#include "SCC.h"
#include "ProductSCC.h"
#include "ArcSync.h"

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
