/*
 * ArcSync.cpp
 *
 *  Created on: 29 sept. 2015
 *      Author: biba
 */

#include "misc.h"

namespace dss {

MetaState *ArcSync::getMetaStateDest() {
    return m_destination;
}


void ArcSync::setData(ArrayModel<string> source, TransitionFusionSet *transition, MetaState *destination) {
    m_fusion = transition;
    m_destination = destination;
    m_source = source;
}


void ArcSync::setDestination(MetaState *destination) {
    m_destination = destination;
}

TransitionFusionSet *ArcSync::getFusion() {
    return m_fusion;
}

ArrayModel<string> *ArcSync::getStartProduct() {
    return &m_source;
}
}