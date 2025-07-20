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


    void ArcSync::setData(ArrayModel<string> source,const std::string& transition, MetaState *destination) {
        m_fusion = transition;
        m_destination = destination;
        m_source = source;
    }


    void ArcSync::setDestination(MetaState *destination) {
        m_destination = destination;
    }

    std::string& ArcSync::getTransitionName() {
        return m_fusion;
    }

    ArrayModel<string> *ArcSync::getStartProduct() {
        return &m_source;
    }


    ArcSync::ArcSync(ArrayModel<string> source_product, MetaState *destination,const std::string& transition):m_source(source_product), m_fusion(transition), m_destination(destination) {


    }

    bool ArcSync::operator==(const ArcSync &o) const {
        return (m_fusion == o.m_fusion) && (m_source == o.m_source) && (m_destination == o.m_destination);
    }
}
