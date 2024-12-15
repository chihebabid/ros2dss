/*
 * ArcSync.h
 *
 *  Created on: 29 sept. 2015
 *      Author: biba
 */

#ifndef ARCSYNC_H_
#define ARCSYNC_H_
#include "misc.h"

namespace dss {

class ArcSync {
public:
    ArcSync()=default;
    virtual ~ArcSync()=default;

    TransitionFusionSet *getFusion();

    void setDestination(MetaState *destination);

    void setData(ProductSCC source, TransitionFusionSet *transition, MetaState *destination);

    MetaState *getMetaStateDest();

    ProductSCC *getStartProduct();
private:
    TransitionFusionSet *m_fusion;
    ProductSCC m_source;
    MetaState *m_destination;
};

}
#endif /* ARCSYNC_H_ */
