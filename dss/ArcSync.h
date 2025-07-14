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
    ArcSync(ArrayModel<string>, MetaState *destination, std::string *transition);

    ArcSync()=default;
    virtual ~ArcSync()=default;

    std::string *getFusion();

    void setDestination(MetaState *destination);

    void setData(ArrayModel<string> source, std::string *transition, MetaState *destination);

    MetaState *getMetaStateDest();

    ArrayModel<string> *getStartProduct();
private:
    std::string *m_fusion{};
    ArrayModel<string> m_source;
    MetaState *m_destination{};
};

}
#endif /* ARCSYNC_H_ */
