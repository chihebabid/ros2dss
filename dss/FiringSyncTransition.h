//
// Created by chiheb on 27/12/24.
//

#ifndef FIRINGSYNCTRANSITION_H
#define FIRINGSYNCTRANSITION_H
#include "misc.h"
/*
 * @brief This class represents a firing synchronization transition
 */
namespace dss {
    class FiringSyncTransition {
    public:
        FiringSyncTransition(SCC* ,const std::string &,SCC *);
        bool operator==(const FiringSyncTransition &o) const;
        bool operator<(const FiringSyncTransition &o) const ;
        [[nodiscard]] SCC* getDestSCC() const;
        [[nodiscard]] SCC * getSCCSource() const;
        [[nodiscard]] std::string getTransition() const;
    private:
        SCC *m_source;
        std::string m_transition;
        SCC *m_dest;
    };


}


#endif //FIRINGSYNCTRANSITION_H
