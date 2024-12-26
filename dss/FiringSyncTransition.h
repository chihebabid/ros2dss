//
// Created by chiheb on 27/12/24.
//

#ifndef FIRINGSYNCTRANSITION_H
#define FIRINGSYNCTRANSITION_H

/*
 * @brief This class represents a firing synchronization transition
 */
namespace dss {
    class FiringSyncTransition {
    public:
        FiringSyncTransition(SCC* ,const std::string &,SCC *);
        bool operator==(const FiringSyncTransition &o) const;
    private:
        SCC *m_source,*m_dest;
        std::string m_transition;
    };
}


#endif //FIRINGSYNCTRANSITION_H
