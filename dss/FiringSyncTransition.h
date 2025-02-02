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
        bool operator<(const FiringSyncTransition &o) const ;
        SCC* getDestSCC() const;
        [[nodiscard]] SCC * getSCCSource() const;
        [[nodiscard]] std::string getTransition() const;
        operator firing_sync_t() const;
    private:
        SCC *m_source,*m_dest;
        std::string m_transition;
    };


}


#endif //FIRINGSYNCTRANSITION_H
