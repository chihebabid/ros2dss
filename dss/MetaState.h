/*
 * MetaState.h
 *
 *  Created on: 17 sept. 2015
 *      Author: LIP2
 */

#ifndef METASTATE_H_
#define METASTATE_H_

#include "EquivalenceMS.h"
#include "misc.h"

namespace dss {

using namespace std;

class ArcSync;
class ProductSCC;

class MetaState {
public:
    MetaState();
    explicit MetaState(const size_t s);
    virtual ~MetaState()=default;

    vector<ArcSync *> &getSyncSucc();
    const vector<Marking *> &getListMarkings() const;
    vector<SCC *> *getListSCCs();
    SCC *findSCC(Marking *state) const;
    SCC *getInitialSCC() const;


    void addSyncArc(ArcSync *sync_arc);
    size_t getArcCount() const;
    bool operator==(MetaState &ms);
    [[nodiscard]] uint32_t getId() const;
    Marking *existMarking(Marking *marq);
    Marking *insertMarking(Marking *m);
    void computeSCCTarjan();
    void setIdModule(const size_t id) { m_id_module = id;}
    [[nodiscard]] size_t getIdModule() const { return m_id_module;}
    void setSCCName(const string &name,const int pos);
    std::string& getSCCName(const int pos);
    void setName(const ArrayModel<string> &);
    void setName(const std::vector<string> &);
    ArrayModel<string> getName() const;
    std::string toString();
    Marking* getInitialMarking() const ;
    bool isEquivalent(const ArrayModel<string>&) const;
    EquivalenceMS &getEquivalence();
    [[nodiscard]] bool getProcessedReduction() const;
    void setProcessedReduction(const bool v);
private:
    ArrayModel<string> m_metastate_name;
    EquivalenceMS m_equivalence;
    size_t m_id_module;

    vector<ArcSync *> mArcs;
    uint32_t m_id;
    static uint32_t m_Counter;

    vector<Marking *> m_nodes;
    vector<string> ml_transition_names;
    vector<SCC *> ml_scc;
    unsigned int m_index; // Used in Tarjan algorithm
    vector<Marking *> m_stack;
    void computeStrongConnectedComponents(Marking *v);
    bool m_processed_reduction {false};

};
}
#endif /* METASTATE_H_ */
