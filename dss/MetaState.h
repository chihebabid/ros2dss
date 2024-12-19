/*
 * MetaState.h
 *
 *  Created on: 17 sept. 2015
 *      Author: LIP2
 */

#ifndef METASTATE_H_
#define METASTATE_H_

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
    vector<Marking *> &getListMarkings();
    vector<SCC *> *getListSCCs();
    SCC *findSCC(Marking *state);
    SCC *getInitialSCC();
    ProductSCC *getSCCProductName();
    void setSCCProductName(ProductSCC *name);
    void addSyncArc(ArcSync *sync_arc);
    uint32_t getArcCount();
    bool operator==(MetaState &ms);
    [[nodiscard]] uint32_t getId() const;
    Marking *existMarking(Marking *marq);
    Marking *insertMarking(Marking *m);
    void computeSCCTarjan();
    void setIdModule(const size_t id) { m_id_module = id;}
    [[nodiscard]] size_t getIdModule() const { return m_id_module;}
    void setSCCName(const string &name,const int pos);
    std::string& getSCCName(const int pos);
private:
    std::vector<std::string> m_metastate_name;
    size_t m_id_module;
    ProductSCC *m_name {nullptr};
    vector<ArcSync *> mArcs;
    uint32_t m_id;
    static uint32_t m_Counter;

    vector<Marking *> m_nodes;
    vector<string> ml_transition_names;
    vector<SCC *> ml_scc;
    unsigned int m_index; // Used in Tarjan algorithm
    vector<Marking *> m_stack;
    void computeStrongConnectedComponents(Marking *v);
};
}
#endif /* METASTATE_H_ */
