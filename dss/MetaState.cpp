/*
 * MetaState.cpp
 *
 *  Created on: 17 sept. 2015
 *      Author: LIP2
 */
#include "misc.h"

namespace dss {
    MetaState::MetaState() {
        m_id = m_Counter++;
    }

    MetaState::MetaState(const size_t s): m_metastate_name(s) {
        m_id = m_Counter++;
    }

    const vector<Marking *> &MetaState::getListMarkings() const {
        return m_nodes;
    }

    /*vector<InternalArc>* MetaState::getListArcs() {
        return m_graph->getListArcs();
    }*/

    vector<SCC *> *MetaState::getListSCCs() {
        return &ml_scc;
    }

    SCC *MetaState::findSCC(Marking *state) {
        for (size_t i{}; i < ml_scc.size(); i++) {
            vector<Marking *> *list_states = ml_scc[i]->getListStates();
            if (std::find(list_states->begin(), list_states->end(), state) != list_states->end()) return ml_scc.at(i);
        }
        return nullptr;
    }

    SCC *MetaState::getInitialSCC() {
        return findSCC(m_nodes[0]);
    }


    void MetaState::addSyncArc(ArcSync *sync_arc) {
        mArcs.push_back(sync_arc);
    }

    vector<ArcSync *> &MetaState::getSyncSucc() {
        return mArcs;
    }

    uint32_t MetaState::getArcCount() {
        u_int32_t res = 0;
        for (auto elt: m_nodes)
            res += elt->getListSucc().size();
        return res;
    }

    bool MetaState::operator==(MetaState &ms) {
        if (ms.getListMarkings().size() != getListMarkings().size()) return false;
        for (auto &elt1: ms.getListMarkings()) {
            bool found = false;
            for (const auto &elt2: getListMarkings()) {
                if (*elt1 == *elt2) {
                    found = true;
                    break;
                }
            }
            if (!found) return false;
        }
        return true;
    }

    uint32_t MetaState::getId() const {
        return m_id;
    }

    uint32_t MetaState::m_Counter{0};

    void MetaState::computeStrongConnectedComponents(Marking *v) {
        v->index = m_index;
        v->lowlink = m_index;
        m_index++;
        m_stack.push_back(v);
        v->onstack = true;

        // Consider successors of v

        for (const auto &elt: v->getListSucc()) {
            Marking *w = elt.second;
            if (w->index == -1) {
                computeStrongConnectedComponents(w);
                v->lowlink = min(v->lowlink, w->lowlink);
            } else if (w->onstack) v->lowlink = min(v->lowlink, w->index);
        }
        // If v is a root node, pop the stack and generate an SCC
        if (v->lowlink == v->index) {
            SCC *scc = new SCC();
            scc->setMetaState(this);
            Marking *w;
            do {
                w = m_stack.back();
                m_stack.pop_back();
                w->onstack = false;
                scc->addState(w);
            } while (w != v);
            ml_scc.emplace_back(scc);
        }
    }

    Marking *MetaState::existMarking(Marking *marq) {
        for (const auto &elt: m_nodes) {
            if (*elt == *marq) return elt;
        }
        return nullptr;
    }

    Marking *MetaState::insertMarking(Marking *m) {
        Marking *p = existMarking(m);
        if (!p) m_nodes.push_back(m);
        return p;
    }


    void MetaState::computeSCCTarjan() {
        m_index = 0;
        for (auto &node: m_nodes) {
            node->onstack = false;
            node->index = -1;
            node->lowlink = 0;
        }

        for (auto &node: m_nodes) {
            if (node->index == -1) computeStrongConnectedComponents(node);
        }
    }

    void MetaState::setSCCName(const string &name, const int pos) {
        m_metastate_name[pos] = name;
    }

    std::string &MetaState::getSCCName(const int pos) {
        return m_metastate_name[pos];
    }

    void MetaState::setName(const ArrayModel<std::string> &name) {
        m_metastate_name = name;
    }

    /*
     * @brief getName
     * @return the name of a metastate
     */
    ArrayModel<string> MetaState::getName() const {
        return m_metastate_name;
    }

    std::string MetaState::toString()  {
        std::string  strbuf{"["};
        for (size_t i{};i<m_metastate_name.size();++i) {
            strbuf=strbuf+m_metastate_name[i];
            strbuf+=std::string{","};
        }
        strbuf[strbuf.size()-1]=']';
        return strbuf;
    }

}
