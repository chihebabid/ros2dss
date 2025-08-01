#include "misc.h"

namespace dss {
    SCC::SCC() {
        mId = mCounter++;
    }


    Marking *SCC::findMarking(Marking *m) const {
        auto it{std::find_if(m_list.begin(), m_list.end(), [m](const Marking *elt) { return *m == *elt; })};
        return it == m_list.end() ? nullptr : *it;
    }

    // Compare whether two SCCs are equal
    bool SCC::isEqual(const SCC &scc) {
        if (scc.m_list.size() != this->m_list.size()) return false;
        if (this->m_list.empty()) return true;
        if (findMarking(scc.m_list[0])) return true;
        return false;
    }

    size_t SCC::getCount() const {
        return m_list.size();
    }

    void SCC::addState(Marking *m) {
        //if (!existMarking(m))
        m->setSCCContainer(this);
        m_list.emplace_back(m);
    }

    const vector<Marking *> *SCC::getListStates() const {
        return &m_list;
    }

    uint32_t SCC::getId() const {
        return mId;
    }


    uint32_t SCC::mCounter{0};

    MetaState *SCC::getMetaState() const {
        return m_parentMetaState;
    }

    void SCC::setMetaState(MetaState *ms) {
        m_parentMetaState = ms;
    }


    void SCC::IteratorSucc::update() {
        m_succ.clear();
        for (const auto &marking: m_ptr->m_list) {
            for (const auto &succ: marking->getListSucc()) {
                m_succ.emplace_back(succ.second->getSCCContainer(), succ.first);
            }
        }


        // TODO: reimplement this loop
        /*auto ms{m_ptr->getMetaState()};
         * auto module{ms->getIdModule()};
         *for(auto & edge : ms->getSyncSucc()) {
            auto startProduct {edge->getStartProduct()};
            if (m_ptr==startProduct->getSCC(module)) {
                m_succ.emplace_back(edge->getMetaStateDest()->getInitialSCC(),edge->getFusion()->getTransitionOfModule(module));
            }
        }*/
    }

    bool SCC::operator==(const SCC &o) const {
        if (m_list.size() != o.m_list.size()) return false;
        for (const auto &marking: o.m_list) {
            if (!findMarking(marking)) return false;
        }
        return true;
    }

    string SCC::getName(const PetriNet* petri) const {
        return petri->getSCCName(this);
    }
}
