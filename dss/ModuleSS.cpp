//
// Created by chiheb on 13/07/22.
//
#include "misc.h"
namespace dss {
MetaState *ModuleSS::getInitialMS() {
    return mlMetaState[0];
}

/*
 * Try to insert a new MetaState
 * @return : false if it is already exists
 */
bool ModuleSS::insertMS(MetaState *ms) {
    ms->setIdModule(m_id_module);
    mlMetaState.push_back(ms);
    return true;
}

MetaState *ModuleSS::findMetaState(ArrayModel<string> &productscc) {
    for (const auto &elt: mlMetaState) {
        if ((elt->getName()) == productscc) return elt;
    }
    return nullptr;
}

MetaState *ModuleSS::findMetaState(const string &name) {
  for (const auto &elt: mlMetaState) {
    if (elt->toString() == name) return elt;
  }
  return nullptr;
}

size_t ModuleSS::getMetaStateCount() const {
    return mlMetaState.size();
}

MetaState *ModuleSS::getMetaState(const int32_t &pos) {
    return mlMetaState[pos];
}

/*
 * @brief Return Metastates list
 */
std::vector<MetaState *> &ModuleSS::getLMetaState() {
    return mlMetaState;
}

/*
 * @brief Remove a metastate
 */
void ModuleSS::removeMetaState(MetaState *ms) {
    for (auto edge: ms->getSyncSucc()) {
        delete edge;
    }

    auto ptrMl = std::remove(mlMetaState.begin(), mlMetaState.end(), ms);
    mlMetaState.erase(ptrMl, mlMetaState.end());

    auto compare = [&ms](ArcSync *arc) -> bool {
        return ms == arc->getMetaStateDest();
    };

    for (const auto elt: mlMetaState) {
        if (!elt->getSyncSucc().empty()) {
            auto ptr = std::find_if(begin(elt->getSyncSucc()), end(elt->getSyncSucc()), compare);
            if (ptr != elt->getSyncSucc().end()) {
                //cout<<"erassed\n";
                //cout<<"size before : "<<elt->getSyncSucc().size()<<endl;
                elt->getSyncSucc().erase(ptr);
                //cout<<"size after : "<<elt->getSyncSucc().size()<<endl;
                delete (*ptr);
            }
        }
    }
}
}
ostream &operator<<(ostream &os, const dss::ModuleSS &ss) {
    //os << dt.mo << '/' << dt.da << '/' << dt.yr;

    return os;
}

