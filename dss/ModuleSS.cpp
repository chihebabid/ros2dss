//
// Created by chiheb on 13/07/22.
//
#include <rclcpp/logging.hpp>

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
                    elt->getSyncSucc().erase(ptr);
                    delete (*ptr);
                }
            }
        }
    }

    size_t ModuleSS::getStatesCount() const {
        size_t res{};
        for (const auto &ms: mlMetaState) {
            res += ms->getListMarkings().size();
        }
        return res;
    }

    size_t ModuleSS::getSyncEdgesCount() const {
        size_t res{};
        for (const auto &ms: mlMetaState) {
            res += ms->getSyncSucc().size();
        }
        return res;
    }

    void ModuleSS::stats() const {
        auto my_logger{rclcpp::get_logger("STATS")};
        RCLCPP_INFO(my_logger, "#Metastates: %ld", getMetaStateCount());
        RCLCPP_INFO(my_logger, "#States: %ld", getStatesCount());
        RCLCPP_INFO(my_logger, "#Edges: %ld", getSyncEdgesCount());
        for (size_t i{}; i < getMetaStateCount(); ++i) {
            dss::MetaState *ms{mlMetaState[i]};
            RCLCPP_INFO(my_logger, "#Metastates: %s", ms->toString().c_str());
            for (const auto &arc: ms->getSyncSucc()) {
                RCLCPP_INFO(my_logger, "Arc: %s -> %s (%s)",
                            dss::arrayModelToStdString(*(arc->getStartProduct())).c_str(),
                            dss::arrayModelToStdString((arc->getMetaStateDest()->getName())).c_str(),
                            arc->getTransitionName().c_str());
            }
        }
        RCLCPP_INFO(my_logger, "End stats...");
    }


    /*
 * @brief Check whether a metastate can be fused with another
 * @param ms a Metastate
 * @param module Module index
 * @return true if *ms can be fusedm else false
 */
    MetaState *ModuleSS::findEquivalentMS(MetaState *ms) {
        for (const auto &elt: mlMetaState) {
            if (elt != ms && *elt == *ms) {
                //Compare out edges
                auto lEdges1 = ms->getSyncSucc();
                auto lEdges2 = elt->getSyncSucc();
                if (lEdges1.size() == lEdges2.size()) {
                    // Check that all edges are the same
                    bool areSame = true;
                    for (const auto &edge1: lEdges1) {
                        auto compare = [ms,elt,&edge1](ArcSync *arc) {
                            if ((edge1->getMetaStateDest()==ms) and (edge1->getTransitionName() == arc->getTransitionName()) and
                                       (elt == arc->getMetaStateDest())) {
                                return true;
                            }
                            return edge1->getTransitionName() == arc->getTransitionName() && edge1->getMetaStateDest()
                                   == arc->getMetaStateDest();
                        };
                        auto res = std::find_if(lEdges2.begin(), lEdges2.end(), compare);
                        if (res == lEdges2.end()) {
                            areSame = false;
                            break;
                        }
                    }
                    if (areSame) return elt;
                }
            }
        }
        return nullptr;
    }

    MetaState *ModuleSS::findExtendedMetaState(const ArrayModel<std::string> &productscc) {
        for (const auto &elt: mlMetaState) {
            if ((elt->getName() == productscc) || elt->getEquivalence().findMetaState(productscc)) return elt;
        }
        return nullptr;
    }

    void ModuleSS::reduce(MetaState *ms) {
        if (auto e_ms = findEquivalentMS(ms); e_ms) {
            ms->getEquivalence().mergeMetaStates(e_ms->getEquivalence());
            ms->getEquivalence().insertMetaState(e_ms->getName());
            for (const auto &m: mlMetaState) {
                for (const auto &edge: m->getSyncSucc()) {
                    if (edge->getMetaStateDest() == e_ms) edge->setDestination(ms);
                }
            }
            // Remove the metastate
            RCLCPP_INFO(rclcpp::get_logger("ModuleSS"), "Removing equivalent metastate %s equivalent to %s", e_ms->toString().c_str(),ms->toString().c_str());
            removeMetaState(e_ms);
        }
    }
}

ostream &operator<<(ostream &os, const dss::ModuleSS &ss) {
    //os << dt.mo << '/' << dt.da << '/' << dt.yr;

    return os;
}
