//
// Created by chiheb on 13/07/22.
//

#ifndef DISTRIBUTEDSTATESPACE_MODULESS_H
#define DISTRIBUTEDSTATESPACE_MODULESS_H

#include "misc.h"

namespace dss {
    class ModuleSS {
    public:
        ModuleSS(size_t id): m_id_module(id) {
        }

        MetaState *getInitialMS();

        bool insertMS(MetaState *ms);

        MetaState *findMetaState(ArrayModel<string> &productscc);
        MetaState *findMetaState(const string &);

        MetaState *findExtendedMetaState(const ArrayModel<std::string> &productscc);
        friend ostream &operator<<(ostream &os, const ModuleSS &ss);

        size_t getMetaStateCount() const;
        size_t getStatesCount() const;
        size_t getSyncEdgesCount() const;

        MetaState *getMetaState(const int32_t &pos);

        vector<MetaState *> &getLMetaState();

        void removeMetaState(MetaState *ms);

        size_t getIDModule() const { return m_id_module; }

        void stats() const;

        void reduce(MetaState *ms);

    private:
        MetaState *findEquivalentMS(MetaState *ms);
        vector<MetaState *> mlMetaState;
        size_t m_id_module;
    };
}

#endif //DISTRIBUTEDSTATESPACE_MODULESS_H
