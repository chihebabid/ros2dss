//
// Created by chiheb on 18/12/24.
//

#include "misc.h"

namespace dss {
    void MetaStateName::setSCCName(const std::string &name,const int pos) {
      m_name[pos]=name;
    }
    std::string &MetaStateName::getSCCName(const int pos) {
      return m_name[pos];
    }
}