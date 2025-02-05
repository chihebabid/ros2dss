//
// Created by chiheb on 04/02/25.
//

#ifndef ACKMANAGE_H
#define ACKMANAGE_H

#include "gmisc.h"

class AckManage {
public:
    AckManage(const size_t);
    auto reset() -> void;
    auto operator[](const size_t) -> byte_t &;
    auto all() -> bool;
private:
      dss::ArrayModel<byte_t> m_ack_modules;
};



#endif //ACKMANAGE_H
