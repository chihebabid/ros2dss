//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"


AckManage::AckManage(const size_t modules_count):m_ack_modules(modules_count) {
    reset();
}

auto AckManage::reset() -> void {
    for (uint32_t i{}; i < m_ack_modules.size(); ++i) {
        m_ack_modules[i] = 0;
    }
}

auto AckManage::operator[](const size_t index) -> AckManage & {
    m_ack_modules[index] = 1;
    return *this;
}

auto AckManage::all() -> bool {
    for (uint32_t i{1}; i < m_ack_modules.size(); ++i) {
        if (m_ack_modules[i] == 0) return false;
    }
    return true;
}