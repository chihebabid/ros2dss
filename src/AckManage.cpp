//
// Created by chiheb on 04/02/25.
//

#include "gmisc.h"


AckManage::AckManage(const size_t modules_count):m_ack_modules(modules_count) {
  	RCLCPP_INFO(rclcpp::get_logger("AckManage"), ": %d ",modules_count);
    reset();
}

auto AckManage::reset() -> void {
    for (size_t i{}; i < m_ack_modules.size(); ++i) {
        m_ack_modules[i] = 0;
    }
}

auto AckManage::operator[](const size_t index) -> byte_t & {
    m_ack_modules[index] = 1;
    return m_ack_modules[index];
}

auto AckManage::all() -> bool {
    for (uint32_t i{1}; i < m_ack_modules.size(); ++i) {
        if (m_ack_modules[i] == 0) return false;
    }
    return true;
}