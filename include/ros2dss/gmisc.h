//
// Created by chiheb on 16/12/24.
//

#ifndef GMISC_H
#define GMISC_H
#include <memory>
#include "misc.h"
template class dss::ArrayModel<std::string>;
inline std::unique_ptr<dss::MarkingArray> _ptr_modules {};
inline std::unique_ptr<dss::ArrayModel<std::string>> _ptr_metastate_name;// {};
#endif //MISC_H
