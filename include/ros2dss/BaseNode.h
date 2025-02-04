//
// Created by chiheb on 04/02/25.
//

#ifndef BASENODE_H
#define BASENODE_H

#include "gmisc.h"

class BaseNode : public rclcpp::Node {
public:
  BaseNode(dss::PetriNet  *petri,const string &);
protected:
  virtual void run()=0;
  dss::PetriNet  *m_petri;
  dss::ModuleSS  *m_module_ss {};
  dss::MetaState* m_current_meta_state {};

};



#endif //BASENODE_H
