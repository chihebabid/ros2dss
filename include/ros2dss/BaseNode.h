//
// Created by chiheb on 04/02/25.
//

#ifndef BASENODE_H
#define BASENODE_H

#include "gmisc.h"

class BaseNode : public rclcpp::Node {
public:
  BaseNode(dss::PetriNet  *petri,const string &);
  virtual void run()=0;
  auto shouldShutdown() const -> bool;
  auto requestShutdown() -> void;
protected:
  dss::PetriNet  *m_petri;
  dss::ModuleSS  *m_module_ss {};
  dss::MetaState* m_current_meta_state {};
  bool m_should_shutdown {false};

};



#endif //BASENODE_H
