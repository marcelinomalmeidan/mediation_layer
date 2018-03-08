#ifndef CALLBACKS_H_
#define CALLBACKS_H_

#include "mediation_layer/globals.h"

namespace callbacks {

void PVACallback(const mg_msgs::PVA::ConstPtr& msg,
                 const std::string& quad_name);

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                  const std::string& quad_name);

}  // namespace callbacks

#endif  // CALLBACKS_H_