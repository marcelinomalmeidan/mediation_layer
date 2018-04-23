#ifndef SERVICES_H_
#define SERVICES_H_

#include "mediation_layer/globals.h"

namespace services {

bool SetShield(mg_msgs::SetQuadBool::Request  &req,
               mg_msgs::SetQuadBool::Response &res);

}  // namespace services

#endif  // SERVICES_H_