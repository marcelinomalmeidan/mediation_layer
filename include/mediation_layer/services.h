#ifndef SERVICES_H_
#define SERVICES_H_

#include "mediation_layer/globals.h"

namespace services {

bool SetShield(mg_msgs::SetQuadBool::Request  &req,
               mg_msgs::SetQuadBool::Response &res);

// Set 1 for taking off/landing
// Set 0 for not taking off/landing
bool SetTakeOffLanding(mg_msgs::SetQuadBool::Request  &req,
	                   mg_msgs::SetQuadBool::Response &res);

// Set 1 for signaling that quad is ready for game
bool ReadyForGame(mg_msgs::SetQuadBool::Request  &req,
	              mg_msgs::SetQuadBool::Response &res);

}  // namespace services

#endif  // SERVICES_H_