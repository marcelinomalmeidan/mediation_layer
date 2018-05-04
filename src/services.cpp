
#include "mediation_layer/services.h"

namespace services {

bool SetShield(mg_msgs::SetQuadBool::Request  &req,
               mg_msgs::SetQuadBool::Response &res) {
	pthread_mutex_lock(&mutexes_.m_ml_class);
		std::set<QuadData>::iterator it;
		globals_.obj_mid_layer.FindQuadIndex(req.quad_name, &it);
		if(it != globals_.obj_mid_layer.quads_.end()) {
			it->has_shield = req.set_bool;
			ROS_INFO("Toggling shield to %d for quad %s", 
				req.set_bool, req.quad_name.c_str());
		} else {
			ROS_INFO("[mediation layer] Service SetShield: Quad not found!");
		}
	pthread_mutex_unlock(&mutexes_.m_ml_class);

	res.success = true;
	return true;
}

// Set 1 for taking off/landing
// Set 0 for not taking off/landing
bool SetTakeOffLanding(mg_msgs::SetQuadBool::Request  &req,
	                   mg_msgs::SetQuadBool::Response &res) {
	pthread_mutex_lock(&mutexes_.m_ml_class);
		std::set<QuadData>::iterator it;
		globals_.obj_mid_layer.FindQuadIndex(req.quad_name, &it);
		if(it != globals_.obj_mid_layer.quads_.end()) {
			it->is_takeoff_landing = req.set_bool;
			if (req.set_bool) {
				ROS_INFO("Quad %s has requested to land!",
					req.quad_name.c_str());
			}
		} else {
			ROS_INFO("[mediation layer] Service SetTakeOffLanding: Quad not found!");
		}
	pthread_mutex_unlock(&mutexes_.m_ml_class);
	
	res.success = true;
	return true;
}

// Set 1 for signaling that quad is ready for game
bool ReadyForGame(mg_msgs::SetQuadBool::Request  &req,
	              mg_msgs::SetQuadBool::Response &res) {
	pthread_mutex_lock(&mutexes_.m_ml_class);
		std::set<QuadData>::iterator it;
		globals_.obj_mid_layer.FindQuadIndex(req.quad_name, &it);
		if(it != globals_.obj_mid_layer.quads_.end()) {
			it->is_takeoff_landing = !req.set_bool;
			if (req.set_bool) {
				ROS_INFO("Quad %s is ready to start the match!",
					req.quad_name.c_str());
			}
		} else {
			ROS_INFO("[mediation layer] Service ReadyForGame: Quad not found!");
		}
	pthread_mutex_unlock(&mutexes_.m_ml_class);

	res.success = true;
	return true;
}

}  // namespace services