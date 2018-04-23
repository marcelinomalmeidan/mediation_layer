
#include "mediation_layer/services.h"

namespace services {

bool SetShield(mg_msgs::SetQuadBool::Request  &req,
               mg_msgs::SetQuadBool::Response &res) {
	// Update reference
	pthread_mutex_lock(&mutexes_.m_ml_class);
		std::set<QuadData>::iterator it;
		globals_.obj_mid_layer.FindQuadIndex(req.quad_name, &it);
		it->has_shield = req.set_bool;
	pthread_mutex_unlock(&mutexes_.m_ml_class);
	res.success = true;
	ROS_INFO("Toggling shield for quad %s", req.quad_name.c_str());
	// res.message = "Quad " + quad_name + " shield is set to " + std::to_string(req.data);

	return true;
}

}  // namespace services