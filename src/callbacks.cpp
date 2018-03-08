
#include "mediation_layer/callbacks.h"

namespace callbacks {

void PVACallback(const mg_msgs::PVA::ConstPtr& msg,
                 const std::string& quad_name) {
	// Update reference
	pthread_mutex_lock(&mutexes_.m_ml_class);
		globals_.obj_mid_layer.UpdateQuadReference(quad_name, *msg);
	pthread_mutex_unlock(&mutexes_.m_ml_class);
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                  const std::string& quad_name) {
	// Update vehicle odometry and set active to true
	pthread_mutex_lock(&mutexes_.m_ml_class);
		globals_.obj_mid_layer.UpdateQuadOdom(quad_name, *msg);
	pthread_mutex_unlock(&mutexes_.m_ml_class);
}

}  // namespace callbacks