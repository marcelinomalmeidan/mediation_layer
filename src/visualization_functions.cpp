#include "mediation_layer/visualization_functions.h"

namespace visualization_functions {


void SelectColor(const uint &i, std_msgs::ColorRGBA *color) {
	const uint n_colors = 9;
	const uint index = i % n_colors;  // Returns a number between 0 and 8
	switch(index) {
        case 0: *color = Color::Red(); break;
        case 1: *color = Color::Blue(); break;
        case 2: *color = Color::Green(); break;
        case 3: *color = Color::Yellow(); break;
        case 4: *color = Color::Orange(); break;
        case 5: *color = Color::Purple(); break;
        case 6: *color = Color::Chartreuse(); break;
        case 7: *color = Color::Teal(); break;
        case 8: *color = Color::Pink(); break;
    }
}

void SelectColor(const std::string &des_color, std_msgs::ColorRGBA *color) {
	if(des_color.compare("red") == 0) {
		*color = Color::Red();
	} else if(des_color.compare("blue") == 0) {
		*color = Color::Blue();
	} else if(des_color.compare("green") == 0) {
		*color = Color::Green();
	} else if(des_color.compare("yellow") == 0) {
		*color = Color::Yellow();
	} else if(des_color.compare("orange") == 0) {
		*color = Color::Orange();
	} else if(des_color.compare("purple") == 0) {
		*color = Color::Purple();
	} else if(des_color.compare("chartreuse") == 0) {
		*color = Color::Chartreuse();
	} else if(des_color.compare("teal") == 0) {
		*color = Color::Teal();
	} else if(des_color.compare("pink") == 0) {
		*color = Color::Pink();
	} else {
		*color = Color::White();
	} 
}

void SphereMarker(const Eigen::Vector3d &point,
	              const std::string &frame_id,
	              const std::string &ns,  // namespace
	              const double &size,
	              const std_msgs::ColorRGBA &color,
	              const double &transparency,  // 0 -> transparent, 1 -> opaque
	              const int &seqNumber,
	              visualization_msgs::MarkerArray *markerArray) {
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color = color;
	marker.color.a = transparency;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.pose.orientation.w = 1.0;

	geometry_msgs::Point position;
	position.x = point(0);
	position.y = point(1);
	position.z = point(2);
	marker.pose.position = position;
	marker.id = seqNumber;
	marker.lifetime = ros::Duration(1.0);
	markerArray->markers.push_back(marker);
}

void PlaneMarker(const Eigen::Vector3d &point,
                 const std::string frame_id,
                 const std::string &ns,  // namespace
                 const double &size,
                 const std_msgs::ColorRGBA &color,
                 const double &transparency,  // 0 -> transparent, 1 -> opaque
                 const int &seqNumber,
                 const double heading_direction,
                 visualization_msgs::MarkerArray *markerArray) {
    // Initialize array
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.id = seqNumber;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = color;
    marker.color.a = transparency;

	Eigen::Vector3d origin, x_dir, y_dir, z_dir;
	Eigen::Vector3d corner1, corner2, corner3, corner4;
	x_dir << cos(heading_direction), sin(heading_direction), 0.0;
	z_dir << 0.0, 0.0, 1.0;
	y_dir << z_dir.cross(x_dir);
	
	origin = point + 0.5*size*x_dir;
	corner1 = origin - 0.5*size*y_dir - 0.5*size*z_dir;
	corner2 = origin + 0.5*size*y_dir + 0.5*size*z_dir;
	corner3 = origin + 0.5*size*y_dir - 0.5*size*z_dir;
	corner4 = origin - 0.5*size*y_dir + 0.5*size*z_dir;

    // Get markers
    visualization_functions::SetPlanePoints(corner1, corner2, corner3, corner4, marker.color, &marker);
    markerArray->markers.push_back(marker);
}

void MeshMarker(const Eigen::Vector3d &point,
	            const Eigen::Quaterniond &quat,
	            const std::string &frame_id,
	            const std::string &ns,  // namespace
	            const double &size,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
	            const int &seqNumber,
	            visualization_msgs::MarkerArray *markerArray) {
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://mediation_layer/meshes/quadrotor_base.dae";
	marker.action = visualization_msgs::Marker::ADD;
	marker.color = color;
	marker.color.a = transparency;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.pose.orientation.w = quat.w();
	marker.pose.orientation.x = quat.x();
	marker.pose.orientation.y = quat.y();
	marker.pose.orientation.z = quat.z();

	geometry_msgs::Point position;
	position.x = point(0);
	position.y = point(1);
	position.z = point(2);
	marker.pose.position = position;
	marker.id = seqNumber;
	marker.lifetime = ros::Duration(1.0);
	markerArray->markers.push_back(marker);
}

void ForceMarker(const Eigen::Vector3d &point,
				 const Eigen::Vector3d &force,
                 const double &max_accel,
                 const std::string &frame_id,
                 const std::string &ns,  // namespace
                 const std_msgs::ColorRGBA &color,
                 const int &seqNumber,
                 visualization_msgs::MarkerArray *markerArray) {
	visualization_msgs::Marker marker;

	// Get initial and final points
	Eigen::Vector3d p0, pf;	 // Final point
	p0 = point + Eigen::Vector3d(0.0, 0.0, 0.1);

	// If force is too small, publish an invisible arrow
	const double epsilon = 0.001;
	const double force_norm = force.norm();
	if (force_norm < epsilon) {
		marker.scale.x = 0;
		marker.scale.y = 0;
		marker.scale.z = 0;
		pf = p0;
	} else {
		Eigen::Vector3d direction = force.normalized();
		Eigen::Vector3d force_new = 
						std::min(force_norm, max_accel)*direction;
		pf = p0 + 0.1*force_new;

		// Set arrow parameters
		const double length = (pf-p0).norm();
		marker.scale.x = std::min(0.1, length*0.1);
		marker.scale.y = std::min(0.2, length*0.2);
		marker.scale.z = std::min(0.2, length*0.2);
	}

	// Arrow properties
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color = color;
	marker.color.a = 1.0;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();

	// Set point in structure
	marker.points.resize(2);
	marker.points[0] = helper::SetPoint(p0(0), p0(1), p0(2));
	marker.points[1] = helper::SetPoint(pf(0), pf(1), pf(2));
	marker.id = seqNumber;
	marker.lifetime = ros::Duration(1.0);
	markerArray->markers.push_back(marker);
}

void NameMarker(const Eigen::Vector3d &point,
				const std::string &quad_name,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const std_msgs::ColorRGBA &color,
                const int &seqNumber,
                visualization_msgs::MarkerArray *markerArray) {
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color = color;
	marker.text = quad_name;
	marker.color.a = 1.0;
	marker.scale.z = 0.15;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.pose.position = 
		helper::SetPoint(point(0), point(1)-0.1, point(2)+0.25);
	marker.pose.orientation.w = 1.0;
	marker.id = seqNumber;
	marker.lifetime = ros::Duration(1.0);

	markerArray->markers.push_back(marker);
}

void FrameMarker(const Eigen::Vector3d &point,
                 const Eigen::Matrix3d &Rot,
                 const std::string &frame_id,
                 const std::string &ns,  // namespace
                 const std_msgs::ColorRGBA &color,
                 const int &seqNumber,
                 const double length,
                 visualization_msgs::MarkerArray *markerArray) {
	// const double diameter = 0.01;
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color = color;
	marker.color.a = 1.0;
	marker.scale.x = length*0.1;
	marker.scale.y = length*2*0.1;
	marker.scale.z = length*0.2;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.lifetime = ros::Duration(1.0);

	Eigen::Vector3d x_dir, y_dir, z_dir, xf, yf, zf;
	x_dir << Rot(0,0), Rot(1,0), Rot(2,0);
	y_dir << Rot(0,1), Rot(1,1), Rot(2,1);
	z_dir << Rot(0,2), Rot(1,2), Rot(2,2);
	xf = point + length*x_dir;
	yf = point + length*y_dir;
	zf = point + length*z_dir;

	// Add points
	// geometry_msgs::Point p0 = SetPoint(point(0), point(1), point(2));
	marker.points.resize(2);
	marker.points[0] = helper::SetPoint(point(0), point(1), point(2));
	marker.points[1] = helper::SetPoint(xf(0), xf(1), xf(2));
	marker.id = seqNumber;
	markerArray->markers.push_back(marker);
	marker.points[1] = helper::SetPoint(yf(0), yf(1), yf(2));
	marker.id = seqNumber + 1;
	markerArray->markers.push_back(marker);
	marker.points[1] = helper::SetPoint(zf(0), zf(1), zf(2));
	marker.id = seqNumber + 2;

	markerArray->markers.push_back(marker);
}

// Marker for visualizing a plane
// corner1 is assumed to be in an opposite edge w.r.t. corner2
void PlaneMarker(const Eigen::Vector3d corner1,
                 const Eigen::Vector3d corner2,
                 const Eigen::Vector3d corner3,
                 const Eigen::Vector3d corner4,
                 const std::string frame_id,
                 const std::string &ns,  // namespace
                 const std_msgs::ColorRGBA &color,
                 const double &transparency,  // 0 -> transparent, 1 -> opaque
                 visualization_msgs::MarkerArray *marker_array) {
    // Initialize array
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.id = 0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = color;
    marker.color.a = transparency;

    visualization_functions::SetPlanePoints(corner1, corner2, corner3, corner4, marker.color, &marker);

    marker_array->markers.push_back(marker);
}

// corner1 is assumed to be in an opposite edge w.r.t. corner2
void SetPlanePoints(const Eigen::Vector3d corner1,
                    const Eigen::Vector3d corner2,
                    const Eigen::Vector3d corner3,
                    const Eigen::Vector3d corner4,
                    const std_msgs::ColorRGBA color,
                    visualization_msgs::Marker *marker) {
    marker->points.push_back(helper::Vec3d2point(corner1));
    marker->colors.push_back(color);
    marker->points.push_back(helper::Vec3d2point(corner2));
    marker->colors.push_back(color);
    marker->points.push_back(helper::Vec3d2point(corner3));
    marker->colors.push_back(color);
    marker->points.push_back(helper::Vec3d2point(corner1));
    marker->colors.push_back(color);
    marker->points.push_back(helper::Vec3d2point(corner2));
    marker->colors.push_back(color);
    marker->points.push_back(helper::Vec3d2point(corner4));
    marker->colors.push_back(color);
}

}  // namespace visualization_functions