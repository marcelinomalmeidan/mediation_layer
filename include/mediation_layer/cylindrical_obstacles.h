#ifndef CYLINDRICAL_OBSTACLES_H_
#define CYLINDRICAL_OBSTACLES_H_

#include <Eigen/Dense>
#include "mediation_layer/linear_algebra.h"

class CylindricalObstacle {
 public:

 	// Constructors
 	CylindricalObstacle() {};
 	CylindricalObstacle(const Eigen::Vector3d &center,
 		                const double &height,
 		                const double &radius) {
 		center_ = center;
 		height_ = height;
 		radius_ = radius;
 	}

 	// If the obstacle is above  or below cylinder level, the distance
 	// is returned as infinity
 	void DistancePoint2Cylinder(const Eigen::Vector3d &point,
                                double *dist,
                                Eigen::Vector3d *normal) {
 		const double cylinder_top_z = center_[2] + height_/2.0;
 		const double cylinder_bottom_z = center_[2] - height_/2.0;
 		const double point_height = point[2];
 		
 		// Check if quad is within cylinder's height
 		if((point_height < cylinder_bottom_z) || (point_height > cylinder_top_z)) {
 			*dist = std::numeric_limits<double>::infinity();
 			*normal << 0.0, 0.0, 0.0;
 		} else {
 			// Construct a vertical line
 			Line3d z_line(Eigen::Vector3d(center_[0], center_[1], 0.0),
 			          	  Eigen::Vector3d(center_[0], center_[1], 1.0));

 			// Project point onto vertical line
 			Eigen::Vector3d nearest_point;
			z_line.ProjectPointOntoLine(point, &nearest_point, dist);
			*normal = point - nearest_point;
 		}
 	}

	void GetParameters(Eigen::Vector3d *center,
		               double *height,
		               double *radius) const {
 		*center = center_;
 		*height = height_;
 		*radius = radius_;
 	}

 private:
 	Eigen::Vector3d center_;
 	double height_;
 	double radius_;
};

class CylindricalObstacleSet {
 public:
 	std::vector<CylindricalObstacle> cyl_obstacle_set_;

 	// Constructors
 	CylindricalObstacleSet() {};

 	// Methods
 	void AddObstacle(const Eigen::Vector3d &center,
 		             const double &height,
 		             const double &radius) {
 		cyl_obstacle_set_.
 			push_back(CylindricalObstacle(center, height, radius));
 	}

 	// Return distance to obstacles
 	void DistancePoint2Cylinders(const Eigen::Vector3d &point,
                                 std::vector<double> *dists,
                                 std::vector<Eigen::Vector3d> *normals) {
 		for(uint i = 0; i < cyl_obstacle_set_.size(); i++) {
 			double dist;
 			Eigen::Vector3d normal;
 			cyl_obstacle_set_[i].DistancePoint2Cylinder(point, &dist, &normal);
 			dists->push_back(dist);
 			normals->push_back(normal);
 		}
 	}

 	uint GetNumObstacles() const {
 		return cyl_obstacle_set_.size();
 	}
};

#endif  // CYLINDRICAL_OBSTACLES_H_
