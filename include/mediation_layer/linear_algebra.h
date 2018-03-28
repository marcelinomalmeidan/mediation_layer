/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <vector>
#include <string>
#include "mediation_layer/helper.h"
#include "mediation_layer/visualization_functions.h"

// Line parameterized as l = p0 + t.vec, t belongs to (-inf,inf)
// This class is used in point compression algorithms
class Line3d{
 public:
    Eigen::Vector3d p0_;
    Eigen::Vector3d vec_;

    // Constructor: returns line that goes through two points p1, p2
    Line3d(const Eigen::Vector3d &p1,
            const Eigen::Vector3d &p2) {
        p0_ = p1;
        vec_ = p1 - p2;
    }

    // Methods
    // Calculate the distance between one point and the line
    void DistancePoint2Line(const Eigen::Vector3d &point,
                            double *dist) {
        // Two points on the line
        const Eigen::Vector3d x1 = p0_;
        const Eigen::Vector3d x2 = p0_ + vec_;

        // Optimal t is the t at which the point is closest to the line
        // t_opt = (x1-x2)'*(x1-p)/norm(x1-x2)^2;
        static double t_opt;
        if (x1 == x2) {
            t_opt = 0;
        } else {
            const double gain = 1.0/(pow((x1-x2).norm(), 2.0));
            t_opt = gain*(x1-x2).transpose()*(x1-point);
        }

        // p_opt is the closest point between the point and the line
        const Eigen::Vector3d p_opt = x1 + t_opt*vec_;

        *dist = (point - p_opt).norm();
    }
};

// Plane parameterized as an origin and a normal vector
class Plane3d{
 public:
    Eigen::Vector3d origin_;
    Eigen::Vector3d normal_;

    // Constructor: plane given by an origin and a normal
    Plane3d(const Eigen::Vector3d &origin,
            const Eigen::Vector3d &normal) {
        origin_ = origin;
        if (normal.norm() == 0) {
            std::cout << "Warning: plane ill-defined: "
                      << "zero-norm normal vector!"
                      << std::endl;
            normal_ << 0.0, 0.0, 0.0;
        } else {
            normal_ = normal.normalized();
        }
    }
    // Plane given by three points.
    // Normal is in the direction of (p2-p1)x(p3-p1)
    Plane3d(const Eigen::Vector3d &p1,
            const Eigen::Vector3d &p2,
            const Eigen::Vector3d &p3) {
        const Eigen::Vector3d v1 = p2 - p1;
        const Eigen::Vector3d v2 = p3 - p1;
        // const Eigen::Vector3d v3 = (p1 + p2 + p3)/3.0;
        if ((v1.norm() == 0) || (v2.norm() == 0) || ((p2-p3).norm() == 0)) {
            std::cout << "Warning: plane ill-defined: "
                      << "three distinct points are needed!"
                      << std::endl;
            origin_ = p1;
            normal_ << 0.0, 0.0, 0.0;
        } else {
            origin_ = p1;
            normal_ = v1.cross(v2).normalized();
        }
    }

    Plane3d() {}

    void TransformPlane(const Eigen::Affine3d &transform,
                        Plane3d *transformedPlane) {
        transformedPlane->origin_ = transform*origin_;
        transformedPlane->normal_ = (transform.linear()*normal_).normalized();
    }

    // This function returns a positive value if point is in the
    // direction of the normal, and a negative value if it is
    // in the opposite direction of the normal
    void DistancePoint2Plane(const Eigen::Vector3d &point,
                             double *dist) {
        // Get distance between point and origin
        const Eigen::Vector3d dist_point_origin = point - origin_;

        // Project the calculated distance into the normal vector
        *dist =  dist_point_origin.dot(normal_);
    }

    // This function returns a positive distance if point is in the
    // direction of the normal, and a negative value if it is
    // in the opposite direction of the normal
    void ProjectPointOntoPlane(const Eigen::Vector3d &point,
                               Eigen::Vector3d *nearest_point,
                               double *dist) {
        this->DistancePoint2Plane(point, dist);
        *nearest_point = point - (*dist)*normal_;
    }
};

// Walls are planes with boundaries
class Wall{
 public:
    Plane3d plane_;
    Eigen::Vector3d direction1_, direction2_;
    double width_, height_;

    // Constructors ----------------------------------------------
    Wall() { }

    // Construct wall from an origin, a normal, and two directions
    // height is along direction1, while width is along direction2
    // (make sure that normal, direction1, and direction2 
    //  are mutually perpendicular!!!!)
    Wall(const Eigen::Vector3d &origin,
         const Eigen::Vector3d &normal,
         const Eigen::Vector3d &direction1,
         const Eigen::Vector3d &direction2,
         const double height,
         const double width) {
        plane_ = Plane3d(origin, normal);
        direction1_ = direction1;
        direction2_ = direction2;
        height_ = height;
        width_ = width;
    }

    // Calculates distance to wall plane
    // This function returns a positive value if point is in the
    // direction of the normal, and a negative value if it is
    // in the opposite direction of the normal
    void DistancePoint2WallPlane(const Eigen::Vector3d &point,
                                 double *dist) {
        plane_.DistancePoint2Plane(point, dist);
    }
};


// Create box with normals facing inwards
class BoxPlanes{
 public:
    std::string name_;
    Plane3d bottom_plane_;
    Plane3d upper_plane_;
    Plane3d left_plane_;
    Plane3d right_plane_;
    Plane3d back_plane_;
    Plane3d front_plane_;
    Eigen::Vector3d lower_corner_;
    Eigen::Vector3d upper_corner_;
    Eigen::Vector3d ULB_, URB_, ULF_, URF_, DLB_, DRB_, DLF_, DRF_;
    // U = Up, D = Down, R = Right, L = Left, B = Back, F = Front

    // Constructors ----------------------------------------------
    BoxPlanes() { }

    // Construct box from opposite corners
    BoxPlanes(const Eigen::Vector3d corner1,
              const Eigen::Vector3d corner2,
              const std::string name){
        const double min_x = std::min(corner1[0], corner2[0]);
        const double max_x = std::max(corner1[0], corner2[0]);
        const double min_y = std::min(corner1[1], corner2[1]);
        const double max_y = std::max(corner1[1], corner2[1]);
        const double min_z = std::min(corner1[2], corner2[2]);
        const double max_z = std::max(corner1[2], corner2[2]);

        // Set lower and upper corners
        lower_corner_ = Eigen::Vector3d(min_x, min_y, min_z);
        upper_corner_ = Eigen::Vector3d(max_x, max_y, max_z);

        // Create a plane for each face of the box
        bottom_plane_ = Plane3d(lower_corner_, Eigen::Vector3d(0.0, 0.0, 1.0));
        right_plane_ = Plane3d(lower_corner_, Eigen::Vector3d(0.0, 1.0, 0.0));
        back_plane_ = Plane3d(lower_corner_, Eigen::Vector3d(1.0, 0.0, 0.0));
        upper_plane_ = Plane3d(upper_corner_, Eigen::Vector3d(0.0, 0.0, -1.0));
        left_plane_ = Plane3d(upper_corner_, Eigen::Vector3d(0.0, -1.0, 0.0));
        front_plane_ = Plane3d(upper_corner_, Eigen::Vector3d(-1.0, 0.0, 0.0));

        // Set all vertex of the box
        ULB_ = Eigen::Vector3d(min_x, min_y, max_z);
        URB_ = Eigen::Vector3d(min_x, max_y, max_z);
        ULF_ = Eigen::Vector3d(max_x, min_y, max_z);
        URF_ = Eigen::Vector3d(max_x, max_y, max_z);
        DLB_ = Eigen::Vector3d(min_x, min_y, min_z);
        DRB_ = Eigen::Vector3d(min_x, max_y, min_z);
        DLF_ = Eigen::Vector3d(max_x, min_y, min_z);
        DRF_ = Eigen::Vector3d(max_x, max_y, min_z);

        // Set name of the box
        name_ = name;
    }

    // Methods ---------------------------------------------------
    
    // Returns distance between point and all the planes with their
    // respective normal directions
    void DistancePoint2Planes(const Eigen::Vector3d &point,
                              std::vector<double> *dist,
                              std::vector<Eigen::Vector3d> *normal) {
        std::vector<double> local_dist;
        std::vector<Eigen::Vector3d> local_normal;
        local_dist.resize(6);
        local_normal.resize(6);

        bottom_plane_.DistancePoint2Plane(point, &local_dist[0]);
        local_normal[0] = bottom_plane_.normal_;
        upper_plane_.DistancePoint2Plane(point, &local_dist[1]);
        local_normal[1] = upper_plane_.normal_;
        left_plane_.DistancePoint2Plane(point, &local_dist[2]);
        local_normal[2] = left_plane_.normal_;
        right_plane_.DistancePoint2Plane(point, &local_dist[3]);
        local_normal[3] = right_plane_.normal_;
        back_plane_.DistancePoint2Plane(point, &local_dist[4]);
        local_normal[4] = back_plane_.normal_;
        front_plane_.DistancePoint2Plane(point, &local_dist[5]);
        local_normal[5] = front_plane_.normal_;

        *dist = local_dist;
        *normal = local_normal;
    }

    // Return visualization markers for box visualization
    void VisualizeBox(const std::string frame_id,
                      visualization_msgs::MarkerArray *boxPlanes) {
        // Initialize array
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "box/" + name_;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.id = 0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = visualization_functions::Color::Black();
        marker.color.a = 0.4;

        // Back face
        visualization_functions::SetPlanePoints(ULB_, DRB_, URB_, DLB_, marker.color, &marker);
        visualization_functions::SetPlanePoints(ULF_, DRF_, URF_, DLF_, marker.color, &marker);
        visualization_functions::SetPlanePoints(DLB_, ULF_, ULB_, DLF_, marker.color, &marker);
        visualization_functions::SetPlanePoints(DRB_, URF_, URB_, DRF_, marker.color, &marker);

        boxPlanes->markers.push_back(marker);
    }
};

class FrustumPlanes{
 public:
    Plane3d left_plane_;
    Plane3d right_plane_;
    Plane3d up_plane_;
    Plane3d down_plane_;
    Eigen::Vector3d UL_, UR_, DL_, DR_, origin_;

    FrustumPlanes(const double fov,
                  const double aspectRatio) {
        double znear = 1.0;
        double hh = tan(fov/2.0);
        double hw = -hh*aspectRatio;
        double normalize = 1.0/Eigen::Vector3d(hw, hh, znear).norm();
        UL_ = normalize*Eigen::Vector3d(-hw, hh, znear);
        UR_ = normalize*Eigen::Vector3d(hw, hh, znear);
        DL_ = normalize*Eigen::Vector3d(-hw, -hh, znear);
        DR_ = normalize*Eigen::Vector3d(hw, -hh, znear);
        origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        left_plane_ = Plane3d(origin_, DL_, UL_);
        right_plane_ = Plane3d(origin_, UR_, DR_);
        up_plane_ = Plane3d(origin_, UL_, UR_);
        down_plane_ = Plane3d(origin_, DR_, DL_);
    }

    FrustumPlanes(const double fx,
                  const double fy,
                  const double cx,
                  const double cy,
                  const uint32_t width,
                  const uint32_t height) {
        double znear = 1.0;
        double x0 = 0.0, y0 = 0.0;
        double xf = static_cast<float>(width) - 1;
        double yf = static_cast<float>(height) - 1;
        double XL = -znear*(x0-cx)/fx;  // X left
        double XR = -znear*(xf-cx)/fx;  // X right
        double YU = -znear*(y0-cy)/fy;  // Y up
        double YD = -znear*(yf-cy)/fy;  // Y down
        UL_ = Eigen::Vector3d(XL, YU, znear).normalized();
        UR_ = Eigen::Vector3d(XR, YU, znear).normalized();
        DL_ = Eigen::Vector3d(XL, YD, znear).normalized();
        DR_ = Eigen::Vector3d(XR, YD, znear).normalized();
        origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        left_plane_ = Plane3d(origin_, DL_, UL_);
        right_plane_ = Plane3d(origin_, UR_, DR_);
        up_plane_ = Plane3d(origin_, UL_, UR_);
        down_plane_ = Plane3d(origin_, DR_, DL_);
    }

    FrustumPlanes() {}

    void TransformFrustum(const Eigen::Affine3d &transform,
                          FrustumPlanes *transformed_frustum) {
        left_plane_.TransformPlane(transform, &transformed_frustum->left_plane_);
        right_plane_.TransformPlane(transform, &transformed_frustum->right_plane_);
        up_plane_.TransformPlane(transform, &transformed_frustum->up_plane_);
        down_plane_.TransformPlane(transform, &transformed_frustum->down_plane_);
        transformed_frustum->origin_ = transform*origin_;
        transformed_frustum->UL_ = transform*UL_;
        transformed_frustum->UR_ = transform*UR_;
        transformed_frustum->DL_ = transform*DL_;
        transformed_frustum->DR_ = transform*DR_;
    }

    bool IsPointWithinFrustum(const Eigen::Vector3d &pt) const {
        if ((pt - left_plane_.origin_).dot(left_plane_.normal_) < 0) {
            return false;
        } else if ((pt - right_plane_.origin_).dot(right_plane_.normal_) < 0) {
            return false;
        } else if ((pt - up_plane_.origin_).dot(up_plane_.normal_) < 0) {
            return false;
        } else if ((pt - down_plane_.origin_).dot(down_plane_.normal_) < 0) {
            return false;
        } else {
            return true;
        }
    }

    // Return visualization markers frustum visualization
    void VisualizeFrustum(const std::string &frame_id,
                          visualization_msgs::Marker *line_list) {
        // Initialize array
        line_list->header.frame_id = frame_id;
        line_list->header.stamp = ros::Time::now();
        line_list->ns = "fustrum/" + frame_id;
        line_list->action = visualization_msgs::Marker::ADD;
        line_list->pose.orientation.w = 1.0;
        line_list->type = visualization_msgs::Marker::LINE_LIST;
        line_list->id = 0;
        line_list->scale.x = 0.01;  // Line width
        line_list->color = visualization_functions::Color::Red();

        std::vector<Eigen::Vector3d> points;
        points.push_back(origin_);
        points.push_back(UL_);
        points.push_back(UR_);
        points.push_back(DL_);
        points.push_back(DR_);

        geometry_msgs::Point node1, node2;
        for (uint i = 0; i < points.size(); i++) {
            for (uint j = 0; j < points.size(); j++) {
                if (i == j) {
                    continue;
                }
                node1 = helper::Vec3d2point(points[i]);
                node2 = helper::Vec3d2point(points[j]);
                // EigenPoint2RosPoint(points[i], &node1);
                // EigenPoint2RosPoint(points[j], &node2);
                line_list->points.push_back(node1);
                line_list->points.push_back(node2);
            }
        }
    }
};

#endif  // LINEAR_ALGEBRA_H_
