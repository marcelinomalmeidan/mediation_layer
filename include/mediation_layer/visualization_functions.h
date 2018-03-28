#ifndef MAPPER_VISUALIZATION_FUNCTIONS_H_
#define MAPPER_VISUALIZATION_FUNCTIONS_H_

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "mediation_layer/helper.h"


namespace visualization_functions {

// Some colors for visualization markers
class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color DarkGreen() { return Color(0.0, 0.3, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

void SelectColor(const uint &i, std_msgs::ColorRGBA *color);

void SelectColor(const std::string &des_color, std_msgs::ColorRGBA *color);

// Sphere of influence around a quadcopter
void SphereMarker(const Eigen::Vector3d &point,
                  const std::string &frame_id,
                  const std::string &ns,  // namespace
                  const double &size,
                  const std_msgs::ColorRGBA &color,
                  const double &transparency,  // 0 -> transparent, 1 -> opaque
                  const int &seqNumber,
                  visualization_msgs::MarkerArray *markerArray);

// Plane with normal along the heading direction
void PlaneMarker(const Eigen::Vector3d &point,
                 const std::string frame_id,
                 const std::string &ns,  // namespace
                 const double &size,
                 const std_msgs::ColorRGBA &color,
                 const double &transparency,  // 0 -> transparent, 1 -> opaque
                 const int &seqNumber,
                 const double heading_direction,
                 visualization_msgs::MarkerArray *markerArray);

// Cuboid with length along x, width along y, height along z, rotated
// around z with angle given by heading_direction
void CuboidMarker(const Eigen::Vector3d &point,
                  const std::string frame_id,
                  const std::string &ns,  // namespace
                  const double &length,
                  const double &width,
                  const double &height,
                  const double &heading_direction,
                  const std_msgs::ColorRGBA &color,
                  const double &transparency,  // 0 -> transparent, 1 -> opaque
                  const int &seqNumber,
                  visualization_msgs::MarkerArray *markerArray);

// Quadcopter mesh
void MeshMarker(const Eigen::Vector3d &point,
                const Eigen::Quaterniond &quat,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const double &size,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
                const int &seqNumber,
                visualization_msgs::MarkerArray *markerArray);

// Arrow representing forces acting on a quad
void ForceMarker(const Eigen::Vector3d &point,
                 const Eigen::Vector3d &force,
                 const double &max_accel,
                 const std::string &frame_id,
                 const std::string &ns,  // namespace
                 const std_msgs::ColorRGBA &color,
                 const int &seqNumber,
                 visualization_msgs::MarkerArray *markerArray);

// Quad name
void NameMarker(const Eigen::Vector3d &point,
                const std::string &quad_name,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const std_msgs::ColorRGBA &color,
                const int &seqNumber,
                visualization_msgs::MarkerArray *markerArray);

// xyz frame with origin at quadcopter position
void FrameMarker(const Eigen::Vector3d &point,
                 const Eigen::Matrix3d &Rot,
                 const std::string &frame_id,
                 const std::string &ns,  // namespace
                 const std_msgs::ColorRGBA &color,
                 const int &seqNumber,
                 const double length,
                 visualization_msgs::MarkerArray *markerArray);

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
                 visualization_msgs::MarkerArray *marker_array);

// Return triangle edges for plane visualizations.
// Every plane has two triangles
// corner1 is assumed to be in an opposite edge w.r.t. corner2
void SetPlanePoints(const Eigen::Vector3d corner1,
                    const Eigen::Vector3d corner2,
                    const Eigen::Vector3d corner3,
                    const Eigen::Vector3d corner4,
                    const std_msgs::ColorRGBA color,
                    visualization_msgs::Marker *marker);

}


#endif  // MAPPER_VISUALIZATION_FUNCTIONS_H_