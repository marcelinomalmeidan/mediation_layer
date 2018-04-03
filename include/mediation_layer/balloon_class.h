#ifndef BALLOON_H_
#define BALLOON_H_

#include <Eigen/Dense>


class Balloon {
 public:
    std::string name_;
    std_msgs::ColorRGBA color_;
    Eigen::Vector3d position_;
    bool popped_;

    Balloon( ) { };

    Balloon(const std::string name,
            const std_msgs::ColorRGBA &color,
            const Eigen::Vector3d position) {
        name_ = name;
        color_ = color;
        position_ = position;
        popped_ = false;
    }

    bool is_popped() {
        return popped_;
    }
};

#endif  // BALLOON_H_
