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

class BalloonSet {
 public:
    std::vector<Balloon> balloons_;
    double pop_distance_ = 0.25;     // Minimum distance to pop a balloon
    bool all_unpopped_ = true;

    BalloonSet() {};

    BalloonSet(const double &pop_distance) {
        pop_distance_ = pop_distance;
    }

    void AddBalloon(const std::string name,
                    const std_msgs::ColorRGBA &color,
                    const Eigen::Vector3d position) {
        balloons_.push_back(Balloon(name, color, position));
    }

    // Pop balloons that are close enough to quads
    void PopBalloons(const std::vector<Eigen::Vector3d> &quad_positions) {
        for (uint i = 0; i < balloons_.size(); i++) {
            // We don't check if balloon is already popped
            if (balloons_[i].is_popped()) {
                continue;
            }

            for (uint j = 0; j < quad_positions.size(); j++) {

                // If balloon isn't popped, we check proximity with quads
                const double dist = 
                    (balloons_[i].position_ - quad_positions[j]).norm();
                if (dist < pop_distance_) {
                    balloons_[i].popped_ = true;
                    all_unpopped_ = false;
                    ROS_INFO("[mediation layer]: %s balloon popped!", 
                             balloons_[i].name_.c_str());
                }
            }
        }
    }

    // Set all balloons to "unpopped"
    void ResetBalloons() {
        if(!all_unpopped_) {
            for (uint i = 0; i < balloons_.size(); i++) {
                balloons_[i].popped_ = false;
                all_unpopped_ = true;
                ROS_INFO("[mediation layer]: Balloons were healed!");
            }
        }
    }
};

#endif  // BALLOON_H_
