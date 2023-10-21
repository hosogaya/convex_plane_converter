#pragma once

#include <eigen3/Eigen/Core>
#include <convex_plane_msgs/msg/convex_plane.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

namespace convex_plane
{

class ConvexPlaneConverter
{
public:
    ConvexPlaneConverter();
    ~ConvexPlaneConverter();

    static bool fromMessage(
        const convex_plane_msgs::msg::ConvexPlane& message, 
        Eigen::MatrixXd& A, Eigen::VectorXd& b, 
        Eigen::MatrixXd& C, Eigen::VectorXd& d,
        Eigen::VectorXd& normal
    );

    static bool fromMessage(
        const convex_plane_msgs::msg::ConvexPlane& message, 
        Eigen::MatrixXd& A, Eigen::VectorXd& b, 
        Eigen::VectorXd& normal
    );

    static bool toMessge(
        const Eigen::MatrixXd& A, const Eigen::VectorXd& b, 
        const Eigen::MatrixXd& C, const Eigen::VectorXd& c,
        const Eigen::MatrixXd& normal, 
        const convex_plane_msgs::msg::ConvexPlane& message
    );
};
}