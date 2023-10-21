#pragma once

#include <eigen3/Eigen/Core>
#include <convex_plane_msgs/msg/convex_plane.hpp>
#include <convex_plane_converter/convex_plane_msgs_hlper.h>
#include <grid_map_msgs/msg/grid_map.hpp>

namespace convex_plane
{
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
class ConvexPlaneConverter
{
public:
    ConvexPlaneConverter();
    ~ConvexPlaneConverter();

    static bool fromMessage(
        const convex_plane_msgs::msg::ConvexPlane& message, 
        Matrix& A, Vector& b, 
        Matrix& C, Vector& d,
        Vector& normal
    );

    // static bool fromMessage(
    //     const convex_plane_msgs::msg::ConvexPlane& message, 
    //     Matrix& A, Vector& b, 
    //     Vector& normal
    // );

    static bool toMessge(
        const Matrix& A, const Vector& b, 
        const Matrix& C, const Vector& d,
        const Vector& normal, 
        convex_plane_msgs::msg::ConvexPlane& message
    );
};
}