#pragma once

#include <eigen3/Eigen/Core>
#include <convex_plane_msgs/msg/convex_plane.hpp>
#include <convex_plane_converter/convex_plane_msgs_hlper.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <convex_plane_msgs/msg/convex_planes_with_grid_map.hpp>
#include <iris_2d/iris_2d.h>

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
    //     const convex_plane_msgs::msg::ConvexPlanes& message,
    //     const std::vector<iris_2d::Region>& regions,
    //     const std::vector<Vector>& normal,
    //     const std::vector<int>& label  
    // );

    static bool fromMessage(
        const convex_plane_msgs::msg::ConvexPlanesWithGridMap& message, 
        grid_map::GridMap& map, std::vector<iris_2d::Region>& regions, 
        std::vector<Vector>& normals, std::vector<int>& labels
    );

    static bool toMessge(
        const Matrix& A, const Vector& b, 
        const Matrix& C, const Vector& d,
        const Vector& normal, 
        convex_plane_msgs::msg::ConvexPlane& message
    );

    static bool addPlaneToMessage(
        const Matrix& A, const Vector& b, 
        const Matrix& C, const Vector& d,
        const Vector& normal, const int label,
        convex_plane_msgs::msg::ConvexPlanes& planes
    );

    static bool addPlaneToMessage(
        const iris_2d::Region& region, const Vector& normal, 
        const int label,
        convex_plane_msgs::msg::ConvexPlanes& planes
    );

    static convex_plane_msgs::msg::ConvexPlanesWithGridMap::UniquePtr toMessage(
        const std::vector<iris_2d::Region>& regions, 
        const std::vector<Vector>& normals, const std::vector<float>& label, 
        const grid_map::GridMap& map
    );

    static convex_plane_msgs::msg::ConvexPlanesWithGridMap::UniquePtr toMessage(
        const std::vector<iris_2d::Region>& regions, 
        const std::vector<Vector>& normals, const std::vector<float>& label, 
        const grid_map::GridMap& map, const std::vector<std::string>& layers
    );
};
}