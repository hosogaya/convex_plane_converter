#include <rclcpp/rclcpp.hpp>
#include <convex_plane_msgs/msg/convex_plane.hpp>
#include <convex_plane_msgs/srv/get_convex_plane.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>

void get(
    const convex_plane_msgs::srv::GetConvexPlane::Request::SharedPtr request,
    convex_plane_msgs::srv::GetConvexPlane::Response::SharedPtr response
)
{
    // grid_map::GridMap map;
    // grid_map::GridMapRosConverter::fromMessage(request->map, map);
    
    response->plane.a_matrix.layout.dim.resize(1);
    response->plane.a_matrix.layout.dim[0].label = "A_matrix";

    response->plane.b_vector.layout.dim.resize(1);
    response->plane.b_vector.layout.dim[0].label = "b_vector";

    response->plane.c_matrix.layout.dim.resize(1);
    response->plane.c_matrix.layout.dim[0].label = "C_matrix";

    response->plane.d_vector.layout.dim.resize(1);
    response->plane.d_vector.layout.dim[0].label = "d_vector";

    response->plane.normal.layout.dim.resize(1);
    response->plane.normal.layout.dim[0].label = "normal";
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_convex_plane_server");  // CHANGE

    rclcpp::Service<convex_plane_msgs::srv::GetConvexPlane>::SharedPtr service =                 // CHANGE
        node->create_service<convex_plane_msgs::srv::GetConvexPlane>("get_convex_plane",  &get);     // CHANGE

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to get convex plane.");      // CHANGE

    rclcpp::spin(node);
    rclcpp::shutdown();
}