#include <rclcpp/rclcpp.hpp>
#include <convex_plane_msgs/msg/convex_plane.hpp>
#include <convex_plane_msgs/srv/get_convex_plane.hpp>
#include <chrono>
using namespace std::chrono_literals;

template<typename MultiArraytype_>
void printMultiArray(const MultiArraytype_& m, const std::string& name)
// void printMultiArray(const std_msgs::msg::Float32MultiArray& m, const std::string& name)
{
    std::cout << name << ": " << std::endl;
    std::cout << "dim size: " << m.layout.dim.size() << std::endl;
    for (int i=0; i<m.layout.dim.size(); ++i)
    {
        std::cout << "message dim stride" << i << ": " << m.layout.dim[i].stride << std::endl;
        std::cout << "message dim size" << i << ": " << m.layout.dim[i].size << std::endl;
        std::cout << "message dim label" << i << ": " << m.layout.dim[i].label << std::endl;
    }
    std::cout << "message data: " << std::endl;
    for (size_t i=0; i<m.layout.dim[0].size; ++i)
    {
        for (size_t j=0; j<m.layout.dim[1].size; ++j)
        {
            std::cout << m.data.at(i*m.layout.dim[1].size + j) << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_convex_plane_client", rclcpp::NodeOptions().use_intra_process_comms(true)); 
    rclcpp::Client<convex_plane_msgs::srv::GetConvexPlane>::SharedPtr client =                        
        node->create_client<convex_plane_msgs::srv::GetConvexPlane>("get_convex_plane");
    
    auto request = std::make_shared<convex_plane_msgs::srv::GetConvexPlane_Request>();
    RCLCPP_INFO(rclcpp::get_logger("client"), "0x%x", &(request->map));
    RCLCPP_INFO(rclcpp::get_logger("client"), "0x%x", &(request->label));

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success");
        printMultiArray(result.get()->plane.a_matrix, "A matrix");
        printMultiArray(result.get()->plane.b_vector, "b vector");
        printMultiArray(result.get()->plane.c_matrix, "C matrix");
        printMultiArray(result.get()->plane.d_vector, "d vector");
        printMultiArray(result.get()->plane.normal, "normal");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    
    }

    rclcpp::shutdown();
    return 0;
}