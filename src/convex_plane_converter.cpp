#include <convex_plane_converter/convex_plane_converter.h>

namespace convex_plane
{
ConvexPlaneConverter::ConvexPlaneConverter() {}
ConvexPlaneConverter::~ConvexPlaneConverter() {}

bool ConvexPlaneConverter::fromMessage(
    const convex_plane_msgs::msg::ConvexPlane& message, 
    Eigen::MatrixXd& A, Eigen::VectorXd& b, 
    Eigen::MatrixXd& C, Eigen::VectorXd& d,
    Eigen::VectorXd& normal
)
{
    
}
}