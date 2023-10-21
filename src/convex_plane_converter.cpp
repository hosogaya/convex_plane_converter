#include <convex_plane_converter/convex_plane_converter.h>

namespace convex_plane
{
ConvexPlaneConverter::ConvexPlaneConverter() {}
ConvexPlaneConverter::~ConvexPlaneConverter() {}

bool ConvexPlaneConverter::fromMessage(
    const convex_plane_msgs::msg::ConvexPlane& message, 
    Matrix& A, Vector& b, 
    Matrix& C, Vector& d,
    Vector& normal
)
{
    if (!mulitArrayMessage2Eigen(message.a_matrix, A)) return false;
    if (!mulitArrayMessage2Eigen(message.b_vector, b)) return false;
    if (!mulitArrayMessage2Eigen(message.c_matrix, C)) return false;
    if (!mulitArrayMessage2Eigen(message.d_vector, d)) return false;
    if (!mulitArrayMessage2Eigen(message.normal, normal)) return false;

    return true;
}

bool ConvexPlaneConverter::toMessge(
    const Matrix& A, const Vector& b, 
    const Matrix& C, const Vector& d,
    const Vector& normal, 
    convex_plane_msgs::msg::ConvexPlane& message
)
{
    if (!eigen2MultiArrayMessage(A, message.a_matrix)) return false;
    if (!eigen2MultiArrayMessage(b, message.b_vector)) return false;
    if (!eigen2MultiArrayMessage(C, message.c_matrix)) return false;
    if (!eigen2MultiArrayMessage(d, message.d_vector)) return false;
    if (!eigen2MultiArrayMessage(normal, message.normal)) return false;

    return true;
}
}