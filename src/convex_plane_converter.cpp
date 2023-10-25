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

bool ConvexPlaneConverter::fromMessage(
    const convex_plane_msgs::msg::ConvexPlanesWithGridMap& message, 
    grid_map::GridMap& map, std::vector<iris::IRISRegion>& regions, 
    std::vector<Vector>& normals, std::vector<int>& labels
)
{
    // create grid map
    grid_map::GridMapRosConverter::fromMessage(message.map, map);

    // create regions, normals and labels
    const convex_plane_msgs::msg::ConvexPlanes& planes = message.plane;

    
    normals.resize(planes.label.size());
    labels = planes.label;
    for (size_t i=0; i<planes.label.size(); ++i)
    {
        regions.emplace_back(2);
        mulitArrayMessage2Eigen(planes.a_matrix[i], regions[i].polyhedron.getAref());
        mulitArrayMessage2Eigen(planes.b_vector[i], regions[i].polyhedron.getBref());
        mulitArrayMessage2Eigen(planes.c_matrix[i], regions[i].ellipsoid.getCref());
        mulitArrayMessage2Eigen(planes.d_vector[i], regions[i].ellipsoid.getDref());
        mulitArrayMessage2Eigen(planes.normal[i], normals[i]);
    }

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

bool ConvexPlaneConverter::addPlaneToMessage(
    const Matrix& A, const Vector& b, 
    const Matrix& C, const Vector& d,
    const Vector& normal, const int label,
    convex_plane_msgs::msg::ConvexPlanes& planes
)
{
    planes.a_matrix.push_back(eigen2MultiArrayMessage(A));
    planes.b_vector.push_back(eigen2MultiArrayMessage(b));
    planes.c_matrix.push_back(eigen2MultiArrayMessage(C));
    planes.d_vector.push_back(eigen2MultiArrayMessage(d));
    planes.normal.push_back(eigen2MultiArrayMessage(normal));
    planes.label.push_back(label);

    return true;
}

bool ConvexPlaneConverter::addPlaneToMessage(
    const iris::IRISRegion& region, const Vector& normal, 
    const int label,
    convex_plane_msgs::msg::ConvexPlanes& planes
)
{
    addPlaneToMessage(region.polyhedron.getA(), region.polyhedron.getB(), region.ellipsoid.getC(), region.ellipsoid.getD(), normal, label, planes);
    return true;
}

convex_plane_msgs::msg::ConvexPlanesWithGridMap::UniquePtr ConvexPlaneConverter::toMessage(
    const std::vector<iris::IRISRegion>& regions, 
    const std::vector<Vector>& normals, const std::vector<float>& labels, 
    const grid_map::GridMap& map
)
{
    return toMessage(regions, normals, labels, map, map.getLayers());
}

convex_plane_msgs::msg::ConvexPlanesWithGridMap::UniquePtr ConvexPlaneConverter::toMessage(
    const std::vector<iris::IRISRegion>& regions, 
    const std::vector<Vector>& normals, const std::vector<float>& labels, 
    const grid_map::GridMap& map, const std::vector<std::string>& layers
)
{
    convex_plane_msgs::msg::ConvexPlanesWithGridMap::UniquePtr
        message = std::make_unique<convex_plane_msgs::msg::ConvexPlanesWithGridMap>();
    
    // create grid_map_msgs
    grid_map::GridMapRosConverter::toMessage(map, layers, message->map);

    // create convex_plane_msgs
    convex_plane_msgs::msg::ConvexPlanes& planes = message->plane;

    // resize
    planes.a_matrix.resize(labels.size());
    planes.b_vector.resize(labels.size());
    planes.c_matrix.resize(labels.size());
    planes.d_vector.resize(labels.size());
    planes.normal.resize(labels.size());
    planes.label.resize(labels.size());
    for (size_t i=0; i<labels.size(); ++i)
    {
        eigen2MultiArrayMessage(regions[i].polyhedron.getA(), planes.a_matrix[i]);
        eigen2MultiArrayMessage(regions[i].polyhedron.getB(), planes.b_vector[i]);
        eigen2MultiArrayMessage(regions[i].ellipsoid.getC(), planes.c_matrix[i]);
        eigen2MultiArrayMessage(regions[i].ellipsoid.getD(), planes.d_vector[i]);
        eigen2MultiArrayMessage(normals[i], planes.normal[i]);
        planes.label[i] = labels[i];
    }

    return message;
}

}