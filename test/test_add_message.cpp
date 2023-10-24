#include <convex_plane_converter/convex_plane_converter.h>
#include <iostream>

template<typename Eigentype_>
void printMatrix(const Eigentype_& e, const std::string& name)
{
    std::cout << name << ": " << std::endl;
    std::cout << e << std::endl;
}

template<typename MultiArraytype_>
void printMultiArray(const MultiArraytype_& m, const std::string& name)
// void printMultiArray(const std_msgs::msg::Float32MultiArray& m, const std::string& name)
{
    std::cout << name << ": " << std::endl;
    std::cout << "dim size: " << m.layout.dim.size() << std::endl;
    for (size_t i=0; i<m.layout.dim.size(); ++i)
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

int main()
{
    Eigen::VectorXd b(2), d(2);
    Eigen::MatrixXd A(2,2), C(2, 2);
    for (int i=0; i<A.rows(); ++i)
    {
        for (int j=0; j<A.cols(); ++j)
        {
            A(i,j) = i*A.cols() + j;
            C(i,j) = i*A.cols() + j;
        }
        b(i) = i;
        d(i) = i;
    }
    Eigen::VectorXd normal(3);
    normal << 0, 1, 2;
    printMatrix(A, "A matrix");
    printMatrix(b, "b vector");
    printMatrix(C, "C matrix");
    printMatrix(d, "d vector");
    printMatrix(normal, "normal");
    std::vector<int> labels(10);
    for (size_t i=0; i<labels.size(); ++i)
    {
    // convex_plane_msgs::msg::ConvexPlanes message;
    iris::IRISProblem problem(2);
    convex_plane_msgs::msg::ConvexPlanesWithGridMap message;
    iris::IRISRegion region(2);
    region.ellipsoid.setC(C);
    region.ellipsoid.setD(d);
    region.polyhedron.setA(A);
    region.polyhedron.setB(b);
    // convex_plane::ConvexPlaneConverter::addPlaneToMessage(A, b, C, d, normal, 1, message);
    convex_plane::ConvexPlaneConverter::addPlaneToMessage(region, normal, labels[i], message.plane);
    // printMultiArray(message.a_matrix[0], "A matrix");
    // printMultiArray(message.b_vector[0], "b vector");
    // printMultiArray(message.c_matrix[0], "C matrix");
    // printMultiArray(message.d_vector[0], "d vector");
    // printMultiArray(message.normal[0], "normal");
    }

    // if (!convex_plane::ConvexPlaneConverter::fromMessage(message, A, b, C, d, normal))
    // {
    //     std::cout << "fromMessage failed" << std::endl;
    //     return 1;
    // }
    // printMatrix(A, "A matrix");
    // printMatrix(b, "b vector");
    // printMatrix(C, "C matrix");
    // printMatrix(d, "d vector");
    // printMatrix(normal, "normal");
    
    return 0;
}