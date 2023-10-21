#include <convex_plane_converter/convex_plane_msgs_hlper.h>
#include <iostream>

int main()
{
    Eigen::VectorXf mat(10);
    std::cout << "source mat: " << std::endl;
    for (int i=0; i<mat.rows(); ++i)
    {
        for (int j=0; j<mat.cols(); ++j)
        {
            mat(i,j) = i*mat.cols()+j;
            std::cout << mat(i,j) << " ";
        }
        std::cout << std::endl;
    }
    std_msgs::msg::Float32MultiArray message;

    convex_plane::eigen2MultiArrayMessage(mat, message);
    std::cout << "message dim size: " << message.layout.dim.size() << std::endl;
    std::cout << "message dim stride0: " << message.layout.dim[0].stride << std::endl;
    std::cout << "message dim size0: " << message.layout.dim[0].size << std::endl;
    std::cout << "message dim stride1: " << message.layout.dim[1].stride << std::endl;
    std::cout << "message dim size1: " << message.layout.dim[1].size << std::endl;
    std::cout << "message dim label0: " << message.layout.dim[0].label << std::endl;
    std::cout << "message dim label1: " << message.layout.dim[1].label << std::endl;
    std::cout << "message data: " << std::endl;
    for (int i=0; i<message.layout.dim[0].size; ++i)
    {
        for (int j=0; j<message.layout.dim[1].size; ++j)
        {
            std::cout << message.data.at(i*message.layout.dim[1].size + j) << " ";
        }
        std::cout << std::endl;
    }
    convex_plane::mulitArrayMessage2Eigen(message, mat);
    std::cout << "dest mat: " << std::endl;
    for (int i=0; i<mat.rows(); ++i)
    {
        for (int j=0; j<mat.cols(); ++j)
        {
            std::cout << mat(i,j) << " ";
        }
        std::cout << std::endl;
    }


    return 0;
}