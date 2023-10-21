#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace convex_plane
{
static const std::string row_major = "row";
static const std::string col_major = "col";

template<typename MultiArrayMessageType_>
bool isRowMajor(const MultiArrayMessageType_& m)
{
    return m.layout.dim[0].label == row_major;
}

template<typename MultiArrayMessageType_>
int getRows(const MultiArrayMessageType_& m)
{
    if (isRowMajor(m)) {return m.layout.dim[0].size;}
    return m.layout.dim[1].size;
}

template<typename MultiArrayMessageType_>
int getCols(const MultiArrayMessageType_& m)
{
    if (isRowMajor(m)) {return m.layout.dim[1].size;}
    return m.layout.dim[0].size;
}

template<typename EigenType_, typename MultiArrayMessageType_>
bool mulitArrayMessage2Eigen(
    const MultiArrayMessageType_ &m, 
    EigenType_& e
)
{
    if (e.IsRowMajor != isRowMajor(m)) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("multiArrayMessage2Eigen"), "failed because the storage order is not compatible");
        return false;
    }

    e.resize(getRows(m), getCols(m));
    for (int i=0; i<m.data.size(); ++i)
        e.data()[i] = m.data[i];
    return true;
}

template<typename EigenType_, typename MultiArrayMessageType_>
bool eigen2MultiArrayMessage(
    const EigenType_& e, 
    MultiArrayMessageType_& m
)
{
    m.layout.dim.resize(2);
    m.layout.dim[0].stride = e.size();
    m.layout.dim[0].size = e.outerSize();
    m.layout.dim[1].stride = e.innerSize();
    m.layout.dim[1].size = e.innerSize();

    if (e.IsRowMajor)
    {
        m.layout.dim[0].label = row_major;
        m.layout.dim[1].label = col_major;
    }
    else
    {
        m.layout.dim[0].label = col_major;
        m.layout.dim[1].label = row_major;
    }

    m.data.insert(m.data.begin()+m.layout.data_offset, e.data(), e.data()+e.size());
    return true;
}
}