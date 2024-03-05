#ifndef AUTOWARE_RAPTOR_DBW_INTERFACE__AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HPP_
#define AUTOWARE_RAPTOR_DBW_INTERFACE__AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HPP_

#include <autoware_raptor_dbw_interface/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware_raptor_dbw_interface
{
    class AUTOWARE_RAPTOR_DBW_INTERFACE_PUBLIC RaptorDBWInterfaceNode
    {
        public:
            explicit RaptorDBWInterfaceNode(const rclcpp::NodeOptions & options);

    };  // class RaptorDBWInterface

}  // namespace autoware_raptor_dbw_interface

##endif // AUTOWARE_RAPTOR_DBW_INTERFACE__AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HPP_