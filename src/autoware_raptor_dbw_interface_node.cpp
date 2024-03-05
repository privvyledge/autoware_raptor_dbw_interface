#include "autoware_raptor_dbw_interface/ne_raptor_interface_node.hpp"
#include "autoware_raptor_dbw_interface/ne_raptor_interface.hpp"

#include <autoware_auto_common/common/types.hpp>

#include <memory>
#include <string>
#include <unordered_set>

namespace autoware_raptor_dbw_interface
{
    using autoware_auto_common::common::types::float32_t;
    RaptorDBWInterfaceNode::RaptorDBWInterfaceNode(const rclcpp::NodeOptions & options)
    : Node("autoware_raptor_dbw_interface", options)
    {
        declare_parameter("ne_raptor.ecu_build_num").get<uint16_t>(),
        declare_parameter("ne_raptor.front_axle_to_cog").get<float32_t>(),
        declare_parameter("ne_raptor.rear_axle_to_cog").get<float32_t>(),
        declare_parameter("ne_raptor.steer_to_tire_ratio").get<float32_t>(),
        declare_parameter("ne_raptor.max_steer_angle").get<float32_t>(),
//        get_state_machine().get_config().accel_limits().max(),
//        get_state_machine().get_config().accel_limits().min(),
        declare_parameter("ne_raptor.acceleration_positive_jerk_limit").get<float32_t>(),
        declare_parameter("ne_raptor.deceleration_negative_jerk_limit").get<float32_t>(),
        declare_parameter("ne_raptor.pub_period").get<uint32_t>()
    }

}  // namespace autoware_raptor_dbw_interface
#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_raptor_dbw_interface::RaptorDBWInterfaceNode)