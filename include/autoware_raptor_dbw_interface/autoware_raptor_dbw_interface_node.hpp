#ifndef AUTOWARE_RAPTOR_DBW_INTERFACE__AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HPP_
#define AUTOWARE_RAPTOR_DBW_INTERFACE__AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HPP_

//#include <autoware_raptor_dbw_interface/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware_raptor_dbw_interface
{
    class AUTOWARE_RAPTOR_DBW_INTERFACE_PUBLIC RaptorDBWInterfaceNode : public rclcpp::Node
    {
        public:
            explicit RaptorDBWInterfaceNode(const rclcpp::NodeOptions & options);

        private:
            // Publishers (to Raptor DBW)
            rclcpp::Publisher<AcceleratorPedalCmd>::SharedPtr m_accel_cmd_pub;
            rclcpp::Publisher<BrakeCmd>::SharedPtr m_brake_cmd_pub;
            rclcpp::Publisher<GearCmd>::SharedPtr m_gear_cmd_pub;
            rclcpp::Publisher<GlobalEnableCmd>::SharedPtr m_gl_en_cmd_pub;
            rclcpp::Publisher<MiscCmd>::SharedPtr m_misc_cmd_pub;
            rclcpp::Publisher<SteeringCmd>::SharedPtr m_steer_cmd_pub;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_dbw_enable_cmd_pub;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_dbw_disable_cmd_pub;

            // Publishers (to Autoware)
            rclcpp::Publisher<VehicleKinematicState>::SharedPtr m_vehicle_kin_state_pub; // todo: might remove
            rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;  // gear_rpt_pub_
            rclcpp::Publisher<ControlModeReport>::SharedPtr control_mode_rpt_pub_;
            rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_rpt_pub_;
            rclcpp::Publisher<VelocityReport>::SharedPtr velocity_rpt_pub_;
            rclcpp::Publisher<BatteryStatus>::SharedPtr battery_rpt_pub_;
            rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_pub_;

            // Subscribers (from Raptor DBW)
            rclcpp::SubscriptionBase::SharedPtr
                m_brake_rpt_sub, m_gear_rpt_sub, m_misc_rpt_sub,
                m_other_acts_rpt_sub, m_steering_rpt_sub, m_wheel_spd_rpt_sub;

            // Subscribers (from Autoware)
            rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
            rclcpp::Subscription<GearCommand>::SharedPtr gear_cmd_sub_;
            rclcpp::Subscription<GateMode>::SharedPtr gate_mode_cmd_sub_;
            rclcpp::Subscription<VehicleEmergencyStamped>::SharedPtr emergency_cmd_sub_;

    };  // class RaptorDBWInterface

}  // namespace autoware_raptor_dbw_interface

##endif // AUTOWARE_RAPTOR_DBW_INTERFACE__AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HPP_