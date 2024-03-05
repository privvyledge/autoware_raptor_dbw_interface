#ifndef AUTOWARE_RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_
#define AUTOWARE_RAPTOR_DBW_INTERFACE__RAPTOR_DBW_INTERFACE_HPP_

// Raptor DBW command messages
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/gear_cmd.hpp>
#include <raptor_dbw_msgs/msg/global_enable_cmd.hpp>
#include <raptor_dbw_msgs/msg/misc_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/actuator_control_mode.hpp>

// Raptor DBW Core Feedback messages
#include <raptor_dbw_msgs/msg/brake_report.hpp>
#include <raptor_dbw_msgs/msg/gear_report.hpp>
#include <raptor_dbw_msgs/msg/misc_report.hpp>
#include <raptor_dbw_msgs/msg/other_actuators_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/gear.hpp>
#include <raptor_dbw_msgs/msg/parking_brake.hpp>

// Raptor DBW Miscellaneous Feedback messages.
#include <raptor_dbw_msgs/msg/door_request.hpp>
#include <raptor_dbw_msgs/msg/door_state.hpp>
#include <raptor_dbw_msgs/msg/high_beam.hpp>
#include <raptor_dbw_msgs/msg/high_beam_state.hpp>
#include <raptor_dbw_msgs/msg/horn_state.hpp>
#include <raptor_dbw_msgs/msg/ignition.hpp>
#include <raptor_dbw_msgs/msg/low_beam.hpp>
#include <raptor_dbw_msgs/msg/turn_signal.hpp>
#include <raptor_dbw_msgs/msg/wiper_front.hpp>
#include <raptor_dbw_msgs/msg/wiper_rear.hpp>

// Autoware Core Command messages
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
// todo: remove the lines below and replace raw_control
#include <autoware_auto_vehicle_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_control_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>

// Autoware Core Feedback messages
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <vehicle_info_util/vehicle_info.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_vehicle_msgs/srv/autonomy_mode_change.hpp>
//#include <vehicle_interface/dbw_state_machine.hpp>
//#include <vehicle_interface/platform_interface.hpp>

// Autoware Miscellaneous Command messages
#include <autoware_auto_vehicle_msgs/msg/headlights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/wipers_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/srv/autonomy_mode_change.hpp>
// todo remove the two include statements below
//#include <autoware_auto_common/common/types.hpp>
//#include <motion_common/motion_common.hpp>

// Type Definitions
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <cstdint>
#include <unordered_map>


// Namespace definitions.  Todo: remove autoware_auto_common definitions and replace float32_t with float, float64_t with double, bool8_t with bool. Get Tau and Pi from C++
using autoware_auto_common::common::types::bool8_t;
using autoware_auto_common::common::types::float32_t;
using autoware_auto_common::common::types::float64_t;
using autoware_auto_common::common::types::TAU;
using autoware::common::types::PI;

// Raptor DBW Command namespaces
using raptor_dbw_msgs::msg::AcceleratorPedalCmd;
using raptor_dbw_msgs::msg::BrakeCmd;
using raptor_dbw_msgs::msg::GearCmd;
using raptor_dbw_msgs::msg::GlobalEnableCmd;
using raptor_dbw_msgs::msg::MiscCmd;
using raptor_dbw_msgs::msg::SteeringCmd;
using raptor_dbw_msgs::msg::ActuatorControlMode;
using raptor_dbw_msgs::msg::ParkingBrake;
using autoware_auto_control_msgs::msg::HighLevelControlCommand;
using autoware_auto_vehicle_msgs::msg::RawControlCommand;
using autoware_auto_vehicle_msgs::msg::VehicleControlCommand;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::VehicleStateCommand;

// Raptor DBW Feedback namespaces
using raptor_dbw_msgs::msg::BrakeReport;
using raptor_dbw_msgs::msg::MiscReport;
using raptor_dbw_msgs::msg::OtherActuatorsReport;
using raptor_dbw_msgs::msg::SteeringReport;
using raptor_dbw_msgs::msg::WheelSpeedReport;

// Raptor DBW Miscellaneous Feedback namespaces. Todo: implement
using raptor_dbw_msgs::msg::DoorRequest;
using raptor_dbw_msgs::msg::DoorState;
using raptor_dbw_msgs::msg::HighBeam;
using raptor_dbw_msgs::msg::HighBeamState;
using raptor_dbw_msgs::msg::HornState;
using raptor_dbw_msgs::msg::Ignition;
using raptor_dbw_msgs::msg::LowBeam;
using raptor_dbw_msgs::msg::TurnSignal;
using raptor_dbw_msgs::msg::WiperFront;
using raptor_dbw_msgs::msg::WiperRear;

// Autoware Miscellaeous Command namespaces. Todo: implement
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using autoware_auto_vehicle_msgs::msg::WipersCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;

// Autoware Feedback namespaces
using vehicle_info_util::VehicleInfo;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::ControlModeReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using tier4_control_msgs::msg::GateMode;
using autoware_auto_vehicle_msgs::msg::VehicleStateReport;
using autoware_auto_vehicle_msgs::msg::VehicleOdometry;
using autoware_auto_vehicle_msgs::msg::VehicleKinematicState;

// Autoware Command namespaces
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::srv::AutonomyModeChange;
using ModeChangeRequest = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Request;
using ModeChangeResponse = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Response;

// Type definition namespaces
using std_msgs::msg::Float64;
using tier4_control_msgs::msg::GateMode;
using tier4_vehicle_msgs::msg::BatteryStatus;
using tier4_vehicle_msgs::msg::VehicleEmergencyStamped;
using namespace std::chrono_literals;  // NOLINT

// Map states, i.e autoware <--> raptor states. Todo: replace with DBW state machine
std::unordered_map<uint8_t, uint8_t> control_mode_mapping = {
  {GateMode::AUTO, ControlModeReport::AUTONOMOUS},
  {GateMode::EXTERNAL, ControlModeReport::MANUAL}
};


namespace autoware_raptor_dbw_interface
{
    class AUTOWARE_RAPTOR_DBW_INTERFACE_PUBLIC RaptorDBWInterface : public rclcpp::Node
    {
        public:
            explicit RaptorDBWInterface (
//                const rclcpp::NodeOptions & options,
                rclcpp::Node & node,
                uint16_t ecu_build_num,
                float32_t front_axle_to_cog,
                float32_t rear_axle_to_cog,
                float32_t steer_to_tire_ratio,
                float32_t max_steer_angle,
                float32_t acceleration_limit,
                float32_t deceleration_limit,
                float32_t acceleration_positive_jerk_limit,
                float32_t deceleration_negative_jerk_limit,
                uint32_t pub_period
            );

            ~RaptorDBWInterface() noexcept override = default;

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
            rclcpp::Publisher<SteeringReport>::SharedPtr steering_rpt_pub_;
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

            // Initializae empty messages
            autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
            AcceleratorPedalCmd m_accel_cmd{};
            BrakeCmd m_brake_cmd{};
            GearCmd m_gear_cmd{};
            GlobalEnableCmd m_gl_en_cmd{};
            MiscCmd m_misc_cmd{};
            SteeringCmd m_steer_cmd{};

            // Declare constants
            rclcpp::Logger m_logger;
            uint16_t m_ecu_build_num;
            float32_t m_front_axle_to_cog;
            float32_t m_rear_axle_to_cog;
            float32_t m_steer_to_tire_ratio;
            float32_t m_max_steer_angle;
            float32_t m_acceleration_limit;
            float32_t m_deceleration_limit;
            float32_t m_acceleration_positive_jerk_limit;
            float32_t m_deceleration_negative_jerk_limit;
            std::chrono::milliseconds m_pub_period;
//            std::unique_ptr<DbwStateMachine> m_dbw_state_machine;  // todo: implement
            uint8_t m_rolling_counter;
            rclcpp::Clock m_clock;
            rclcpp::TimerBase::SharedPtr m_timer;

            // vehicle parameter constants
            float wheel_diameter_param_{0.1};
            float motor_ratio_{0.01};
            float max_steer_angle_param_{0.5};
            const float max_vel_m_s_;

            // setup constants for Autoware. todo: replace with vehicle state
            VehicleInfo vehicle_info_;
            uint8_t control_mode_;
            uint8_t gear_;  // current gear
            Gear current_gear_{Gear::PARK};  // current gear
            double battery_charge_;
            bool emergency_;  // emergency stop
            bool emergency_stop_{false};  // emergency stop
            double steering_angle_rpt_;  // current steer angle
            double current_steer_angle_{0.0};  // current steer angle
            double current_heading_rate_{0.0};
            double velocity_rpt_;

            // Declare and Initialize states
            bool8_t m_seen_brake_rpt{false};
            bool8_t m_seen_gear_rpt{false};
            bool8_t m_seen_misc_rpt{false};
            bool8_t m_seen_steering_rpt{false};
            bool8_t m_seen_wheel_spd_rpt{false};
            bool8_t m_seen_vehicle_state_cmd{false};
            float32_t m_travel_direction{0.0F};

            // Callbacks
            void on_brake_report(const BrakeReport::SharedPtr & msg);
            void on_gear_report(const GearReport::SharedPtr & msg);
            void on_misc_report(const MiscReport::SharedPtr & msg);
            void on_other_actuators_report(const OtherActuatorsReport::SharedPtr & msg);
            void on_steering_report(const SteeringReport::SharedPtr & msg);
            void on_wheel_spd_report(const WheelSpeedReport::SharedPtr & msg);

            void callback_control_cmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
            void to_vehicle();  // same as cmdCallback
            void from_vehicle();

    };  // class RaptorDBWInterface

}  // namespace autoware_raptor_dbw_interface