#include "autoware_raptor_dbw_interface/ne_raptor_interface.hpp"

#include <rclcpp/logging.hpp>
#include <time_utils/time_utils.hpp>

#include <cmath>
#include <stdexcept>
#include <iostream>

namespace autoware_raptor_dbw_interface
{
    RaptorDBWInterfaceNode::RaptorDBWInterfaceNode(
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
    )
    : m_logger{node.get_logger()},
      m_ecu_build_num{ecu_build_num},
      m_front_axle_to_cog{front_axle_to_cog},
      m_rear_axle_to_cog{rear_axle_to_cog},
      m_steer_to_tire_ratio{steer_to_tire_ratio},
      m_max_steer_angle{max_steer_angle},
      m_acceleration_limit{acceleration_limit},
      m_deceleration_limit{std::fabs(deceleration_limit)},
      m_acceleration_positive_jerk_limit{acceleration_positive_jerk_limit},
      m_deceleration_negative_jerk_limit{deceleration_negative_jerk_limit},
      m_pub_period{std::chrono::milliseconds(pub_period)},
//      m_dbw_state_machine(new DbwStateMachine{3}),
      m_rolling_counter{0},
      m_clock{RCL_SYSTEM_TIME}

      {
          // Publishers (to Raptor DBW)
          m_accel_cmd_pub = node.create_publisher<AcceleratorPedalCmd>("accelerator_pedal_cmd", 1);
          m_brake_cmd_pub = node.create_publisher<BrakeCmd>("brake_cmd", 1);
          m_gear_cmd_pub = node.create_publisher<GearCmd>("gear_cmd", 1);
          m_gl_en_cmd_pub = node.create_publisher<GlobalEnableCmd>("global_enable_cmd", 1);
          m_misc_cmd_pub = node.create_publisher<MiscCmd>("misc_cmd", 1);
          m_steer_cmd_pub = node.create_publisher<SteeringCmd>("steering_cmd", 1);
          m_dbw_enable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("enable", 10);
          m_dbw_disable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("disable", 10);

          // Publishers (to Autoware)
          m_vehicle_kin_state_pub = node.create_publisher<VehicleKinematicState>(
            "vehicle_kinematic_state",
            10);
          gear_status_pub_ = node.create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
            "/vehicle/status/gear_status", rclcpp::QoS{1});
          // todo: add more publishers to Autoware

          // Subscribers (from Raptor DBW)
          m_brake_rpt_sub =
            node.create_subscription<BrakeReport>(
            "brake_report", rclcpp::QoS{20},
            [this](BrakeReport::SharedPtr msg) {on_brake_report(msg);});
          m_gear_rpt_sub =
            node.create_subscription<GearReport>(
            "gear_report", rclcpp::QoS{20},
            [this](GearReport::SharedPtr msg) {on_gear_report(msg);});
          m_misc_rpt_sub =
            node.create_subscription<MiscReport>(
            "misc_report", rclcpp::QoS{2},
            [this](MiscReport::SharedPtr msg) {on_misc_report(msg);});
          m_other_acts_rpt_sub =
            node.create_subscription<OtherActuatorsReport>(
            "other_actuators_report", rclcpp::QoS{20},
            [this](OtherActuatorsReport::SharedPtr msg) {on_other_actuators_report(msg);});
          m_steering_rpt_sub =
            node.create_subscription<SteeringReport>(
            "steering_report", rclcpp::QoS{20},
            [this](SteeringReport::SharedPtr msg) {on_steering_report(msg);});
          m_wheel_spd_rpt_sub =
            node.create_subscription<WheelSpeedReport>(
            "wheel_speed_report", rclcpp::QoS{20},
            [this](WheelSpeedReport::SharedPtr msg) {on_wheel_spd_report(msg);});

          // Subscribers (from Autoware)
          control_cmd_sub_ = node.create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
            "/control/command/control_cmd", 1, std::bind(&NERaptorInterface::callback_control_cmd, this, _1));

          // Initialize command values
          auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();  // vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
          m_gl_en_cmd.ecu_build_number = m_ecu_build_num;
          m_gl_en_cmd.enable_joystick_limits = false;

          m_accel_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;   // vehicle speed
          m_accel_cmd.ignore = false;
          m_accel_cmd.accel_limit = m_acceleration_limit;
          m_accel_cmd.accel_positive_jerk_limit = m_acceleration_positive_jerk_limit;

          m_brake_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;   // vehicle speed
          m_brake_cmd.decel_limit = m_deceleration_limit;
          m_brake_cmd.decel_negative_jerk_limit = m_deceleration_negative_jerk_limit;

          m_steer_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_ACTUATOR;  // angular position
          m_steer_cmd.ignore = false;

          m_gear_cmd.cmd.gear = GearReport::NONE;

          m_misc_cmd.door_request_right_rear.value = DoorRequest::NO_REQUEST;
          m_misc_cmd.door_request_left_rear.value = DoorRequest::NO_REQUEST;
          m_misc_cmd.door_request_lift_gate.value = DoorRequest::NO_REQUEST;
          m_misc_cmd.rear_wiper_cmd.status = WiperRear::OFF;
          m_misc_cmd.ignition_cmd.status = Ignition::NO_REQUEST;
          m_misc_cmd.cmd.value = TurnSignal::NONE;
          m_misc_cmd.low_beam_cmd.status = LowBeam::OFF;
          m_misc_cmd.high_beam_cmd.status = HighBeam::OFF;
          m_misc_cmd.front_wiper_cmd.status = WiperFront::OFF;

          m_timer = node.create_wall_timer(m_pub_period, std::bind(&NERaptorInterface::to_vehicle, this));
      }

      void NERaptorInterface::callback_control_cmd(
          const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
            {
              control_cmd_ptr_ = msg;
            }

      void NERaptorInterface::to_vehicle()
        {
          // note that this is the same as the to_vehicle() callback in Autoware Universe. todo
          // Increment rolling counter
          m_rolling_counter++;
          if (m_rolling_counter > 15) {
            m_rolling_counter = 0;
          }

          // Publish commands to NE Raptor DBW

          // Set state flags
        }

      bool8_t NERaptorInterface::send_state_command(const VehicleStateCommand & msg)
        {
          // alias to to_autoware(). todo
          // Sends gear status commands, etc.

          // Set gear values

          // Set blinker values

          // Set parking brake command

        }

      bool8_t NERaptorInterface::send_control_command(const HighLevelControlCommand & msg)
        {
          // this parses Autoware commands and sends to the car. Same as callback_control_cmd

          // Using curvature for control

          // Set limits

          // Check for invalid changes in direction

          // Set commands
        }

      bool8_t NERaptorInterface::send_control_command(const VehicleControlCommand & msg)
        {

        }

      bool8_t NERaptorInterface::send_control_command(const AckermannControlCommand & msg)
        {

        }

      bool8_t NERaptorInterface::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
        {

        }

      void NERaptorInterface::send_headlights_command(const HeadlightsCommand & msg)
        {

        }

      void NERaptorInterface::send_horn_command(const HornCommand & msg)
        {
          // Set misc command values
        }

      void NERaptorInterface::send_wipers_command(const WipersCommand & msg)
        {

        }

      void NERaptorInterface::on_brake_report(const BrakeReport::SharedPtr & msg)
        {

        }

      void NERaptorInterface::on_gear_report(const GearReport::SharedPtr & msg)
        {

        }

      void NERaptorInterface::on_misc_report(const MiscReport::SharedPtr & msg)
        {

        }

      void NERaptorInterface::on_other_actuators_report(const OtherActuatorsReport::SharedPtr & msg)
        {

        }

      void NERaptorInterface::on_steering_report(const SteeringReport::SharedPtr & msg)
        {

        }

      void NERaptorInterface::on_wheel_spd_report(const WheelSpeedReport::SharedPtr & msg)
        {

        }

      // Update x, y, heading, and heading_rate
      void NERaptorInterface::kinematic_bicycle_model(
          float32_t dt, VehicleKinematicState * vks)
        {
        }

}  // namespace autoware_raptor_dbw_interface