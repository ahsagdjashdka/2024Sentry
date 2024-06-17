/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rm_serial_driver.hpp/cpp
  * @brief      串口通信模块
  * @note       感谢@ChenJun创建本模块并开源，
  *             现内容为北极熊基于开源模块进行修改并适配自己的车车后的结果。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2022            ChenJun         1. done
  *  V1.0.1     2023-12-11      Penguin         1. 添加与rm_rune_dector_node模块连接的Client
  *  V1.0.2     2024-3-1        LihanChen       1. 添加导航数据包，并重命名packet和相关函数
  *  V1.0.3     2024-3-4        LihanChen       1. 添加裁判系统数据包
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  all_robot_hp_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::AllRobotHP>("/all_robot_hp", 3);
  robot_status_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("/robot_status", 10);
  game_status_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::GameStatus>("/game_status", 1);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  rune_detector_param_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(this, "rm_rune_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "gimbal_odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendDataVision, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_chassis", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&RMSerialDriver::sendDataTwist, this, std::placeholders::_1));
  // robot_control_sub_ = this->create_subscription<rm_decision_interfaces::msg::RobotControl>(
  //   "/robot_control", rclcpp::QoS(rclcpp::KeepLast(1)),
  //   std::bind(&RMSerialDriver::sendRobotControl, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      switch (header[0]) {
        case 0x5A:
          receiveDataVision(header);
          break;
        case 0x5B:
          receiveDataAllRobotHP(header);
          break;
        case 0x5C:
          receiveDataGameStatus(header);
          break;
        case 0x5D:
          receiveDataRobotStatus(header);
          break;
        default:
          RCLCPP_WARN(get_logger(), "Unknown header received: %02X", header[0]);
          break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
    }
  }
}

void RMSerialDriver::receiveDataVision(std::vector<uint8_t> header)
{
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacketVision));

  try {
    data.resize(sizeof(ReceivePacketVision) - 1);
    serial_driver_->port()->receive(data);

    data.insert(data.begin(), header[0]);
    ReceivePacketVision packet = fromVector<ReceivePacketVision>(data);

    bool crc_ok =
      crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (crc_ok) {
      if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
        setParam(rclcpp::Parameter("detect_color", packet.detect_color));
        previous_receive_color_ = packet.detect_color;
      }

      if (packet.reset_tracker) {
        resetTracker();
      }

      geometry_msgs::msg::TransformStamped t;
      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
      t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      t.header.frame_id = "gimbal_odom";
      t.child_frame_id = "gimbal_link";
      tf2::Quaternion q;
      q.setRPY(packet.roll, packet.pitch, packet.yaw);
      t.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(t);

      if (abs(packet.aim_x) > 0.01) {
        aiming_point_.header.stamp = this->now();
        aiming_point_.pose.position.x = packet.aim_x;
        aiming_point_.pose.position.y = packet.aim_y;
        aiming_point_.pose.position.z = packet.aim_z;
        marker_pub_->publish(aiming_point_);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "CRC error!");
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::receiveDataAllRobotHP(std::vector<uint8_t> header)
{
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacketAllRobotHP));

  try {
    data.resize(sizeof(ReceivePacketAllRobotHP) - 1);
    serial_driver_->port()->receive(data);

    data.insert(data.begin(), header[0]);
    ReceivePacketAllRobotHP packet = fromVector<ReceivePacketAllRobotHP>(data);

    bool crc_ok =
      crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (crc_ok) {
      // all_robot_hp_.red_1_robot_hp = packet.red_1_robot_hp;
      // all_robot_hp_.red_2_robot_hp = packet.red_2_robot_hp;
      // all_robot_hp_.red_3_robot_hp = packet.red_3_robot_hp;
      // all_robot_hp_.red_4_robot_hp = packet.red_4_robot_hp;
      // all_robot_hp_.red_5_robot_hp = packet.red_5_robot_hp;
      // all_robot_hp_.red_7_robot_hp = packet.red_7_robot_hp;
      // all_robot_hp_.red_base_hp = packet.red_base_hp;
      all_robot_hp_.red_outpost_hp = packet.red_outpost_hp;
      // all_robot_hp_.blue_1_robot_hp = packet.blue_1_robot_hp;
      // all_robot_hp_.blue_2_robot_hp = packet.blue_2_robot_hp;
      // all_robot_hp_.blue_3_robot_hp = packet.blue_3_robot_hp;
      // all_robot_hp_.blue_4_robot_hp = packet.blue_4_robot_hp;
      // all_robot_hp_.blue_5_robot_hp = packet.blue_5_robot_hp;
      // all_robot_hp_.blue_7_robot_hp = packet.blue_7_robot_hp;
      // all_robot_hp_.blue_base_hp = packet.blue_base_hp;
      all_robot_hp_.blue_outpost_hp = packet.blue_outpost_hp;

      all_robot_hp_pub_->publish(all_robot_hp_);
    } else {
      RCLCPP_ERROR(get_logger(), "CRC error!");
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::receiveDataGameStatus(std::vector<uint8_t> header)
{
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacketGameStatus));

  try {
    data.resize(sizeof(ReceivePacketGameStatus) - 1);
    serial_driver_->port()->receive(data);

    data.insert(data.begin(), header[0]);
    ReceivePacketGameStatus packet = fromVector<ReceivePacketGameStatus>(data);

    bool crc_ok =
      crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (crc_ok) {
      game_status_.game_progress = packet.game_progress;
      game_status_.stage_remain_time = packet.stage_remain_time;

      game_status_pub_->publish(game_status_);
    } else {
      RCLCPP_ERROR(get_logger(), "CRC error!");
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::receiveDataRobotStatus(std::vector<uint8_t> header)
{
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacketRobotStatus));

  try {
    data.resize(sizeof(ReceivePacketRobotStatus) - 1);
    serial_driver_->port()->receive(data);

    data.insert(data.begin(), header[0]);
    ReceivePacketRobotStatus packet = fromVector<ReceivePacketRobotStatus>(data);

    bool crc_ok =
      crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (crc_ok) {
      robot_status_.robot_id = packet.robot_id;
      robot_status_.current_hp = packet.current_hp;
      // robot_status_.shooter_heat = packet.shooter_heat;
      // robot_status_.team_color = packet.team_color;
      // robot_status_.is_attacked = packet.is_attacked;

      robot_status_pub_->publish(robot_status_);
    } else {
      RCLCPP_ERROR(get_logger(), "CRC error!");
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendDataVision(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> ID_UNIT8_MAP{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacketVision packet;
    packet.tracking = msg->tracking;
    packet.id = ID_UNIT8_MAP.at(msg->id);
    packet.armors_num = msg->armors_num;
    packet.x = msg->position.x;
    packet.y = msg->position.y;
    packet.z = msg->position.z;
    packet.yaw = msg->yaw;
    packet.vx = msg->velocity.x;
    packet.vy = msg->velocity.y;
    packet.vz = msg->velocity.z;
    packet.v_yaw = msg->v_yaw;
    packet.r1 = msg->radius_1;
    packet.r2 = msg->radius_2;
    packet.dz = msg->dz;

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendDataTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    SendPacketTwist packet;
    packet.linear_x = msg->linear.x;
    packet.linear_y = msg->linear.y;
    // packet.linear_z = msg->linear.z;
    // packet.angular_x = msg->angular.x;
    // packet.angular_y = msg->angular.y;
    packet.angular_z = msg->angular.z;

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

// void RMSerialDriver::sendRobotControl(
//   const rm_decision_interfaces::msg::RobotControl::SharedPtr msg)
// {
//   try {
//     SendPacketRobotControl packet;
//     packet.stop_gimbal_scan = msg->stop_gimbal_scan;
//     packet.chassis_spin_vel = msg->chassis_spin_vel;

//     crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
//     std::vector<uint8_t> data = toVector(packet);
//     serial_driver_->port()->send(data);

//   } catch (const std::exception & ex) {
//     RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
//     reopenPort();
//   }
// }

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Armor service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting armor detect_color to %ld...", param.as_int());

    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set armor detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }

  if (!rune_detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Rune service not ready, skipping parameter set");
    return;
  }

  if (
    !set_rune_detector_param_future_.valid() ||
    set_rune_detector_param_future_.wait_for(std::chrono::seconds(0)) ==
      std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting rune detect_color to %ld...", param.as_int());
    set_rune_detector_param_future_ = rune_detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set rune detect_color to %ld!", 1 - param.as_int());
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
