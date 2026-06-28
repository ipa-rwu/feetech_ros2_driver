#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_ros2_driver/feetech_ros2_driver.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace feetech_ros2_driver {
namespace {

double to_position_from_ticks(const int raw_ticks, const JointMappingConfig& config) {
  if (config.mapping == JointPositionMapping::kGripperJaw) {
    const double span = static_cast<double>(config.range_max - config.range_min);
    double normalized = (static_cast<double>(raw_ticks) - static_cast<double>(config.range_min)) / span;
    normalized = std::clamp(normalized, 0.0, 1.0);
    if (config.drive_mode) {
      normalized = 1.0 - normalized;
    }
    return config.position_lower + normalized * (config.position_upper - config.position_lower);
  }

  return feetech_driver::to_radians(raw_ticks - feetech_driver::kStsMidpoint);
}

int to_ticks_from_position(const double position, const JointMappingConfig& config) {
  if (config.mapping == JointPositionMapping::kGripperJaw) {
    const double span = config.position_upper - config.position_lower;
    double normalized = (position - config.position_lower) / span;
    normalized = std::clamp(normalized, 0.0, 1.0);
    if (config.drive_mode) {
      normalized = 1.0 - normalized;
    }
    return static_cast<int>(std::lround(static_cast<double>(config.range_min) +
                                        normalized * static_cast<double>(config.range_max - config.range_min)));
  }

  return feetech_driver::from_radians(position) + feetech_driver::kStsMidpoint;
}

}  // namespace

feetech_driver::Expected<JointCommandMode> FeetechHardwareInterface::get_joint_command_mode_(
    const hardware_interface::ComponentInfo& joint) {
  bool has_position_command = false;
  bool has_velocity_command = false;

  for (const auto& command_interface : joint.command_interfaces) {
    if (command_interface.name == hardware_interface::HW_IF_POSITION) {
      has_position_command = true;
      continue;
    }
    if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
      has_velocity_command = true;
      continue;
    }
    return tl::make_unexpected(
        fmt::format("Joint '{}' has unsupported command interface '{}'", joint.name, command_interface.name));
  }

  if (has_position_command && has_velocity_command) {
    return tl::make_unexpected(
        fmt::format("Joint '{}' cannot expose both position and velocity command interfaces", joint.name));
  }
  if (has_position_command) {
    return JointCommandMode::kPosition;
  }
  if (has_velocity_command) {
    return JointCommandMode::kVelocity;
  }
  return JointCommandMode::kNone;
}

#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
#else
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#endif
    return CallbackReturn::ERROR;
  }

  if (init_transport_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  JointIdConfigMap yaml_by_id;
  if (load_yaml_config_and_warn_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (configure_joints_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (validate_model_series_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::init_transport_() {
  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error(
        "FeetechHardwareInterface::init_transport_ Hardware parameter [usb_port] not found! "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }

  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);

  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("FeetechHardwareInterface::init_transport_ -> {}", result.error());
    return CallbackReturn::ERROR;
  }

  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  return CallbackReturn::SUCCESS;
}

// Optional YAML overlay — if not provided, URDF params are used as-is.
// Builds an ID-keyed map: URDF id is the hardware identity, YAML name is just a label.
CallbackReturn FeetechHardwareInterface::load_yaml_config_and_warn_(JointIdConfigMap& out_yaml) {
  out_yaml.clear();

  const auto cfg_it = info_.hardware_parameters.find("joint_config_file");
  if (cfg_it == info_.hardware_parameters.end() || cfg_it->second.empty()) {
    return CallbackReturn::SUCCESS;  // no YAML — fall back to URDF params only
  }

  auto loaded = load_joint_config(cfg_it->second);
  if (!loaded) {
    return CallbackReturn::ERROR;
  }

  // Re-key by servo id
  for (auto& [name, params] : *loaded) {
    auto it = params.find("id");
    if (it == params.end()) {
      spdlog::error("YAML joint '{}' has no 'id' parameter", name);
      return CallbackReturn::ERROR;
    }
    int id = std::stoi(it->second);
    if (!out_yaml.emplace(id, std::move(params)).second) {
      spdlog::error("Duplicate servo id {} in YAML (joint '{}')", id, name);
      return CallbackReturn::ERROR;
    }
  }

  // Warn: URDF ids missing in YAML
  for (const auto& j : info_.joints) {
    auto id_it = j.parameters.find("id");
    if (id_it != j.parameters.end() && out_yaml.find(std::stoi(id_it->second)) == out_yaml.end()) {
      spdlog::warn("URDF joint '{}' (id={}) has no YAML entry (using URDF defaults)", j.name, id_it->second);
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::configure_joints_(const JointIdConfigMap& yaml_by_id) {
  joint_ids_.assign(info_.joints.size(), 0);
  joint_command_modes_.assign(info_.joints.size(), JointCommandMode::kNone);
  joint_mapping_configs_.assign(info_.joints.size(), JointMappingConfig{});

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    const std::string& joint_name = joint.name;

    // Required: id (from URDF — hardware identity)
    const auto urdf_id_it = joint.parameters.find("id");
    if (urdf_id_it == joint.parameters.end()) {
      spdlog::error("Joint '{}' does not have required 'id' parameter", joint_name);
      return CallbackReturn::ERROR;
    }
    const int id = std::stoi(urdf_id_it->second);
    joint_ids_[i] = static_cast<uint8_t>(id);

    const auto command_mode = get_joint_command_mode_(joint);
    if (!command_mode) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", command_mode.error());
      return CallbackReturn::ERROR;
    }
    joint_command_modes_[i] = *command_mode;

    spdlog::info("Configuring joint '{}' (id={}, mode={})",
                 joint_name,
                 id,
                 joint_command_modes_[i] == JointCommandMode::kVelocity ? "velocity"
                 : joint_command_modes_[i] == JointCommandMode::kPosition ? "position"
                                                                          : "none");

    // Merge YAML config (looked up by servo id) over URDF params
    JointParams merged_params;
    if (auto it = yaml_by_id.find(id); it != yaml_by_id.end()) {
      merged_params = merge_joint_params(it->second, joint.parameters);
    } else {
      merged_params = JointParams(joint.parameters.begin(), joint.parameters.end());
    }

    if (merged_params.find("offset") != merged_params.end()) {
      spdlog::warn("Joint '{}': 'offset' param is deprecated and ignored — use 'homing_offset' instead", joint_name);
    }

    const auto mapping_config = build_joint_mapping_config_(merged_params);
    if (!mapping_config) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ mapping [{} id={}] -> {}",
                    joint_name,
                    id,
                    mapping_config.error());
      return CallbackReturn::ERROR;
    }
    joint_mapping_configs_[i] = *mapping_config;

    // Disable torque and unlock EPROM before writing parameters
    if (const auto result = communication_protocol_->disable_torque(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ disable_torque [{} id={}] -> {}",
                    joint_name,
                    id,
                    result.error());
      return CallbackReturn::ERROR;
    }

    if (joint_command_modes_[i] == JointCommandMode::kPosition) {
      if (const auto result = communication_protocol_->set_mode(joint_ids_[i], feetech_driver::OperationMode::kPosition);
          !result) {
        spdlog::error("FeetechHardwareInterface::configure_joints_ set_mode(position) [{} id={}] -> {}",
                      joint_name,
                      id,
                      result.error());
        return CallbackReturn::ERROR;
      }
    } else if (joint_command_modes_[i] == JointCommandMode::kVelocity) {
      if (const auto result = communication_protocol_->set_mode(joint_ids_[i], feetech_driver::OperationMode::kSpeed);
          !result) {
        spdlog::error("FeetechHardwareInterface::configure_joints_ set_mode(speed) [{} id={}] -> {}",
                      joint_name,
                      id,
                      result.error());
        return CallbackReturn::ERROR;
      }
    }

    // Single-byte parameters (0-255)
    for (const auto& [parameter_name, address] : {std::pair{"p_coefficient", SMS_STS_P_COEF},
                                                  {"d_coefficient", SMS_STS_D_COEF},
                                                  {"i_coefficient", SMS_STS_I_COEF},
                                                  {"overload_torque", SMS_STS_OVERLOAD_TORQUE},
                                                  {"return_delay_time", SMS_STS_RETURN_DELAY},
                                                  {"acceleration", SMS_STS_ACC}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        const auto result = communication_protocol_->write(
            joint_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ write-u8 [{} id={} param={}] -> {}",
                        joint_name,
                        id,
                        parameter_name,
                        result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte unsigned parameters
    for (const auto& [parameter_name, address] : {std::pair{"range_min", SMS_STS_MIN_ANGLE_LIMIT_L},
                                                  {"range_max", SMS_STS_MAX_ANGLE_LIMIT_L},
                                                  {"max_torque_limit", SMS_STS_MAX_TORQUE_L},
                                                  {"protection_current", SMS_STS_PROTECTION_CURRENT_L}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        std::array<uint8_t, 2> buf{};
        feetech_driver::to_sts(&buf[0], &buf[1], std::stoi(param_it->second));
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ write-u16 [{} id={} param={}] -> {}",
                        joint_name,
                        id,
                        parameter_name,
                        result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte signed parameters (sign-magnitude encoding)
    for (const auto& [parameter_name, address, sign_bit] :
         {std::tuple{"homing_offset", SMS_STS_OFS_L, SMS_STS_SIGN_BIT_HOMING_OFFSET}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        std::array<uint8_t, 2> buf{};
        const int value = feetech_driver::encode_sign_magnitude(std::stoi(param_it->second), sign_bit);
        feetech_driver::to_sts(&buf[0], &buf[1], value);
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ write-s16 [{} id={} param={}] -> {}",
                        joint_name,
                        id,
                        parameter_name,
                        result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Lock EPROM after writing parameters (for all joints)
    if (const auto result = communication_protocol_->lock_eprom(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ lock_eprom [{} id={}] -> {}",
                    joint_name,
                    id,
                    result.error());
      return CallbackReturn::ERROR;
    }

    // Only enable torque for joints with command interfaces (Follower Arm)
    if (joint_command_modes_[i] != JointCommandMode::kNone) {
      if (const auto result = communication_protocol_->set_torque(joint_ids_[i], true); !result) {
        spdlog::error("FeetechHardwareInterface::configure_joints_ set_torque [{} id={}] -> {}",
                      joint_name,
                      id,
                      result.error());
        return CallbackReturn::ERROR;
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

feetech_driver::Expected<JointMappingConfig> FeetechHardwareInterface::build_joint_mapping_config_(
    const JointParams& merged_params) const {
  JointMappingConfig config;

  if (const auto it = merged_params.find("position_mapping"); it != merged_params.end()) {
    if (it->second == "servo_angle") {
      config.mapping = JointPositionMapping::kServoAngle;
    } else if (it->second == "gripper_jaw") {
      config.mapping = JointPositionMapping::kGripperJaw;
    } else {
      return tl::make_unexpected(fmt::format("Unsupported position_mapping '{}'", it->second));
    }
  }

  if (const auto it = merged_params.find("drive_mode"); it != merged_params.end()) {
    config.drive_mode = std::stoi(it->second) != 0;
  }

  if (config.mapping == JointPositionMapping::kGripperJaw) {
    const auto range_min_it = merged_params.find("range_min");
    const auto range_max_it = merged_params.find("range_max");
    const auto lower_it = merged_params.find("position_lower");
    const auto upper_it = merged_params.find("position_upper");
    if (range_min_it == merged_params.end() || range_max_it == merged_params.end()) {
      return tl::make_unexpected("gripper_jaw mapping requires range_min and range_max");
    }
    if (lower_it == merged_params.end() || upper_it == merged_params.end()) {
      return tl::make_unexpected("gripper_jaw mapping requires position_lower and position_upper");
    }

    config.range_min = std::stoi(range_min_it->second);
    config.range_max = std::stoi(range_max_it->second);
    config.position_lower = std::stod(lower_it->second);
    config.position_upper = std::stod(upper_it->second);

    if (config.range_max <= config.range_min) {
      return tl::make_unexpected(
          fmt::format("invalid tick range: [{}..{}]", config.range_min, config.range_max));
    }
    if (config.position_upper <= config.position_lower) {
      return tl::make_unexpected(
          fmt::format("invalid position range: [{}, {}]", config.position_lower, config.position_upper));
    }
  }

  return config;
}

CallbackReturn FeetechHardwareInterface::validate_model_series_() {
  const auto joint_model_series = joint_ids_ | ranges::views::transform([&](const auto id) {
                                    return communication_protocol_->read_model_number(id)
                                        .and_then(feetech_driver::get_model_name)
                                        .and_then(feetech_driver::get_model_series);
                                  });

  if (std::ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
    spdlog::error("FeetechHardware::validate_model_series_ [One of the joints has an error]. Input: {}",
                  ranges::views::zip(joint_ids_, joint_model_series));
    return CallbackReturn::ERROR;
  }

  const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

  if (ranges::any_of(js, [](const auto& series) { return series != feetech_driver::ModelSeries::kSts; })) {
    spdlog::error("FeetechHardware::validate_model_series_ [Only STS series is supported]. Input (id, series): {}",
                  ranges::views::zip(joint_ids_, js));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FeetechHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_hw_positions_.resize(info_.joints.size(), 0.0);
  state_hw_velocities_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (joint_command_modes_[i] == JointCommandMode::kPosition) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      continue;
    }
    if (joint_command_modes_[i] == JointCommandMode::kVelocity) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
  }

  return command_interfaces;
}

hardware_interface::return_type FeetechHardwareInterface::read(const rclcpp::Time& /* time */,
                                                               const rclcpp::Duration& /* period */) {
  // 4 = 2 bytes for position + 2 bytes for speed
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(joint_ids_.size());
  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
    spdlog::error("FeetechHardwareInterface::read -> {}", result.error());
    return hardware_interface::return_type::ERROR;
  }
  ranges::for_each(data | ranges::views::enumerate, [&](const auto& values) {
    const auto& [index, readings] = values;
    const int raw_position =
        feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[0], .high = readings[1]});
    state_hw_positions_[index] = to_position_from_ticks(raw_position, joint_mapping_configs_[index]);
    state_hw_velocities_[index] = feetech_driver::to_radians(feetech_driver::decode_sign_magnitude(
        feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[2], .high = readings[3]}),
        SMS_STS_SIGN_BIT_VELOCITY));
  });
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Create vectors only for joints that have command interfaces
  std::vector<uint8_t> commanded_joint_ids;
  std::vector<int> commanded_positions;
  std::vector<int> commanded_speeds;
  std::vector<int> commanded_accelerations;
  std::vector<uint8_t> commanded_velocity_joint_ids;
  std::vector<std::array<uint8_t, 2>> commanded_velocities;

  for (uint i = 0; i < info_.joints.size(); i++) {
    if (joint_command_modes_[i] == JointCommandMode::kPosition) {
      commanded_joint_ids.push_back(joint_ids_[i]);
      commanded_positions.push_back(to_ticks_from_position(hw_positions_[i], joint_mapping_configs_[i]));
      commanded_speeds.push_back(2400);       // Default speed
      commanded_accelerations.push_back(50);  // Default acceleration
      continue;
    }

    if (joint_command_modes_[i] == JointCommandMode::kVelocity) {
      try {
        std::array<uint8_t, 2> velocity_buffer{};
        const int encoded_velocity = feetech_driver::encode_sign_magnitude(
            feetech_driver::from_radians(hw_velocities_[i]), SMS_STS_SIGN_BIT_VELOCITY);
        feetech_driver::to_sts(&velocity_buffer[0], &velocity_buffer[1], encoded_velocity);
        commanded_velocity_joint_ids.push_back(joint_ids_[i]);
        commanded_velocities.push_back(velocity_buffer);
      } catch (const std::exception& e) {
        spdlog::error("FeetechHardwareInterface::write velocity encode [{}] -> {}", info_.joints[i].name, e.what());
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  // Only send commands if there are joints to command
  if (!commanded_joint_ids.empty()) {
    const auto write_result = communication_protocol_->sync_write_position(
        commanded_joint_ids, commanded_positions, commanded_speeds, commanded_accelerations);
    if (!write_result) {
      spdlog::error("FeetechHardwareInterface::write -> {}", write_result.error());
      return hardware_interface::return_type::ERROR;
    }
  }

  if (!commanded_velocity_joint_ids.empty()) {
    const auto write_result =
        communication_protocol_->sync_write(commanded_velocity_joint_ids, SMS_STS_GOAL_SPEED_L, commanded_velocities);
    if (!write_result) {
      spdlog::error("FeetechHardwareInterface::write velocity -> {}", write_result.error());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn FeetechHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Time/Duration are not used
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (joint_command_modes_[i] == JointCommandMode::kPosition) {
      hw_positions_[i] = state_hw_positions_[i];
      continue;
    }
    if (joint_command_modes_[i] == JointCommandMode::kVelocity) {
      hw_velocities_[i] = 0.0;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // all joints torque off
  const auto torque_disable_parameters =
      std::vector(joint_ids_.size(), std::experimental::make_array(static_cast<uint8_t>(0)));
  if (const auto result =
          communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_disable_parameters);
      !result) {
    spdlog::error("FeetechHardwareInterface::on_deactivate -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace feetech_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::FeetechHardwareInterface, hardware_interface::SystemInterface)
