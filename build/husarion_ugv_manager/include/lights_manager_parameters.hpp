#pragma message("#include \"lights_manager_parameters.hpp\" is deprecated. Use #include <husarion_ugv_manager/lights_manager_parameters.hpp> instead.")
// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace lights_manager {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::string bt_project_path = "";
        int64_t bt_server_port = 5555;
        std::vector<std::string> plugin_libs = {};
        std::vector<std::string> ros_plugin_libs = {};
        double timer_frequency = 10.0;
        struct Battery {
            double charging_anim_step = 0.1;
            struct AnimPeriod {
                double critical = 15.0;
                double low = 30.0;
            } anim_period;
            struct Percent {
                int64_t window_len = 6;
                struct Threshold {
                    double critical = 0.1;
                    double low = 0.4;
                } threshold;
            } percent;
        } battery;
        struct RosCommunicationTimeout {
            double availability = 1.0;
            double response = 1.0;
        } ros_communication_timeout;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        int64_t bt_server_port = 5555;
        double timer_frequency = 10.0;
        struct Battery {
            double charging_anim_step = 0.1;
            struct AnimPeriod {
                double critical = 15.0;
                double low = 30.0;
            } anim_period;
            struct Percent {
                int64_t window_len = 6;
                struct Threshold {
                    double critical = 0.1;
                    double low = 0.4;
                } threshold;
            } percent;
        } battery;
        struct RosCommunicationTimeout {
            double availability = 1.0;
            double response = 1.0;
        } ros_communication_timeout;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("lights_manager"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "")
    : prefix_{prefix},
      logger_{std::move(logger)} {
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    /**
     * @brief Tries to update the parsed Params object
     * @param params_in The Params object to update
     * @return true if the Params object was updated, false if it was already up to date or the mutex could not be locked
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_update_params(Params & params_in) const {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
          return true;
        }
      }
      return false;
    }

    /**
     * @brief Tries to get the current Params object
     * @param params_in The Params object to fill with the current parameters
     * @return true if mutex can be locked, false if mutex could not be locked
     * @note The parameters are only filled, when the mutex can be locked and the params timestamp is different
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.battery.charging_anim_step = params.battery.charging_anim_step;
      output.battery.anim_period.critical = params.battery.anim_period.critical;
      output.battery.anim_period.low = params.battery.anim_period.low;
      output.battery.percent.window_len = params.battery.percent.window_len;
      output.battery.percent.threshold.critical = params.battery.percent.threshold.critical;
      output.battery.percent.threshold.low = params.battery.percent.threshold.low;
      output.bt_server_port = params.bt_server_port;
      output.ros_communication_timeout.availability = params.ros_communication_timeout.availability;
      output.ros_communication_timeout.response = params.ros_communication_timeout.response;
      output.timer_frequency = params.timer_frequency;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "battery.charging_anim_step")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.battery.charging_anim_step = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "battery.anim_period.critical")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.battery.anim_period.critical = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "battery.anim_period.low")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.battery.anim_period.low = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "battery.percent.window_len")) {
            if(auto validation_result = gt<int64_t>(param, 0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.battery.percent.window_len = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "battery.percent.threshold.critical")) {
            if(auto validation_result = bounds<double>(param, 0.0, 1.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.battery.percent.threshold.critical = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "battery.percent.threshold.low")) {
            if(auto validation_result = bounds<double>(param, 0.0, 1.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.battery.percent.threshold.low = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "bt_project_path")) {
            updated_params.bt_project_path = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "bt_server_port")) {
            if(auto validation_result = gt<int64_t>(param, 0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.bt_server_port = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "plugin_libs")) {
            updated_params.plugin_libs = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "ros_communication_timeout.availability")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.ros_communication_timeout.availability = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "ros_communication_timeout.response")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.ros_communication_timeout.response = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "ros_plugin_libs")) {
            updated_params.ros_plugin_libs = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "timer_frequency")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.timer_frequency = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      if (user_callback_) {
         user_callback_(updated_params);
      }
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "battery.charging_anim_step")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "This parameter defines the minimum change in battery percentage required to trigger an update in the battery charging animation.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.battery.charging_anim_step);
          parameters_interface_->declare_parameter(prefix_ + "battery.charging_anim_step", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "battery.anim_period.critical")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Time in seconds to wait before repeating animation, indicating a critical battery state.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.battery.anim_period.critical);
          parameters_interface_->declare_parameter(prefix_ + "battery.anim_period.critical", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "battery.anim_period.low")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Time in seconds to wait before repeating the animation, indicating a low battery state.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.battery.anim_period.low);
          parameters_interface_->declare_parameter(prefix_ + "battery.anim_period.low", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "battery.percent.window_len")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Moving average window length used to smooth out battery percentage readings.";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 0;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int64_t>::max();
          auto parameter = to_parameter_value(updated_params.battery.percent.window_len);
          parameters_interface_->declare_parameter(prefix_ + "battery.percent.window_len", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "battery.percent.threshold.critical")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If the battery percentage drops below this value, an animation indicating a critical battery state will start being displayed.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = 1.0;
          auto parameter = to_parameter_value(updated_params.battery.percent.threshold.critical);
          parameters_interface_->declare_parameter(prefix_ + "battery.percent.threshold.critical", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "battery.percent.threshold.low")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If the battery percentage drops below this value, the animation indicating a low battery state will start being displayed.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = 1.0;
          auto parameter = to_parameter_value(updated_params.battery.percent.threshold.low);
          parameters_interface_->declare_parameter(prefix_ + "battery.percent.threshold.low", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "bt_project_path")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Path to a BehaviorTree project.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.bt_project_path);
          parameters_interface_->declare_parameter(prefix_ + "bt_project_path", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "bt_server_port")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Port number for the BehaviorTree server.";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 0;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int64_t>::max();
          auto parameter = to_parameter_value(updated_params.bt_server_port);
          parameters_interface_->declare_parameter(prefix_ + "bt_server_port", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "plugin_libs")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "A list with names of plugins that are used in the BehaviorTree project.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.plugin_libs);
          parameters_interface_->declare_parameter(prefix_ + "plugin_libs", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "ros_communication_timeout.availability")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout in seconds to wait for a service/action while initializing a BehaviorTree node.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.ros_communication_timeout.availability);
          parameters_interface_->declare_parameter(prefix_ + "ros_communication_timeout.availability", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "ros_communication_timeout.response")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout in seconds to receive a service/action response after call.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.ros_communication_timeout.response);
          parameters_interface_->declare_parameter(prefix_ + "ros_communication_timeout.response", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "ros_plugin_libs")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "A list with names of ROS plugins that are used in a BehaviorTree project.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.ros_plugin_libs);
          parameters_interface_->declare_parameter(prefix_ + "ros_plugin_libs", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "timer_frequency")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Frequency in Hz at which lights tree will be ticked.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.timer_frequency);
          parameters_interface_->declare_parameter(prefix_ + "timer_frequency", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "battery.charging_anim_step");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'battery.charging_anim_step': {}", validation_result.error()));
      }
      updated_params.battery.charging_anim_step = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "battery.anim_period.critical");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'battery.anim_period.critical': {}", validation_result.error()));
      }
      updated_params.battery.anim_period.critical = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "battery.anim_period.low");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'battery.anim_period.low': {}", validation_result.error()));
      }
      updated_params.battery.anim_period.low = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "battery.percent.window_len");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<int64_t>(param, 0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'battery.percent.window_len': {}", validation_result.error()));
      }
      updated_params.battery.percent.window_len = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "battery.percent.threshold.critical");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = bounds<double>(param, 0.0, 1.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'battery.percent.threshold.critical': {}", validation_result.error()));
      }
      updated_params.battery.percent.threshold.critical = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "battery.percent.threshold.low");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = bounds<double>(param, 0.0, 1.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'battery.percent.threshold.low': {}", validation_result.error()));
      }
      updated_params.battery.percent.threshold.low = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "bt_project_path");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.bt_project_path = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "bt_server_port");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<int64_t>(param, 0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'bt_server_port': {}", validation_result.error()));
      }
      updated_params.bt_server_port = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "plugin_libs");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.plugin_libs = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "ros_communication_timeout.availability");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'ros_communication_timeout.availability': {}", validation_result.error()));
      }
      updated_params.ros_communication_timeout.availability = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "ros_communication_timeout.response");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'ros_communication_timeout.response': {}", validation_result.error()));
      }
      updated_params.ros_communication_timeout.response = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "ros_plugin_libs");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.ros_plugin_libs = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "timer_frequency");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'timer_frequency': {}", validation_result.error()));
      }
      updated_params.timer_frequency = param.as_double();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    using userParameterUpdateCB = std::function<void(const Params&)>;
    void setUserCallback(const userParameterUpdateCB& callback){
      user_callback_ = callback;
    }

    void clearUserCallback(){
      user_callback_ = {};
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
      userParameterUpdateCB user_callback_;

      rclcpp::Logger logger_;
      std::mutex mutable mutex_;
  };

} // namespace lights_manager
