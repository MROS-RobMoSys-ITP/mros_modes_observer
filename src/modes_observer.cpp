/*
Copyright 2020, MROS-Project (Metacontrol for ROS2 Systems)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <mros_modes_observer/modes_observer.hpp>

namespace system_modes_observer
{

ModesObserver::ModesObserver(const std::string & name) :
 Node(name)
{
  declare_parameter("componentsfile", rclcpp::ParameterValue(std::string("")));
  std::string components_path = get_parameter("componentsfile").as_string();
  if (components_path.empty()) {
    throw std::invalid_argument("Need path to components file.");
  }

  read_components_from_yaml(components_path);

  // Subscribe to lifecycle messages of each component
  for (auto component : component_names_)
  {
    std::string lifecycle_topic = "/" + component + "/transition_event";
    
    // Callback for lifecycle transitions
    std::function<void(lifecycle_msgs::msg::TransitionEvent::SharedPtr)> state_callback =
      std::bind(
        &ModesObserver::transition_callback,
        this,
        std::placeholders::_1, component);

    auto transition_sub = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      lifecycle_topic,
      rclcpp::QoS(10),
      state_callback);
    transition_request_sub_.push_back(transition_sub);
  } 

  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      rclcpp::QoS(10));
  RCLCPP_INFO(this->get_logger(), "ModesObserver class initialization completed!!");
  
}

ModesObserver::~ModesObserver()
{
}


void ModesObserver::read_components_from_yaml(const std::string & yaml_file_path)
{
  component_names_.clear();
  rcl_params_t * yaml_params = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  if (!rcl_parse_yaml_file(yaml_file_path.c_str(), yaml_params))
  {
    throw std::runtime_error("Failed to parse components from " + yaml_file_path);
  }

  rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);

  rclcpp::ParameterMap::iterator it;
  for (it = param_map.begin(); it != param_map.end(); it++) 
  {
    std::string part_name(it->first.substr(1));
    RCLCPP_INFO(this->get_logger(), "Part name %s ", part_name.c_str());
    for (auto & param : it->second)
    {
      std::string param_name(param.get_name());
      RCLCPP_INFO(this->get_logger(), "param_name %s ", param_name.c_str());
      if (param.get_name().find("components") != std::string::npos)
      {
        std::string component_name(param.value_to_string());
        RCLCPP_INFO(this->get_logger(), "Component name(s): %s ", component_name.c_str());
        std::istringstream iss(component_name); 
        for(std::string component_name; iss >> component_name; )
        {
          RCLCPP_INFO(this->get_logger(), "Adding component: %s ", component_name.c_str());
          component_names_.push_back(component_name);
        }
      }
    }
  }
}

void ModesObserver::transition_callback(
  const lifecycle_msgs::msg::TransitionEvent::SharedPtr transition_msg, 
  const std::string & component_name)
{
  if ((transition_msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING) &&
      (transition_msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED))
  {
    RCLCPP_WARN(this->get_logger(), 
      "Error processing:: Component %s Goal state: %s ", 
      component_name.c_str(), 
      transition_msg->goal_state.label.c_str());
    RCLCPP_WARN(this->get_logger(), "Sending diagnostic message");
    publish_component_diagnostic(component_name, "False");
    
  }
}

void ModesObserver::publish_component_diagnostic(
  const std::string component_name,
  const std::string component_status)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  std::string key = component_name;
  std::string message = "Component status";
  std::string value = component_status;
  publish_diagnostic(level, key, message, value);
}

void ModesObserver::publish_diagnostic(const uint8_t level, 
const std::string key, 
const std::string message, 
const std::string value)
{
  diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status;
  std::vector<diagnostic_msgs::msg::KeyValue> key_values;
  diagnostic_msgs::msg::DiagnosticStatus status_msg;
  diagnostic_msg.header.stamp = now();

  status_msg.level = level;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  key_values.push_back(key_value);
  status_msg.values = key_values;
  status_msg.message = message;
  diag_status.push_back(status_msg);
  diagnostic_msg.status = diag_status;
  diagnostics_pub_->publish(diagnostic_msg);
}

}// namespace system_modes_observer