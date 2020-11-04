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

#ifndef MODES_OBSERVER_HPP
#define MODES_OBSERVER_HPP

#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <system_modes/srv/get_available_modes.hpp>
#include <system_modes/srv/get_mode.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rcl_yaml_param_parser/parser.h>



namespace system_modes_observer
{

class ModesObserver : public rclcpp::Node
{
    private:
        //void modes_update_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void publish_diagnostic(uint8_t level, 
        std::string key, 
        std::string message, 
        std::string value);
        void publish_component_diagnostic(
            const std::string component_name,
            const std::string component_status);
        void transition_callback(
            const lifecycle_msgs::msg::TransitionEvent::SharedPtr transition_msg, 
            const std::string & component_name);
        void read_components_from_yaml(const std::string & yaml_file_path);
        std::vector<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> transition_request_sub_;
        rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr laser_transition_sub_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
        std::vector<std::string> component_names_;
    public:
        ModesObserver(const std::string & name);
        ~ModesObserver();
};
}// namespace system_modes_observer
#endif /* MODES_OBSERVER_HPP */
