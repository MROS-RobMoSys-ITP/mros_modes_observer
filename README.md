# mros_modes_observer Package   

This package is used to check for possible failures in [managed nodes](http://design.ros2.org/articles/node_lifecycle.html) that correspond to components of a MROS system.

## Usage

To monitor the status of the components (i.e. laser or other sensors) the `mros_modes_observer` subscribes to the `[component_node]/transition_event` topic.

A failure is defined by a transition from `lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING` to `lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED`. In this case, a message marking this is sent to the [mros2_reasoner](https://github.com/tud-cor/mc_mros_reasoner) package (i.e. the Metacontroller) using the `/diagnostics` topic.

## Configuration file

The list of components to be monitored is defined in the [components.yaml](https://github.com/MROS-RobMoSys-ITP/mros_modes_observer/blob/main/params/components.yaml) file. A subscriber is created for each component in that file, therefore the `component` field should correspond to the node name.


## Installation and usage

To use this package, follow the instructions on here https://github.com/MROS-RobMoSys-ITP/MROS-Hands-On
