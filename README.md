# ros2_vda5050_integration

Integration packages to enable communication between ROS 2 and VDA5050 protocol, in particular for those AMRs with a Nav2 stack.

ROS 2 distro tested: `Humble` `Jazzy`

## VDA5050 Connector for ROS 2

The [vda5050_connector](https://github.com/inorbit-ai/ros_amr_interop/tree/galactic-devel/vda5050_connector#readme)
package provides a set of ROS2 nodes for connecting a ROS2-based robot to a [VDA5050 Master Control](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md#-5-process-and-content-of-communication).

For a proper schema of how the connector works, look at [here](https://github.com/inorbit-ai/ros_amr_interop/blob/galactic-devel/vda5050_connector/README.md).

[Reference](https://github.com/inorbit-ai/ros_amr_interop/tree/galactic-devel/vda5050_connector)

## VDA5050 msgs

ROS 2 messages that allow exchange of info between vda5050_controller, mqtt_bridge, and vda5050_adapter.

## VDA5050 serializer

The serialization/deserialization functions to be loaded into the mqtt_bridge.

## VDA5050 adapter

A ROS 2 package with a Python implementation of the adapter. 
