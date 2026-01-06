# CoppeliaComponents

## Introduction

This is a simple hw interface that permits to integrate CoppeliaSim into the ros2_control loop using a topic based system


## Usage

This interface supports joints (includic mimic ones) and sensors.

To use this interface declare inside the urdf an hardware of type coppelia_components/TopicSystem
The required parameters are:

* **joint_cmd_topic**: used to specify the name of the topic to send joint commands to CoppeliaSim, the type of the topic has to be std_msgs/msg/Float64MultiArray

* **joint_state_topic**: used to specify the name of the topic to read joint states from CoppeliaSim, the type of the topic has to be sensor_msgs/msg/JointState

The supported interfaces for joints are the ones supported by the JointState msg:

* **position**

* **velocity**

* **effort**

For sensor you can declare any type of state interface, an associated cmd interface will be created 
so that the values can be update using a standard ros2 forward controller or a custom made one, if needed.
This has the the same effect as setting to true mock_sensor_commands in the ros2 mock_components/GenericSystem


## Example usage

This is a minimal example of a System that uses this interface, declared inside a urdf

::

   <ros2_control name="my_robot" type="system">
        <hardware>
            <plugin>coppelia_components/TopicSystem</plugin>
            <param name="joint_cmd_topic">coppelia_set</param>
            <param name="joint_state_topic">coppelia_joint</param>
        </hardware>
        <joint name="my_robot_joint">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
            <state_interface name="velocity"/>
        </joint>
   </ros2_control>

::
