/*
 * This code defines unit tests for loading different kinds of control systems
 * using the ros2_control framework. The tests cover loading a valid 2-DOF
 * control system, loading a system with an unknown interface, and loading a
 * system missing required parameters.
 */

#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

TEST(TestTopicSystem, load_topic_system_2dof) {
    // Define a valid 2-DOF control system as a string in URDF format
    const std::string hardware_system_2dof =
    R"(
        <ros2_control name="TopicSystem2dof" type="system">
        <hardware>
            <plugin>coppelia_components/TopicSystem</plugin>
            <param name="joint_cmd_topic">/coppelia_set_joints</param>
            <param name="joint_state_topic">/coppelia_joint_states</param>
        </hardware>
        <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <sensor name="tcp_fts_sensor">
            <state_interface name="force.x"/>
            <state_interface name="force.y"/>
            <state_interface name="force.z"/>
            <state_interface name="torque.x"/>
            <state_interface name="torque.y"/>
            <state_interface name="torque.z"/>
            <param name="frame_id">joint1</param>
        </sensor>
        </ros2_control>
    )";

    // Create a URDF string by combining the defined control system with the test assets
    auto urdf = ros2_control_test_assets::urdf_head +
        hardware_system_2dof +
        ros2_control_test_assets::urdf_tail;

    // Assert that loading the control system does not throw an exception
    ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, true, false));
}

TEST(TestTopicSystem, load_wrong_system) {
    // Define a control system with an unknown interface as a string in URDF
    // format
    const std::string wrong_hardware_system =
    R"(
        <ros2_control name="TopicSystem2dof" type="system">
        <hardware>
            <plugin>coppelia_components/TopicSystem</plugin>
            <param name="joint_cmd_topic">/coppelia_set_joints</param>
            <param name="joint_state_topic">/coppelia_joint_states</param>
        </hardware>
        <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="unknown"/>
        </joint>
        </ros2_control>
    )";

    // Create a URDF string by combining the defined control system with the test assets
    auto urdf = ros2_control_test_assets::urdf_head +
        wrong_hardware_system +
        ros2_control_test_assets::urdf_tail;

    // Assert that loading the control system throws an exception
    ASSERT_THROW(hardware_interface::ResourceManager rm(urdf, true, false),
                 std::exception);
}

TEST(TestTopicSystem, load_system_no_topic_param) {
    // Define a control system missing required topic parameters as a string in
    // URDF format
    const std::string hardware_system_no_topic_param =
    R"(
        <ros2_control name="TopicSystem2dof" type="system">
        <hardware>
            <plugin>coppelia_components/TopicSystem</plugin>
            <param name="joint_cmd_topic">/coppelia_set_joints</param>
        </hardware>
        <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <sensor name="tcp_fts_sensor">
            <state_interface name="force.x"/>
            <state_interface name="force.y"/>
            <state_interface name="force.z"/>
            <state_interface name="torque.x"/>
            <state_interface name="torque.y"/>
            <state_interface name="torque.z"/>
            <param name="frame_id">joint1</param>
        </sensor>
        </ros2_control>
    )";

    // Create a URDF string by combining the defined control system with the test assets
    auto urdf = ros2_control_test_assets::urdf_head +
        hardware_system_no_topic_param +
        ros2_control_test_assets::urdf_tail;

    // Assert that loading the control system throws an exception
    ASSERT_THROW(hardware_interface::ResourceManager rm(urdf, true, false),
                 std::exception);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    return RUN_ALL_TESTS();
}
