/**
 * \mainpage Topic System
 * \section intro_sec Introduction
 *
 * This is a simple hardware interface that allows integration of CoppeliaSim
 * into the ros2_control loop using a topic-based system.
 *
 * \section brief_desc Usage
 *
 * This interface supports joints (including mimic ones) and sensors.
 *
 * To use this interface, declare inside the URDF a hardware of type
 * coppelia_components/TopicSystem.
 * The required parameters are:
 *      - <b>joint_cmd_topic</b>: Used to specify the name of the topic to send
 *          joint commands to CoppeliaSim. The type of the topic has to be
 *          std_msgs/msg/Float64MultiArray.
 *      - <b>joint_state_topic</b>: Used to specify the name of the topic to
 *          read joint states from CoppeliaSim. The type of the topic has to be
 *          sensor_msgs/msg/JointState.
 *
 * The supported interfaces for joints are the ones supported by the JointState
 * message:
 *      - <b>position</b>
 *      - <b>velocity</b>
 *      - <b>effort</b>
 *
 * For sensors, you can declare any type of state interface. An associated
 * command interface will be created so that the values can be updated using a
 * standard ros2 forward controller or a custom made one, if needed.
 * This has the same effect as setting to true mock_sensor_commands in the
 * ros2 mock_components/GenericSystem.
 *
 * \section brief_ex Example usage
 * This is a minimal example of a system that uses this interface, declared
 * inside a URDF.
 *
 \verbatim
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
 \endverbatim
 */

/**
 * @file topic_system.hpp
 * @brief Header of the TopicSystem hardware interface
 */
// NOTE: Non-standard pragma
#pragma once

#include <string>
#include <vector>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/executors.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/**
 * @brief Namespace used by the hardware interface TopicSystem
 */
namespace coppelia_components {

/**
 * @brief Hardware interface to integrate CoppeliaSim with ros2 control
 *
 * @note This interface supports sensors and joints (including mimic ones).
 * The supported interfaces for the joints are the ones supported in the
 * JointState messages (position, velocity, and effort).
 * It is mandatory to specify the name of the topics used by CoppeliaSim.
 * To do so, use the parameters:
 *      - joint_cmd_topic (Float64MultiArray)
 *      - joint_state_topic (JointState)
 */
class HARDWARE_INTERFACE_PUBLIC TopicSystem
    : public hardware_interface::SystemInterface {
private:
    template <typename T> using matrix = std::vector<std::vector<T> >;

    void init_hw_data(matrix<double> &, matrix<double> &, const auto &,
                      const std::vector<hardware_interface::ComponentInfo> &);
    bool get_interface(const std::string &, const auto &, const std::string &,
                       const size_t, matrix<double> &, auto &);

protected:
    /**
     * @brief Struct containing the basic information to handle a mimic type
     * joint
     */
    struct mimic_joint {
        /** @brief Index of the mimic joint */
        int64_t i;
        /** @brief Index of the mimicked joint */
        int64_t mimicked_i;
        /** @brief Joint value multiplier (default = 1.0) */
        double c;
    };

    /** @brief Epsilon for double comparisons tolerance */
    static inline constexpr double e = 1e-5;

    ///@{
    /** @name Interfaces indexes */
    /** @brief Indexes of the various interfaces in the matrix */
    static inline constexpr size_t POSITION_INTERFACE = 0;
    static inline constexpr size_t VELOCITY_INTERFACE = 1;
    static inline constexpr size_t EFFORT_INTERFACE = 2;
    ///@}

    /**
     * @brief Array of interfaces supported by the hardware interface for the
     * joints
     */
    static inline constexpr std::array<const char *, 3> standard_interfaces_ = {
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT,
    };

    /** @brief Structure to handle mimic joints */
    std::vector<mimic_joint> mimic_joints_;
    /** @brief Structure for joint commands */
    matrix<double> joint_commands_;
    /** @brief Structure for joint states */
    matrix<double> joint_states_;

    /** @brief Contains the interfaces defined for sensors */
    std::vector<std::string> sensor_interfaces_;
    /** @brief Structure for sensor commands */
    matrix<double> sensor_commands_;
    /** @brief Structure for sensor states */
    matrix<double> sensor_states_;

    /** @brief Subscriber used to read joint states from the joint_state_topic
     */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_states_subscriber_;
    /** @brief Publisher used to write joint commands to the joint_cmd_topic
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        joint_commands_publisher_;
    /** @brief Node used to publish/subscribe */
    rclcpp::Node::SharedPtr node_;
    /** @brief Auxiliary variable to cache the latest read joint state */
    sensor_msgs::msg::JointState latest_joint_state_;

public:
    using return_type = hardware_interface::return_type;
    using cb_return =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    cb_return on_init(const hardware_interface::HardwareInfo &) override;
    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;
    return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
};

}
