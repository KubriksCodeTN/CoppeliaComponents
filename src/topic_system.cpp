/**
 * @file topic_system.cpp
 * @brief Implementation of the TopicSystem hardware interface for CoppeliaSim
 */

// Hacky way of tricking the intellisense into including the needed files before
// having Cmake commands
#if __has_include("topic_system.hpp")
#include "topic_system.hpp"
#else
#include "../include/topic_system.hpp"
#endif

#include <algorithm>
#include <execution>
#include <angles/angles.h>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Since ROS is well made we can't use std::par and std::par_unseq because it's
// not possible to link TBB properly but for some reason std::unseq works fine
// (probably because TBB is not needed as we don't use threads)

namespace coppelia_components {

/**
 * @brief Adjusts the rotations in case of limits between -2PI and 2PI
 *
 * @param current_wrapped_rad Arriving angle
 * @param total_rotation Starting angle
 */
void adjust_rotation(double current_wrapped_rad, double &total_rotation) {
    double delta = 0;
    angles::shortest_angular_distance_with_large_limits(
        total_rotation, current_wrapped_rad, 2 * M_PI, -2 * M_PI, delta);

    // Add the correct delta to the total rotation
    total_rotation += delta;
}

/**
 * @brief Initializes the hardware interface data structures with the initial
 * values from the URDF, it supports both initial_value and
 * initial_${interface_name}
 *
 * @note The use of the parameter initial_${interface_name} is deprecated so it
 * is advised to use initial_value
 */
void TopicSystem::init_hw_data(
    matrix<double> &cmds, matrix<double> &states, const auto &interfaces,
    const std::vector<hardware_interface::ComponentInfo> &c_infos) {
    cmds = matrix<double>(
        interfaces.size(),
        std::vector<double>(c_infos.size(),
                            std::numeric_limits<double>::quiet_NaN()));
    states = matrix<double>(
        interfaces.size(),
        std::vector<double>(c_infos.size(),
                            std::numeric_limits<double>::quiet_NaN()));

    for (uint64_t i = 0; i < c_infos.size(); ++i) {
        const auto &c = c_infos[i];
        for (const auto &interface : c.state_interfaces) {
            auto it = std::find(std::execution::unseq, interfaces.begin(),
                                interfaces.end(), interface.name);
            if (it == interfaces.end())
                continue;

            auto j = it - interfaces.begin();
            // auto it2 = c.parameters.end();
            if (!interface.initial_value.empty())
                states[j][i] =
                    hardware_interface::stod(interface.initial_value);
            else if (auto it2 = c.parameters.find("initial_" + interface.name);
                     it2 != c.parameters.end())
                states[j][i] = hardware_interface::stod(it2->second);
        }
    }
}

/**
 * @brief Gets the interfaces used by the various components and check if they
 * are currently supported by the hardware interface
 */
inline bool TopicSystem::get_interface(const std::string &name,
                                       const auto &interface_list,
                                       const std::string &interface_name,
                                       const uint64_t i, matrix<double> &values,
                                       auto &interfaces) {
    auto it = std::find(std::execution::unseq, interface_list.begin(),
                        interface_list.end(), interface_name);
    if (it == interface_list.end())
        return false;
    auto j = it - interface_list.begin();
    interfaces.emplace_back(name, *it, &values[j][i]);
    return true;
}

/**
 * @brief Function to initialize the data structures needed by the hardware
 * interfaces and creates publisher and subscribers to integrate CoppeliaSim
 * inside the loop.
 *
 * @param info Information about ros_control specified in the URDF
 *
 * @note This implementation supports joints (including mimic types) and
 * sensors. It is needed to specify the name of the command topic and joint
 * state topic using respectively the parameters "joint_cmd_topic" and
 * "joint_state_topic" in the URDF
 */
TopicSystem::cb_return
TopicSystem::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        cb_return::SUCCESS)
        return cb_return::ERROR;

    init_hw_data(joint_commands_, joint_states_, standard_interfaces_,
                 info_.joints);

    // Set all values without initial values to 0
    for (uint64_t j = 0; j < standard_interfaces_.size(); ++j)
        for (uint64_t i = 0; i < info_.joints.size(); ++i)
            if (std::isnan(joint_states_[j][i]))
                joint_states_[j][i] = 0.0;

    for (uint64_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        if (joint.parameters.find("mimic") == joint.parameters.cend())
            continue;

        const auto mimicked_it = std::find_if(
            std::execution::unseq, info_.joints.begin(), info_.joints.end(),
            [&m_j = joint.parameters.at("mimic")](const auto &j_info) {
                return m_j == j_info.name;
            });

        if (mimicked_it == info_.joints.cend())
            throw std::runtime_error("Cannot parse ill-formed mimic joint");

        auto mult_it = joint.parameters.find("multiplier");
        auto m =
            mult_it != joint.parameters.end() ?
                hardware_interface::stod(joint.parameters.at("multiplier")) :
                1.;
        mimic_joints_.emplace_back(
            i, std::distance(info_.joints.begin(), mimicked_it), m);
    }

    for (const auto &sensor : info_.sensors) {
        for (const auto &interface : sensor.state_interfaces) {
            auto it = std::find(std::execution::unseq,
                                sensor_interfaces_.begin(),
                                sensor_interfaces_.end(), interface.name);
            if (it == sensor_interfaces_.end())
                sensor_interfaces_.emplace_back(interface.name);
        }
    }
    init_hw_data(sensor_commands_, sensor_states_, sensor_interfaces_,
                 info_.sensors);

    rclcpp::NodeOptions options;
    options.arguments(
        { "--ros-args", "-r", "__node:=topic_control_" + info_.name });
    node_ = rclcpp::Node::make_shared("_", options);

    const auto get_topic_name = [this](const std::string &parameter) {
        auto it = info_.hardware_parameters.find(parameter);
        if (it != info_.hardware_parameters.end())
            return it->second;
        throw std::runtime_error("Please specify the " + parameter +
                                 " parameter in the ros2 control config");
    };

    joint_commands_publisher_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            get_topic_name("joint_cmd_topic"), rclcpp::QoS(1));
    joint_states_subscriber_ =
        node_->create_subscription<sensor_msgs::msg::JointState>(
            get_topic_name("joint_state_topic"), rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                latest_joint_state_ = *msg;
            });

    RCLCPP_INFO(node_->get_logger(), "-------------------------------");
    RCLCPP_INFO(node_->get_logger(), "Publishing order of the joints:");
    for (const auto &j : info_.joints)
        RCLCPP_INFO(node_->get_logger(), "\t%s", j.name.c_str());
    RCLCPP_INFO(node_->get_logger(), "-------------------------------");

    return cb_return::SUCCESS;
}

/**
 * @brief Exports the pointers to the state interfaces of the robot
 *
 * @return The state interface pointers
 */
std::vector<hardware_interface::StateInterface>
TopicSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint64_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        for (const auto &interface : joint.state_interfaces) {
            if (!get_interface(joint.name, standard_interfaces_, interface.name,
                               i, joint_states_, state_interfaces))
                throw std::runtime_error(
                    "Interface is not supported in JointState msg");
        }
    }

    for (uint64_t i = 0; i < info_.sensors.size(); ++i) {
        const auto &component = info_.sensors[i];
        const auto &interfaces = component.state_interfaces;
        for (const auto &interface : interfaces)
            get_interface(component.name, sensor_interfaces_, interface.name, i,
                          sensor_states_, state_interfaces);
    }

    return state_interfaces;
}

/**
 * @brief Exports the pointers to the command interfaces of the robot
 *
 * @return The command interface pointers
 */
std::vector<hardware_interface::CommandInterface>
TopicSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint64_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        for (const auto &interface : joint.command_interfaces) {
            if (!get_interface(joint.name, standard_interfaces_, interface.name,
                               i, joint_commands_, command_interfaces))
                throw std::runtime_error(
                    "Interface is not supported in JointState msg");
        }
    }

    for (uint64_t i = 0; i < info_.sensors.size(); ++i) {
        const auto &component = info_.sensors[i];
        const auto &interfaces = component.state_interfaces;
        for (const auto &interface : interfaces)
            get_interface(component.name, sensor_interfaces_, interface.name, i,
                          sensor_commands_, command_interfaces);
    }

    return command_interfaces;
}

/**
 * @brief Read the Joint states from Coppelia's joint states topic and updates
 * the joint state interfaces. It also sets the sensor values sent by the
 * forward controller
 */
TopicSystem::return_type
TopicSystem::read([[maybe_unused]] const rclcpp::Time &time,
                  [[maybe_unused]] const rclcpp::Duration &period) {
    if (rclcpp::ok())
        rclcpp::spin_some(node_);

    auto cmd_to_state = [](auto &states, const auto &cmds,
                           uint64_t start_i = 0) {
        for (uint64_t i = start_i; i < states.size(); ++i)
            for (uint64_t j = 0; j < states[i].size(); ++j)
                if (!std::isnan(cmds[i][j]))
                    states[i][j] = cmds[i][j];
    };

    // Handle ordering
    for (uint64_t i = 0; i < latest_joint_state_.name.size(); ++i) {
        const auto &joints = info_.joints;
        auto it = std::find_if(
            std::execution::unseq, joints.begin(), joints.end(),
            [&joint_name = std::as_const(latest_joint_state_.name[i])](
                const hardware_interface::ComponentInfo &info) {
                return joint_name == info.name;
            });
        if (it != joints.end()) {
            auto j = it - joints.begin();
            adjust_rotation(latest_joint_state_.position[i],
                            joint_states_[POSITION_INTERFACE][j]);
            // joint_states_[POSITION_INTERFACE][j] =
            // latest_joint_state_.position[i];
            if (!latest_joint_state_.velocity.empty())
                joint_states_[VELOCITY_INTERFACE][j] =
                    latest_joint_state_.velocity[i];
            if (!latest_joint_state_.effort.empty())
                joint_states_[EFFORT_INTERFACE][j] =
                    latest_joint_state_.effort[i];
        }
    }

    for (auto &joint_state : joint_states_)
        for (const auto &mimic_joint : mimic_joints_)
            joint_state[mimic_joint.i] =
                mimic_joint.c * joint_state[mimic_joint.mimicked_i];

    // For sensor values, simply copy them in the interface
    cmd_to_state(sensor_states_, sensor_commands_);

    return return_type::OK;
}

/**
 * @brief Writes on Coppelia's set states topic the desired joint positions.
 * If the new state moves less than a specific epsilon, no message is sent
 */
TopicSystem::return_type
TopicSystem::write([[maybe_unused]] const rclcpp::Time &time,
                   [[maybe_unused]] const rclcpp::Duration &period) {
    const auto d = std::transform_reduce(
        std::execution::unseq, joint_states_[POSITION_INTERFACE].cbegin(),
        joint_states_[POSITION_INTERFACE].cend(),
        joint_commands_[POSITION_INTERFACE].cbegin(), .0,
        [](const double a, const double b) {
            return std::fabs(a) + std::fabs(b);
        },
        std::minus<double>());

    if (d <= e)
        return return_type::OK;

    std_msgs::msg::Float64MultiArray joint_positions;
    for (uint64_t i = 0; i < info_.joints.size(); ++i)
        joint_positions.data = joint_commands_[POSITION_INTERFACE];

    for (const auto &mimic_joint : mimic_joints_)
        joint_positions.data[mimic_joint.i] =
            joint_positions.data[mimic_joint.mimicked_i] * mimic_joint.c;

    if (rclcpp::ok())
        joint_commands_publisher_->publish(joint_positions);

    return return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(coppelia_components::TopicSystem,
                       hardware_interface::SystemInterface)
