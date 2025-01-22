#ifndef ROS_NYOKKEY4_CONTROLLER_HPP_
#define ROS_NYOKKEY4_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "std_msgs/msg/bool.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ros2_nyokkey4_controller
{
    typedef struct
    {
        double a1;
        double a2;
        double a3;
    } TorqueCoefficients;

    static const std::map<std::string, TorqueCoefficients> torque_coefficients = {
        {"joint_1", {2.1492E-01, 3.1069E-04, 8.8751E-06}},  // WHJ60-100
        {"joint_2", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"joint_3", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"joint_4", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"joint_5", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"joint_6", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"joint_7", {5.1807E-02, -7.7299E-05, 2.4623E-06}}  // WHJ10-80
    };

    class Ros2Nyokkey4Controller : public joint_trajectory_controller::JointTrajectoryController
    {
    public:
        Ros2Nyokkey4Controller() = default;
         ~Ros2Nyokkey4Controller() override = default;
         using joint_trajectory_controller::JointTrajectoryController::JointTrajectoryController;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        // controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        void read_state_from_state_interfaces(JointTrajectoryPoint & state); // hiding


    private:
        void update_pids();
        // struct JointConfig
        // {
        //     std::string name;
        //     double amplitude;
        //     double inv_amplitude;
        //     double frequency;
        //     double phase;
        //     bool is_target;
        //     double current_position;
        //     double current_velocity;
        //     double current_effort;
        //     int16_t encoder_diff;
        // };
        // std::vector<JointConfig> joint_configs_;
        // std::vector<std::string> joint_names_;
        // std::vector<std::string> state_interface_types_;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_encoder_diff_state_interface_;
        // std::unordered_map<
        //     std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        //     state_interface_map_ = {
        //         {"position", &joint_position_state_interface_},
        //         {"velocity", &joint_velocity_state_interface_},
        //         {"effort", &joint_effort_state_interface_},
        //         {"encoder_diff", &joint_encoder_diff_state_interface_}};
        // double homing_velocity_;    // ゼロ位置への移動速度
        // double position_tolerance_; // ゼロ位置の許容誤差
        rclcpp::Time last_log_time_;
    };

} // namespace ros2_nyokkey4_controller

#endif // ROS_NYOKKEY4_CONTROLLER_HPP_