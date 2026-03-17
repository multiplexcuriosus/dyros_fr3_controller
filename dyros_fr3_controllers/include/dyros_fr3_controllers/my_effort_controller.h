#pragma once

#include <string>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <mutex>

#include "math_type_define.h"
#include <Eigen/Eigen>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int32.hpp>

#include "dyros_fr3_interfaces/srv/set_track_pose.hpp"

namespace franka_semantic_components
{
class FrankaRobotModel;
class FrankaRobotState;
}

namespace ConsoleColor
{
  inline constexpr const char* RESET = "\033[0m";
  inline constexpr const char* BLUE  = "\033[34m";
  inline constexpr const char* YELLOW= "\033[33m";
  inline constexpr const char* RED   = "\033[31m";
}

#define LOGI(node, fmt, ...) RCLCPP_INFO((node)->get_logger(),  (std::string(ConsoleColor::BLUE)   + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)
#define LOGW(node, fmt, ...) RCLCPP_WARN((node)->get_logger(),  (std::string(ConsoleColor::YELLOW) + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)
#define LOGE(node, fmt, ...) RCLCPP_ERROR((node)->get_logger(), (std::string(ConsoleColor::RED)    + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace dyros_fr3_controllers
{

class My_effort_controller : public controller_interface::ControllerInterface
{
public:
  ~My_effort_controller() override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  enum class CtrlMode { NONE = 0, HOME = 1, TRACK = 2 };

  Eigen::Vector7d q_init_;
  Eigen::Vector7d qdot_init_;
  Eigen::Vector7d q_;
  Eigen::Vector7d qdot_;
  Eigen::Vector7d torque_;

  Eigen::Vector7d q_desired_;
  Eigen::Vector7d qdot_desired_;
  Eigen::Vector7d torque_desired_;

  Eigen::Matrix7d M_;
  Eigen::Matrix7d M_inv_;
  Eigen::Vector7d c_;
  Eigen::Vector7d g_;

  Eigen::Affine3d x_init_;
  Eigen::Vector6d xdot_init_;
  Eigen::Affine3d x_;
  Eigen::Vector6d xdot_;
  Eigen::Matrix<double, 6, 7> J_;

  Eigen::Affine3d x_desired_;
  Eigen::Vector6d xdot_desired_;
  Eigen::Matrix6d M_task_;
  Eigen::Vector6d g_task_;
  Eigen::Matrix<double, 6, 7> J_T_inv_;

  const double dt_{0.001};
  Eigen::Vector7d kp_joint_;
  Eigen::Vector7d kv_joint_;
  Eigen::Vector7d home_q_;
  Eigen::Vector7d track_q_target_;
  Eigen::Vector7d last_ik_solution_;

  double play_time_{0.0};
  double control_start_time_{0.0};
  double home_duration_sec_{4.0};
  double track_duration_sec_{2.0};

  CtrlMode control_mode_{CtrlMode::HOME};
  bool is_mode_changed_{false};
  bool initialization_flag_{true};
  bool track_target_valid_{false};

  Eigen::Affine3d track_pose_target_{Eigen::Affine3d::Identity()};

  std::string arm_id_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  static constexpr int num_joints = 7;
  bool use_pinocchio_{true};
  bool use_franka_model_{false};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_mode_sub_;
  rclcpp::Service<dyros_fr3_interfaces::srv::SetTrackPose>::SharedPtr track_pose_srv_;

  std::mutex robot_data_mutex_;
  std::mutex calculation_mutex_;
  std::atomic<bool> compute_inflight_{false};
  std::atomic<bool> relax_wait_guard_{false};
  std::thread compute_thread_;
  std::mutex compute_cv_mutex_;
  std::condition_variable compute_cv_;
  std::condition_variable compute_done_cv_;
  bool compute_requested_{false};
  bool compute_completed_{false};
  bool stop_compute_thread_{false};

  void compute();
  void updateJointStates();
  void updateRobotData();
  void setMode(CtrlMode control_mode);
  void computeWorkerLoop();
  void initializeModeTargets();

  void controlModeCallback(const std_msgs::msg::Int32& msg);
  void trackPoseCallback(
      const std::shared_ptr<dyros_fr3_interfaces::srv::SetTrackPose::Request> request,
      std::shared_ptr<dyros_fr3_interfaces::srv::SetTrackPose::Response> response);

  void packPinocchioState(const Eigen::Vector7d& q_arm, const Eigen::Vector7d& qdot_arm);

  Eigen::Vector7d JointPDControl(const Eigen::Vector7d& target_q, const Eigen::Vector7d& target_qdot);
  Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::Pose& pose) const;
  bool solveIk(const Eigen::Affine3d& target_pose, const Eigen::Vector7d& seed_q, Eigen::Vector7d& q_solution);

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::Data data_worker_;
  std::string ee_name_{"fr3_hand_tcp"};
  bool pinocchio_ready_{false};

  Eigen::VectorXd q_model_;
  Eigen::VectorXd qdot_model_;
};

}  // namespace dyros_fr3_controllers