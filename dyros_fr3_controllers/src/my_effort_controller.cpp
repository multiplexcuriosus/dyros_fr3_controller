#include "dyros_fr3_controllers/my_effort_controller.h"
#include <dyros_fr3_controllers/robot_utils.hpp>

namespace dyros_fr3_controllers 
{
// ========================================================================
// ============================ Core Functions ============================
// ========================================================================
My_effort_controller::~My_effort_controller()
{
  {
    std::lock_guard<std::mutex> lock(compute_cv_mutex_);
    stop_compute_thread_ = true;
    compute_requested_ = false;
  }
  compute_cv_.notify_all();
  compute_done_cv_.notify_all();
  if (compute_thread_.joinable())
  {
    compute_thread_.join();
  }
}

CallbackReturn My_effort_controller::on_init() 
{
  try 
  {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("kp_joint_gains", {});
    auto_declare<std::vector<double>>("kd_joint_gains", {});

    const std::string pkg_share = ament_index_cpp::get_package_share_directory("dyros_fr3_controllers");
    urdf_path = pkg_share + "/urdf/fr3_franka_hand.urdf";
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.good()) 
    {
      LOGW(get_node(), "URDF not found at path: %s (Pinocchio will be disabled)", urdf_path.c_str());
      use_pinocchio_ = false;
    }

    if(use_pinocchio_)
    {
      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = pinocchio::Data(model_);
      data_worker_ = pinocchio::Data(model_);
    }

    q_init_.setZero();
    qdot_init_.setZero();

    q_.setZero();
    qdot_.setZero();
    torque_.setZero();

    q_desired_.setZero();
    qdot_desired_.setZero();
    torque_desired_.setZero();

    M_.setIdentity();
    M_inv_.setIdentity();
    c_.setZero();
    g_.setZero();

    x_init_.setIdentity();
    xdot_init_.setZero();

    x_.setIdentity();
    xdot_.setZero();
    J_.setZero();

    M_task_.setIdentity();
    g_task_.setZero();
    J_T_inv_.setZero();

    control_mode_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      "my_effort_controller/control_mode",
      rclcpp::QoS(10),
      std::bind(&My_effort_controller::controlModeCallback, this, std::placeholders::_1)
    );

    if (!compute_thread_.joinable())
    {
      std::lock_guard<std::mutex> lock(compute_cv_mutex_);
      stop_compute_thread_ = false;
      compute_requested_ = false;
      compute_completed_ = false;
      compute_thread_ = std::thread(&My_effort_controller::computeWorkerLoop, this);
    }

  } 
  catch (const std::exception& e) 
  {
    LOGE(get_node(), "Exception during initialization: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration My_effort_controller::command_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) 
  {
    command_interface_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return command_interface_config;
}

controller_interface::InterfaceConfiguration My_effort_controller::state_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) 
  {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints; ++i) 
  {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints; ++i) 
  {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) 
  {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }
  state_interfaces_config.names.push_back(arm_id_ + "/robot_time");
  return state_interfaces_config;
}

CallbackReturn My_effort_controller::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  if (get_node()->get_parameter("arm_id", arm_id_)) 
  {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
  }
  else
  {
    LOGW(get_node(), "Parameter 'arm_id' not set — using defaults fr3");
    arm_id_ = "fr3";
  }

  const Eigen::Vector7d KP_DEFAULT = (Eigen::Vector7d() << 600,600,600,600,250,150,50).finished();
  const Eigen::Vector7d KV_DEFAULT = (Eigen::Vector7d() <<  30, 30, 30, 30, 10, 10, 5).finished();

  kp_joint_ = KP_DEFAULT;
  kv_joint_ = KV_DEFAULT;

  std::vector<double> kp_in;
  if (get_node()->get_parameter("kp_joint_gains", kp_in)) 
  {
    if (kp_in.size() == static_cast<size_t>(num_joints)) 
    {
      for (int i = 0; i < num_joints; ++i) kp_joint_[i] = kp_in[i];
    } 
    else 
    {
      LOGW(get_node(), "Parameter 'kp_joint_gains' size mismatch (%zu, expected %d) — using defaults", kp_in.size(), num_joints);
    }
  } 
  else 
  {
    LOGW(get_node(), "Parameter 'kp_joint_gains' not set — using defaults: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f]",
         KP_DEFAULT[0], KP_DEFAULT[1], KP_DEFAULT[2], KP_DEFAULT[3], KP_DEFAULT[4], KP_DEFAULT[5], KP_DEFAULT[6]);
  }


  std::vector<double> kv_in;
  if (get_node()->get_parameter("kv_joint_gains", kv_in)) 
  {
    if (kv_in.size() == static_cast<size_t>(num_joints)) 
    {
      for (int i = 0; i < num_joints; ++i) kv_joint_[i] = kv_in[i];
    } 
    else 
    {
      LOGW(get_node(), "Parameter 'kv_joint_gains' size mismatch (%zu, expected %d) — using defaults",
            kv_in.size(), num_joints);
    }
  } 
  else 
  {
    LOGW(get_node(), "Parameter 'kv_joint_gains' not set — using defaults: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f]",
         KV_DEFAULT[0], KV_DEFAULT[1], KV_DEFAULT[2], KV_DEFAULT[3], KV_DEFAULT[4], KV_DEFAULT[5], KV_DEFAULT[6]);
  }

  LOGI(get_node(), "arm_id: %s | kp: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f] | kv: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f]",
       arm_id_.c_str(),
       kp_joint_[0], kp_joint_[1], kp_joint_[2], kp_joint_[3], kp_joint_[4], kp_joint_[5], kp_joint_[6],
       kv_joint_[0], kv_joint_[1], kv_joint_[2], kv_joint_[3], kv_joint_[4], kv_joint_[5], kv_joint_[6]);

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + "robot_model",
                                                   arm_id_ + "/" + "robot_state"));

  return CallbackReturn::SUCCESS;
}

CallbackReturn My_effort_controller::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  initialization_flag_ = true;

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  updateJointStates();
  updateRobotData();

  {
    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    q_desired_ = q_;
    qdot_desired_.setZero();
    torque_desired_ = c_;
  }

  LOGI(get_node(), "Controller activated (arm_id: %s, dt: %.3f ms)", arm_id_.c_str(), dt_ * 1000.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn My_effort_controller::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type My_effort_controller::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  SuhanBenchmark bench;

  updateJointStates();
  updateRobotData();

  {
    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    play_time_ = state_interfaces_.back().get_value();
  }

  const double spent_ms = bench.elapsed() * 1000.;
  double budget_ms = (dt_*1000.) - spent_ms - 0.05; // 0.05 ms for command to robot
  if (budget_ms < 0.0) 
  {
    budget_ms = 0.0;
    LOGW(get_node(), "State update exceeded 1.0 ms (%.3f ms)", spent_ms);
  }

  constexpr double kWaitGuardMs = 0.2; // leave headroom for remaining work and comms
  const bool skip_guard = relax_wait_guard_.load(std::memory_order_acquire);
  if (!skip_guard && budget_ms > kWaitGuardMs)
  {
    budget_ms -= kWaitGuardMs;
  }
  else if (!skip_guard)
  {
    budget_ms = 0.0;
  }

  Eigen::Vector7d last_command;
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    last_command = torque_desired_;
  }

  bool used_new_solution = false;

  auto request_compute_and_wait =
      [this](double wait_ms, bool clear_relax_on_success) -> bool
      {
        {
          std::lock_guard<std::mutex> lock(compute_cv_mutex_);
          compute_requested_ = true;
          compute_completed_ = false;
        }
        compute_cv_.notify_one();

        const double clamped_wait = std::max(0.0, std::min(wait_ms, 0.2));
        const auto wait_dur = std::chrono::duration<double, std::milli>(clamped_wait);
        std::unique_lock<std::mutex> wait_lock(compute_cv_mutex_);
        if (compute_done_cv_.wait_for(wait_lock, wait_dur, [this]() { return compute_completed_; }))
        {
          if (clear_relax_on_success)
          {
            relax_wait_guard_.store(false, std::memory_order_release);
          }
          return true;
        }
        return false;
      };
  if (skip_guard)
  {
    if (!compute_inflight_.exchange(true, std::memory_order_acq_rel))
    {
      try
      {
        compute();
        used_new_solution = true;
        relax_wait_guard_.store(false, std::memory_order_release);
      }
      catch (const std::exception& e)
      {
        LOGE(get_node(), "Exception in compute(): %s", e.what());
      }
      catch (...)
      {
        LOGE(get_node(), "Unknown exception in compute()");
      }
      compute_inflight_.store(false, std::memory_order_release);
      {
        std::lock_guard<std::mutex> lock(compute_cv_mutex_);
        compute_completed_ = true;
        compute_requested_ = false;
      }
      compute_done_cv_.notify_all();
    }
  }
  else if (!compute_inflight_.exchange(true, std::memory_order_acq_rel)) 
  {
    if (request_compute_and_wait(budget_ms, false))
    {
      used_new_solution = true;
    }
  } 

  Eigen::Vector7d command;
  if (used_new_solution) 
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    command = torque_desired_;
    relax_wait_guard_.store(false, std::memory_order_release);
  } 
  else 
  {
    command = last_command;
  }

  for (int i = 0; i < num_joints; ++i) 
  {
    command_interfaces_[i].set_value(command[i]);
  }

  return controller_interface::return_type::OK;
}

// ========================================================================
// ====================== Main Controller Functions =======================
// ========================================================================
void My_effort_controller::compute()
{
  std::scoped_lock(robot_data_mutex_, calculation_mutex_);
  if (initialization_flag_ || is_mode_changed_) 
  {            
    control_start_time_ = play_time_;

    q_init_ = q_;
    qdot_init_ = qdot_;
    q_desired_ = q_init_;
    qdot_desired_.setZero();

    x_init_ = x_;
    xdot_init_ = xdot_;
    x_desired_ = x_init_;
    xdot_desired_.setZero();

    initialization_flag_ = false;
    is_mode_changed_ = false;
  }

  if(control_mode_ == CtrlMode::HOME)
  {
    Eigen::Vector7d home_q; 
    home_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
    q_desired_ = DyrosMath::cubicVector<7>(play_time_,
                                           control_start_time_,
                                           control_start_time_ + 4.0,
                                           q_init_,
                                           home_q,
                                           qdot_init_,
                                           Eigen::Vector7d::Zero());

    qdot_desired_ = DyrosMath::cubicDotVector<7>(play_time_,
                                                 control_start_time_,
                                                 control_start_time_ + 4.0,
                                                 q_init_,
                                                 home_q,
                                                 qdot_init_,
                                                 Eigen::Vector7d::Zero());
    
    torque_desired_ = JointPDControl(q_desired_, qdot_desired_);
  }
  else
  {
    q_desired_ = q_;
    qdot_desired_.setZero();
    torque_desired_ = c_;
  }
}

void My_effort_controller::updateJointStates() 
{
  std::lock_guard<std::mutex> lk(robot_data_mutex_);
  for (int i = 0; i < num_joints; ++i) 
  {
    const auto& position_interface = state_interfaces_.at(i);
    const auto& velocity_interface = state_interfaces_.at(num_joints + i);
    const auto& effort_interface = state_interfaces_.at(2 * num_joints + i);
    q_[i] = position_interface.get_value();
    qdot_[i] = velocity_interface.get_value();
    torque_[i] = effort_interface.get_value();
  }
}

void My_effort_controller::updateRobotData()
{
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  std::array<double, 42> endeffector_jacobian_wrt_base = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  {
    std::lock_guard<std::mutex> lock(robot_data_mutex_);
    M_ = Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>(mass.data());
    c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis.data());
    g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity.data());

    x_.matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());

    Eigen::Map<const Eigen::Matrix<double,6,7,Eigen::ColMajor>> J_tmp(endeffector_jacobian_wrt_base.data());
    J_ = J_tmp;

    M_inv_ = M_.inverse();
    xdot_ = J_ * qdot_;
    // M_task_ = (J_ * M_inv_ * J_.transpose()).inverse();
    // J_T_inv_ = M_task_ * J_ * M_inv_;
  }
}

void My_effort_controller::computeWorkerLoop()
{
  std::unique_lock<std::mutex> lock(compute_cv_mutex_);
  while (!stop_compute_thread_)
  {
    compute_cv_.wait(lock, [this]() { return compute_requested_ || stop_compute_thread_; });
    if (stop_compute_thread_)
    {
      break;
    }
    compute_requested_ = false;
    lock.unlock();
    try
    {
      compute();
    }
    catch (const std::exception& e)
    {
      LOGE(get_node(), "Exception in compute(): %s", e.what());
    }
    catch (...)
    {
      LOGE(get_node(), "Unknown exception in compute()");
    }
    lock.lock();
    compute_completed_ = true;
    compute_inflight_.store(false, std::memory_order_release);
    compute_done_cv_.notify_all();
  }
}

void My_effort_controller::setMode(CtrlMode mode)
{
  std::scoped_lock<std::mutex, std::mutex> lk(robot_data_mutex_, calculation_mutex_);

  if (control_mode_ == mode) 
  {
    LOGI(get_node(), "Mode unchanged: %d", static_cast<int>(mode));
    return;
  }
  control_mode_ = mode;
  is_mode_changed_ = true;

  LOGI(get_node(), "Mode changed: %d", static_cast<int>(control_mode_));
}

// ========================================================================
// =========================== ROS Subs & Pubs  ===========================
// ========================================================================
void My_effort_controller::controlModeCallback(const std_msgs::msg::Int32& msg)
{
  LOGI(get_node(), "Mode input received: %d", msg.data);
  switch (msg.data) 
  {
    case 0: setMode(CtrlMode::NONE); break;
    case 1: setMode(CtrlMode::HOME); break;
    default: LOGW(get_node(), "Unknown mode value: %d (ignored)", msg.data); break;
  }
}


// ========================================================================
// ========================== Utility Functions ===========================
// ========================================================================	
Eigen::Vector7d My_effort_controller::JointPDControl(const Eigen::Vector7d target_q, const Eigen::Vector7d target_qdot)
{
  Eigen::Vector7d q_error = target_q - q_;
  Eigen::Vector7d qdot_error = target_qdot - qdot_;
  return kp_joint_.asDiagonal() * (q_error) + kv_joint_.asDiagonal() * (qdot_error) + c_;
}



}  // namespace dyros_fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dyros_fr3_controllers::My_effort_controller,
                       controller_interface::ControllerInterface)

        