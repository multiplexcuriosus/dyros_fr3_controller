#include "dyros_fr3_controllers/my_effort_controller.h"

#include <algorithm>
#include <array>
#include <dyros_fr3_controllers/robot_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>

namespace dyros_fr3_controllers
{

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
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("kp_joint_gains", {});
    auto_declare<std::vector<double>>("kv_joint_gains", {});
    auto_declare<std::vector<double>>(
        "home_joint_pose",
        {0.0, 0.0, 0.0, -M_PI / 2.0, 0.0, M_PI / 2.0, M_PI / 4.0});
    auto_declare<double>("home_duration_sec", 4.0);
    auto_declare<double>("track_duration_sec", 2.0);

    arm_id_ = get_node()->get_parameter("arm_id").as_string();

    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/robot_model", arm_id_ + "/robot_state");

    use_pinocchio_ = false;
    pinocchio_ready_ = false;

    // Initialize Pinocchio from controller_manager's robot_description.
    auto tmp_node = rclcpp::Node::make_shared(
        "_tmp_urdf_client_" + std::string(get_node()->get_name()));
    auto param_client =
        std::make_shared<rclcpp::SyncParametersClient>(tmp_node, "controller_manager");
    param_client->wait_for_service();
    auto cm_params = param_client->get_parameters({"robot_description"});

    if (cm_params.empty() ||
        cm_params[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      LOGW(get_node(),
           "robot_description not available from controller_manager. "
           "Pinocchio-based IK/fallback disabled.");
      pinocchio_ready_ = false;
    }
    else
    {
      try
      {
        std::string tmp_urdf_xml = cm_params[0].value_to_string();
        pinocchio::urdf::buildModelFromXML(tmp_urdf_xml, model_);
        data_ = pinocchio::Data(model_);
        data_worker_ = pinocchio::Data(model_);

        if (model_.getFrameId(ee_name_) >= static_cast<pinocchio::FrameIndex>(model_.nframes))
        {
          LOGW(get_node(),
               "End-effector frame '%s' not found in model. Falling back to 'fr3_link8'.",
               ee_name_.c_str());
          ee_name_ = "fr3_link8";
        }

        pinocchio_ready_ = true;
        LOGI(get_node(),
             "Pinocchio model loaded successfully from controller_manager robot_description.");
      }
      catch (const std::exception& e)
      {
        pinocchio_ready_ = false;
        LOGW(get_node(),
             "Failed to build Pinocchio model from controller_manager robot_description: %s",
             e.what());
      }
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
    x_desired_.setIdentity();
    xdot_desired_.setZero();
    M_task_.setIdentity();
    g_task_.setZero();
    J_T_inv_.setZero();
    home_q_.setZero();
    track_q_target_.setZero();
    last_ik_solution_.setZero();

    control_mode_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        "my_effort_controller/control_mode",
        rclcpp::QoS(10),
        std::bind(&My_effort_controller::controlModeCallback, this, std::placeholders::_1));

    track_pose_srv_ = get_node()->create_service<dyros_fr3_interfaces::srv::SetTrackPose>(
        "my_effort_controller/track_pose",
        std::bind(&My_effort_controller::trackPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

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
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i)
  {
    conf.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return conf;
}

controller_interface::InterfaceConfiguration My_effort_controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i)
  {
    conf.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints; ++i)
  {
    conf.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints; ++i)
  {
    conf.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }

  // Do NOT request franka semantic model interfaces here.
  // They may not exist in MuJoCo, and requesting them makes activation fail.

  return conf;
}

CallbackReturn My_effort_controller::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  home_duration_sec_ = get_node()->get_parameter("home_duration_sec").as_double();
  track_duration_sec_ = get_node()->get_parameter("track_duration_sec").as_double();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state");

  const Eigen::Vector7d kp_default = (Eigen::Vector7d() << 600, 600, 600, 600, 250, 150, 50).finished();
  const Eigen::Vector7d kv_default = (Eigen::Vector7d() << 30, 30, 30, 30, 10, 10, 5).finished();
  kp_joint_ = kp_default;
  kv_joint_ = kv_default;

  std::vector<double> kp_in;
  if (get_node()->get_parameter("kp_joint_gains", kp_in) && kp_in.size() == static_cast<size_t>(num_joints))
  {
    for (int i = 0; i < num_joints; ++i) kp_joint_[i] = kp_in[i];
  }

  std::vector<double> kv_in;
  if (get_node()->get_parameter("kv_joint_gains", kv_in) && kv_in.size() == static_cast<size_t>(num_joints))
  {
    for (int i = 0; i < num_joints; ++i) kv_joint_[i] = kv_in[i];
  }

  std::vector<double> home_in;
  if (get_node()->get_parameter("home_joint_pose", home_in) && home_in.size() == static_cast<size_t>(num_joints))
  {
    for (int i = 0; i < num_joints; ++i) home_q_[i] = home_in[i];
  }
  else
  {
    home_q_ << 0.0, 0.0, 0.0, -M_PI / 2.0, 0.0, M_PI / 2.0, M_PI / 4.0;
  }

  LOGI(get_node(), "Configured arm_id=%s home_duration=%.2f track_duration=%.2f", arm_id_.c_str(), home_duration_sec_, track_duration_sec_);
  LOGI(get_node(), "Initial home pose: [%.3f %.3f %.3f %.3f %.3f %.3f %.3f]",
       home_q_[0], home_q_[1], home_q_[2], home_q_[3], home_q_[4], home_q_[5], home_q_[6]);

  return CallbackReturn::SUCCESS;
}

CallbackReturn My_effort_controller::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  initialization_flag_ = true;
  use_pinocchio_ = true;  // default to sim-safe fallback

  bool semantic_model_ok = false;

  try
  {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    // Probe whether semantic model is actually usable
    (void)franka_robot_model_->getMassMatrix();
    (void)franka_robot_model_->getGravityForceVector();

    semantic_model_ok = true;
  }
  catch (const std::exception& e)
  {
    LOGW(get_node(),
         "Franka semantic model not usable (%s). Falling back to Pinocchio.",
         e.what());
    semantic_model_ok = false;
  }

  use_pinocchio_ = !semantic_model_ok;

  updateJointStates();

  if (use_pinocchio_)
  {
    if (!pinocchio_ready_)
    {
      LOGE(get_node(), "Pinocchio fallback requested but Pinocchio model is not ready.");
      return CallbackReturn::ERROR;
    }
  }

  updateRobotData();

  {
    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    q_desired_ = q_;
    qdot_desired_.setZero();
    torque_desired_.setZero();
  }

  play_time_ = 0.0;

  LOGI(get_node(), "Controller activated. use_pinocchio=%d", static_cast<int>(use_pinocchio_));
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn My_effort_controller::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_pinocchio_ && franka_robot_model_)
  {
    franka_robot_model_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type My_effort_controller::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  SuhanBenchmark bench;

  updateJointStates();
  updateRobotData();

  {
    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    play_time_ += period.seconds();
  }

  try
  {
    compute();
  }
  catch (const std::exception& e)
  {
    LOGE(get_node(), "Exception in compute(): %s", e.what());
    return controller_interface::return_type::ERROR;
  }
  catch (...)
  {
    LOGE(get_node(), "Unknown exception in compute()");
    return controller_interface::return_type::ERROR;
  }

  Eigen::Vector7d command;
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    command = torque_desired_;
  }

  for (int i = 0; i < num_joints; ++i)
  {
    command_interfaces_[i].set_value(command[i]);
  }

  const double total_ms = bench.elapsed() * 1000.0;
  if (total_ms > dt_ * 1000.0)
  {
    LOGW(get_node(), "Control loop exceeded dt: %.3f ms > %.3f ms", total_ms, dt_ * 1000.0);
  }

  return controller_interface::return_type::OK;
}

void My_effort_controller::compute()
{
  std::scoped_lock lock(robot_data_mutex_, calculation_mutex_);

  if (initialization_flag_ || is_mode_changed_)
  {
    initializeModeTargets();
  }

  switch (control_mode_)
  {
    case CtrlMode::HOME:
    {
      q_desired_ = DyrosMath::cubicVector<7>(play_time_, control_start_time_, control_start_time_ + home_duration_sec_,
                                             q_init_, home_q_, qdot_init_, Eigen::Vector7d::Zero());
      qdot_desired_ = DyrosMath::cubicDotVector<7>(play_time_, control_start_time_, control_start_time_ + home_duration_sec_,
                                                   q_init_, home_q_, qdot_init_, Eigen::Vector7d::Zero());
      torque_desired_ = JointPDControl(q_desired_, qdot_desired_);
      break;
    }

    case CtrlMode::TRACK:
    {
      if (!track_target_valid_)
      {
        q_desired_ = q_;
        qdot_desired_.setZero();
        torque_desired_ = c_;
        break;
      }

      q_desired_ = DyrosMath::cubicVector<7>(play_time_, control_start_time_, control_start_time_ + track_duration_sec_,
                                             q_init_, track_q_target_, qdot_init_, Eigen::Vector7d::Zero());
      qdot_desired_ = DyrosMath::cubicDotVector<7>(play_time_, control_start_time_, control_start_time_ + track_duration_sec_,
                                                   q_init_, track_q_target_, qdot_init_, Eigen::Vector7d::Zero());
      torque_desired_ = JointPDControl(q_desired_, qdot_desired_);
      break;
    }

    case CtrlMode::NONE:
    default:
    {
      q_desired_ = q_;
      qdot_desired_.setZero();
      torque_desired_ = c_;
      break;
    }
  }
}

void My_effort_controller::initializeModeTargets()
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

  if (control_mode_ == CtrlMode::TRACK && track_target_valid_)
  {
    Eigen::Vector7d ik_solution = q_;
    if (solveIk(track_pose_target_, last_ik_solution_, ik_solution))
    {
      track_q_target_ = ik_solution;
      last_ik_solution_ = ik_solution;
      x_desired_ = track_pose_target_;
      LOGI(get_node(), "TRACK target accepted. q_target=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f]",
           track_q_target_[0], track_q_target_[1], track_q_target_[2], track_q_target_[3],
           track_q_target_[4], track_q_target_[5], track_q_target_[6]);
    }
    else
    {
      track_target_valid_ = false;
      LOGW(get_node(), "IK failed for requested TRACK pose. Holding current state.");
    }
  }

  initialization_flag_ = false;
  is_mode_changed_ = false;
}

void My_effort_controller::updateJointStates()
{
  std::lock_guard<std::mutex> lk(robot_data_mutex_);
  for (int i = 0; i < num_joints; ++i)
  {
    q_[i] = state_interfaces_.at(i).get_value();
    qdot_[i] = state_interfaces_.at(num_joints + i).get_value();
    torque_[i] = state_interfaces_.at(2 * num_joints + i).get_value();
  }
}

void My_effort_controller::updateRobotData()
{
  if (!use_pinocchio_)
  {
    std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
    std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
    std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
    std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    std::array<double, 42> jac = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

    std::lock_guard<std::mutex> lock(robot_data_mutex_);
    M_ = Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>(mass.data());
    c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis.data());
    g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity.data());

    x_.matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());

    Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::ColMajor>> J_tmp(jac.data());
    J_ = J_tmp;

    M_inv_ = M_.inverse();
    xdot_ = J_ * qdot_;
    return;
  }

  // Pinocchio fallback for simulation
  pinocchio::forwardKinematics(model_, data_, q_, qdot_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::crba(model_, data_, q_);
  pinocchio::computeGeneralizedGravity(model_, data_, q_);
  pinocchio::nonLinearEffects(model_, data_, q_, qdot_);

  const pinocchio::FrameIndex frame_id =
      model_.getFrameId("fr3_hand_tcp") < model_.nframes
          ? model_.getFrameId("fr3_hand_tcp")
          : model_.getFrameId("fr3_link8");

  Eigen::Matrix<double, 6, Eigen::Dynamic> Jtmp(6, model_.nv);
  pinocchio::getFrameJacobian(
      model_, data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jtmp);

  std::lock_guard<std::mutex> lock(robot_data_mutex_);
  M_ = data_.M;
  M_ = 0.5 * (M_ + M_.transpose());
  c_ = data_.nle;
  g_ = data_.g;
  M_inv_ = M_.inverse();

  const auto& oMf = data_.oMf[frame_id];
  x_.matrix() = oMf.toHomogeneousMatrix();
  J_ = Jtmp.leftCols<7>();
  xdot_ = J_ * qdot_;
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
  std::scoped_lock lk(robot_data_mutex_, calculation_mutex_);
  if (control_mode_ == mode)
  {
    return;
  }
  control_mode_ = mode;
  is_mode_changed_ = true;
  LOGI(get_node(), "Mode changed to %d", static_cast<int>(control_mode_));
}

void My_effort_controller::controlModeCallback(const std_msgs::msg::Int32& msg)
{
  switch (msg.data)
  {
    case 0: setMode(CtrlMode::NONE); break;
    case 1: setMode(CtrlMode::HOME); break;
    case 2: setMode(CtrlMode::TRACK); break;
    default: LOGW(get_node(), "Unknown mode value: %d", msg.data); break;
  }
}

void My_effort_controller::trackPoseCallback(
    const std::shared_ptr<dyros_fr3_interfaces::srv::SetTrackPose::Request> request,
    std::shared_ptr<dyros_fr3_interfaces::srv::SetTrackPose::Response> response)
{
  if (!use_pinocchio_)
  {
    response->success = false;
    response->message = "Pinocchio/URDF unavailable, cannot solve IK for TRACK pose.";
    return;
  }

  {
    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    track_pose_target_ = poseMsgToEigen(request->target_pose);
    track_target_valid_ = true;
  }
  setMode(CtrlMode::TRACK);

  response->success = true;
  response->message = "TRACK pose stored. Controller switched to TRACK mode.";
}

Eigen::Vector7d My_effort_controller::JointPDControl(const Eigen::Vector7d& target_q, const Eigen::Vector7d& target_qdot)
{
  const Eigen::Vector7d q_error = target_q - q_;
  const Eigen::Vector7d qdot_error = target_qdot - qdot_;
  return kp_joint_.asDiagonal() * q_error + kv_joint_.asDiagonal() * qdot_error + c_;
}

Eigen::Affine3d My_effort_controller::poseMsgToEigen(const geometry_msgs::msg::Pose& pose) const
{
  Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (quat.norm() < 1e-9)
  {
    quat = Eigen::Quaterniond::Identity();
  }
  else
  {
    quat.normalize();
  }

  Eigen::Affine3d out = Eigen::Affine3d::Identity();
  out.linear() = quat.toRotationMatrix();
  out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return out;
}

bool My_effort_controller::solveIk(const Eigen::Affine3d& target_pose, const Eigen::Vector7d& seed_q, Eigen::Vector7d& q_solution)
{
  if (!use_pinocchio_)
  {
    return false;
  }

  Eigen::Vector7d q_iter = seed_q;
  const pinocchio::FrameIndex frame_id = model_.getFrameId(ee_name_);
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes))
  {
    return false;
  }

  constexpr int kMaxIter = 120;
  constexpr double kPosTol = 1e-3;
  constexpr double kRotTol = 5e-3;
  constexpr double kLambda = 1e-4;
  constexpr double kStep = 0.2;

  for (int i = 0; i < kMaxIter; ++i)
  {
    pinocchio::forwardKinematics(model_, data_worker_, q_iter);
    pinocchio::updateFramePlacements(model_, data_worker_);

    const auto& oMf = data_worker_.oMf[frame_id];
    const Eigen::Vector3d pos_err = target_pose.translation() - oMf.translation();

    const Eigen::Matrix3d R_err = target_pose.rotation() * oMf.rotation().transpose();
    const Eigen::AngleAxisd aa(R_err);
    Eigen::Vector3d ori_err = Eigen::Vector3d::Zero();
    if (std::abs(aa.angle()) > 1e-9)
    {
      ori_err = aa.axis() * aa.angle();
    }

    if (pos_err.norm() < kPosTol && ori_err.norm() < kRotTol)
    {
      q_solution = q_iter;
      return true;
    }

    Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero();
    pinocchio::computeJointJacobians(model_, data_worker_, q_iter);
    pinocchio::getFrameJacobian(model_, data_worker_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    Eigen::Matrix<double, 6, 1> err6;
    err6.head<3>() = pos_err;
    err6.tail<3>() = ori_err;

    const Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * (J * J.transpose() + kLambda * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
    q_iter += kStep * (J_pinv * err6);
  }

  q_solution = q_iter;
  return false;
}

}  // namespace dyros_fr3_controllers

PLUGINLIB_EXPORT_CLASS(dyros_fr3_controllers::My_effort_controller, controller_interface::ControllerInterface)