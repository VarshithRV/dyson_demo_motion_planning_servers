#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include "ur16e_move_server/action/move_to_pose.hpp"

using namespace std::chrono_literals;

class MoveToPoseServer : public rclcpp::Node
{
public:
  using MoveToPose = ur16e_move_server::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

  MoveToPoseServer() : Node("move_to_pose_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "ur_manipulator");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "rws_gripper_tcp");
    approach_height_ = declare_parameter<double>("approach_height", 0.15);
    eef_step_ = declare_parameter<double>("eef_step", 0.01);
    cartesian_min_fraction_ = declare_parameter<double>("cartesian_min_fraction", 0.90);
    place_px_ = declare_parameter<double>("place_pose.x", -0.5);
    place_py_ = declare_parameter<double>("place_pose.y", 0.5);
    place_pz_ = declare_parameter<double>("place_pose.z", 0.01);
    max_ik_attempts_ = declare_parameter<int>("max_ik_attempts", 10);
    use_pre_action_pose_ = declare_parameter<bool>("use_pre_action_pose", true);
    
    pre_action_joints_ = {
      {"shoulder_pan_joint",   1.20110777344728},
      {"shoulder_lift_joint", -1.5901812058332483},
      {"elbow_joint",          1.4020975961789934},
      {"wrist_1_joint",       -1.3838437395893834},
      {"wrist_2_joint",       -1.5707963267948966},
      {"wrist_3_joint",       -0.36870762683331165}
    };
    
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr) {
        have_joint_state_.store(true);
      });

    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_to_pose",
      std::bind(&MoveToPoseServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveToPoseServer::on_cancel, this, std::placeholders::_1),
      std::bind(&MoveToPoseServer::on_accept, this, std::placeholders::_1));

    init_timer_ = create_wall_timer(0ms, std::bind(&MoveToPoseServer::delayed_init, this));
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::atomic<bool> have_joint_state_{false};
  bool mgi_ready_{false};
  std::mutex exec_mtx_;
  std::string planning_group_, end_effector_link_;
  double approach_height_, eef_step_, cartesian_min_fraction_;
  double place_px_, place_py_, place_pz_;
  int max_ik_attempts_;
  bool use_pre_action_pose_;
  std::map<std::string, double> pre_action_joints_;

  geometry_msgs::msg::Pose shift_z(const geometry_msgs::msg::Pose& p, double dz) {
    auto out = p; out.position.z += dz; return out;
  }

  geometry_msgs::msg::Pose make_pose(double x, double y, double z) {
    geometry_msgs::msg::Pose p;
    p.position.x = x; p.position.y = y; p.position.z = z;
    p.orientation.x = 1.0; p.orientation.w = 0.0;
    return p;
  }

  bool move_to_pre_action_pose(const std::string&) {
    const auto* jmg = move_group_->getCurrentState()->getJointModelGroup(planning_group_);
    moveit::core::RobotState target = *move_group_->getCurrentState();
    
    for (const auto& jp : pre_action_joints_) {
      const auto* jm = target.getRobotModel()->getJointModel(jp.first);
      if (jm) {
        double val = jp.second;
        target.setJointPositions(jm, &val);
      }
    }
    target.update();
    
    if (!target.satisfiesBounds(jmg)) {
      target.enforceBounds(jmg);
    }
    
    move_group_->setJointValueTarget(target);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setPlanningTime(10.0);
    
    return plan_and_execute();
  }

  bool move_to_pose_via_ik(const geometry_msgs::msg::Pose& target, const std::string& tag) {
    const auto* jmg = move_group_->getCurrentState()->getJointModelGroup(planning_group_);
    moveit::core::RobotState current = *move_group_->getCurrentState();
    
    std::vector<double> best_sol;
    double best_dist = std::numeric_limits<double>::max();
    bool found = false;
    
    for (int i = 0; i < max_ik_attempts_; ++i) {
      moveit::core::RobotState test = current;
      if (test.setFromIK(jmg, target, end_effector_link_, 0.1)) {
        double dist = test.distance(current, jmg);
        if (dist < best_dist) {
          best_dist = dist;
          test.copyJointGroupPositions(jmg, best_sol);
          found = true;
        }
      }
    }
    
    if (!found) {
      RCLCPP_ERROR(get_logger(), "[%s] IK failed", tag.c_str());
      return false;
    }
    
    moveit::core::RobotState tgt = current;
    tgt.setJointGroupPositions(jmg, best_sol);
    move_group_->setJointValueTarget(tgt);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setPlanningTime(10.0);
    
    return plan_and_execute();
  }

  bool exec_cartesian(const std::vector<geometry_msgs::msg::Pose>& wpts, const std::string& tag) {
    if (wpts.size() < 2) return true;
    
    for (int i = 1; i <= 3; ++i) {
      moveit_msgs::msg::RobotTrajectory traj;
      double frac = move_group_->computeCartesianPath(wpts, eef_step_, 0.0, traj, true);
      
      if (frac >= cartesian_min_fraction_) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          return true;
        }
      }
      if (i < 3) rclcpp::sleep_for(500ms);
    }
    RCLCPP_ERROR(get_logger(), "[%s] Failed", tag.c_str());
    return false;
  }

  bool plan_and_execute() {
    for (int i = 1; i <= 3; ++i) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          return true;
        }
      }
      if (i < 3) {
        move_group_->setStartStateToCurrentState();
        rclcpp::sleep_for(500ms);
      }
    }
    return false;
  }

  bool wait_for_joint_states(double timeout) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate r(100);
    while (rclcpp::ok()) {
      if (have_joint_state_.load()) return true;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout)
        return false;
      r.sleep();
    }
    return false;
  }

  void add_ground() {
    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id = "world";
    ground.id = "ground";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {4.0, 4.0, 0.02};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.z = -0.05;
    ground.primitives.push_back(box);
    ground.primitive_poses.push_back(pose);
    ground.operation = ground.ADD;
    psi_->applyCollisionObjects({ground});
  }

  void delayed_init() {
    if (mgi_ready_) return;
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->startStateMonitor();
    psi_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    wait_for_joint_states(3.0);
    add_ground();
    mgi_ready_ = true;
    init_timer_->cancel();
  }

  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID&,
                                       std::shared_ptr<const MoveToPose::Goal>) {
    if (!mgi_ready_ || !exec_mtx_.try_lock()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    exec_mtx_.unlock();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle>) {
    if (move_group_) move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accept(const std::shared_ptr<GoalHandle> gh) {
    std::thread([this, gh]() { execute(gh); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> gh) {
    std::lock_guard<std::mutex> lk(exec_mtx_);
    auto result = std::make_shared<MoveToPose::Result>();
    auto goal = gh->get_goal();

    try {
      if (!move_group_ || gh->is_canceling()) throw std::runtime_error("Not ready");
      if (!wait_for_joint_states(2.0)) throw std::runtime_error("No joint states");

      auto pick_tgt = goal->target_pose.pose;
      auto pick_app = shift_z(pick_tgt, approach_height_);
      auto place_tgt = make_pose(place_px_, place_py_, place_pz_);
      auto place_app = shift_z(place_tgt, approach_height_);

      move_group_->setStartStateToCurrentState();
      
      if (!move_to_pose_via_ik(pick_app, "pick")) throw std::runtime_error("Step 1");
      rclcpp::sleep_for(500ms);

      if (!exec_cartesian({pick_app, pick_tgt}, "descend")) throw std::runtime_error("Step 2");
      rclcpp::sleep_for(300ms);

      if (gh->is_canceling()) throw std::runtime_error("Canceled");
      rclcpp::sleep_for(800ms);

      if (!exec_cartesian({pick_tgt, pick_app}, "lift")) throw std::runtime_error("Step 4");
      rclcpp::sleep_for(500ms);

      if (use_pre_action_pose_) {
        move_group_->setStartStateToCurrentState();
        if (!move_to_pre_action_pose("pre")) throw std::runtime_error("Step 5");
        rclcpp::sleep_for(500ms);
      }

      move_group_->setStartStateToCurrentState();

      if (!move_to_pose_via_ik(place_app, "place")) throw std::runtime_error("Step 6");
      rclcpp::sleep_for(500ms);

      if (!exec_cartesian({place_app, place_tgt}, "down")) throw std::runtime_error("Step 7");
      rclcpp::sleep_for(300ms);

      if (gh->is_canceling()) throw std::runtime_error("Canceled");
      rclcpp::sleep_for(800ms);
      if (!exec_cartesian({place_tgt, place_app}, "up")) throw std::runtime_error("Step 8");

      if (use_pre_action_pose_) {
        move_group_->setStartStateToCurrentState();
        if (!move_to_pre_action_pose("final")) throw std::runtime_error("Step 9");
      }

      result->result = true;
      gh->succeed(result);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed: %s", e.what());
      result->result = false;
      gh->abort(result);
      if (move_group_) {
        move_group_->stop();
        move_group_->clearPoseTargets();
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToPoseServer>());
  rclcpp::shutdown();
  return 0;
}
