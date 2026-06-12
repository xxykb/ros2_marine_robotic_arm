// landing_tracker.cpp — KUKA KR600 drone landing platform tracker
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
using moveit::core::MoveItErrorCode;

class LandingTracker : public rclcpp::Node {
public:
  LandingTracker() : Node("landing_tracker"), move_group_(nullptr) {
    this->declare_parameter("group_name", "manipulator");
    this->declare_parameter("vessel_frame", "wamv/base_link");
    this->declare_parameter("tracking_rate", 20.0);
    this->declare_parameter("safety_offset_z", 0.3);
    this->declare_parameter("touchdown_height", 1.4);
    this->declare_parameter("touchdown_time", 1.0);
    this->declare_parameter("planning_time", 0.05);
    group_name_ = this->get_parameter("group_name").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/ground_truth_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        last_drone_pose_ = m->pose; drone_seen_ = true;
      });

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/landing_status", 10);

    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/start_landing",
      std::bind(&LandingTracker::onStart, this,
                std::placeholders::_1, std::placeholders::_2));
    abort_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/abort_landing",
      std::bind(&LandingTracker::onAbort, this,
                std::placeholders::_1, std::placeholders::_2));

    init_timer_ = this->create_wall_timer(500ms, [this]() {
      init_timer_->cancel();
      move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), group_name_);
      move_group_->setPlanningTime(this->get_parameter("planning_time").as_double());
      move_group_->setMaxVelocityScalingFactor(0.3);
      move_group_->setMaxAccelerationScalingFactor(0.2);
      RCLCPP_INFO(this->get_logger(), "MoveGroup ready. Planning frame: %s",
                  move_group_->getPlanningFrame().c_str());
      double rate = this->get_parameter("tracking_rate").as_double();
      tracking_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/rate), [this]() { trackingLoop(); });
    });

    setState("IDLE");
    RCLCPP_INFO(this->get_logger(), "Landing Tracker initialized. Waiting for /start_landing ...");
  }

private:
  enum class State { IDLE, TRACKING, LANDING, RETRACT };
  State state_ = State::IDLE;
  std::string group_name_;
  std::shared_ptr<MoveGroupInterface> move_group_;

  geometry_msgs::msg::Pose last_drone_pose_;
  bool drone_seen_ = false;
  rclcpp::Time touchdown_start_;
  bool touchdown_stable_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, abort_srv_;
  rclcpp::TimerBase::SharedPtr init_timer_, tracking_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void onStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    if (!move_group_) { res->success = false; res->message = "MoveIt not ready"; return; }
    if (state_ == State::IDLE) { setState("TRACKING"); res->success = true; res->message = "Started"; }
    else { res->success = false; res->message = "State: " + stateToString(state_); }
  }

  void onAbort(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    setState("RETRACT"); res->success = true; res->message = "Aborting";
  }

  void setState(const std::string& s) {
    if (s=="IDLE") state_=State::IDLE;
    else if (s=="TRACKING") state_=State::TRACKING;
    else if (s=="LANDING") state_=State::LANDING;
    else if (s=="RETRACT") state_=State::RETRACT;
    auto m = std_msgs::msg::String(); m.data=stateToString(state_); status_pub_->publish(m);
  }
  std::string stateToString(State s) {
    switch(s) {
      case State::IDLE: return "IDLE";
      case State::TRACKING: return "TRACKING";
      case State::LANDING: return "LANDING";
      case State::RETRACT: return "RETRACT";
    }
    return "UNKNOWN";
  }

  void trackingLoop() {
    if (!move_group_) return;
    if (state_==State::TRACKING) { trackingStep(); checkLandingCondition(); }
    else if (state_==State::RETRACT) retractToHome();
  }

  bool transformDroneToVessel(geometry_msgs::msg::Pose& vp) {
    std::string vf = this->get_parameter("vessel_frame").as_string();
    geometry_msgs::msg::PoseStamped wp; wp.header.frame_id="world"; wp.pose=last_drone_pose_;
    try { vp=tf_buffer_->transform(wp,vf).pose; return true; }
    catch(const tf2::TransformException&) { return false; }
  }

  void trackingStep() {
    if (!drone_seen_) return;
    geometry_msgs::msg::Pose dv;
    if (!transformDroneToVessel(dv)) return;
    double off=this->get_parameter("safety_offset_z").as_double();
    geometry_msgs::msg::Pose tgt=dv;
    tgt.position.z-=off; tgt.orientation.w=1.0;
    move_group_->setApproximateJointValueTarget(tgt,"tool0");
    MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan)==MoveItErrorCode::SUCCESS) move_group_->asyncExecute(plan);
  }

  void checkLandingCondition() {
    if (!drone_seen_) return;
    double tdz=this->get_parameter("touchdown_height").as_double();
    double tdt=this->get_parameter("touchdown_time").as_double();
    geometry_msgs::msg::Pose dv;
    if (!transformDroneToVessel(dv)) return;
    if (dv.position.z<tdz) {
      if (!touchdown_stable_) { touchdown_start_=this->now(); touchdown_stable_=true; }
      if ((this->now()-touchdown_start_).seconds()>tdt) {
        RCLCPP_INFO(this->get_logger(),"LANDING CONFIRMED!");
        setState("LANDING"); touchdown_stable_=false;
      }
    } else { touchdown_stable_=false; }
  }

  void retractToHome() {
    move_group_->setNamedTarget("home");
    MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan)==MoveItErrorCode::SUCCESS) {
      move_group_->asyncExecute(plan); setState("IDLE");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions o; o.automatically_declare_parameters_from_overrides(true);
  auto n = std::make_shared<LandingTracker>();
  rclcpp::executors::SingleThreadedExecutor e; e.add_node(n); e.spin();
  rclcpp::shutdown();
  return 0;
}