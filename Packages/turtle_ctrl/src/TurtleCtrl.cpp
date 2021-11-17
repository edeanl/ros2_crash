#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <turtle_ctrl/TurtleCtrl.h>

using std::placeholders::_1;

namespace turtle_examples
{
TurtleCtrl::TurtleCtrl(/* args */)
  : Node("turtle_control"), K_(Eigen::Matrix3d::Zero()), first_message_(false), init_ctrl_(true)
{
  // Init Parameters. Load node parameters
  init();

  // Create subscriber to receive the commanded turtle state. This state will be generated from a trajectory 
  // generator
  sub_cmd_pose_ = create_subscription<turtle_msgs::msg::TurtleStateStamped>(
      subs_topic_name_, 10, std::bind(&TurtleCtrl::topic_callback, this, _1));

  // Create a publisher to update the current turtle state. This state will be used by the visualizer
  pub_current_pose_ = create_publisher<turtle_msgs::msg::TurtleStateStamped>(pub_topic_name_, 10);

  // Instantiate tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // MAIN control thread. Using the commanded turtle state, the controller will move the turtle and 
  // publish the current position
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(ctrl_period_), std::bind(&TurtleCtrl::timer_callback, this));
}

TurtleCtrl::~TurtleCtrl()
{
}

void TurtleCtrl::init()
{
  

  // Manual parameter setting
  frame_id_ = "/turtle_cmd";
  subs_topic_name_ = "/turtle_cmd";
  pub_topic_name_ = "/turtle_pose";
  std::vector<double> k = { 0.1, 0.1, 0.01 };
  ctrl_period_ = 1;
  turtle_init_ = { 0, 0, 0 };

  // ROS parameters
  // Declare the ros parameters 
  std::vector<std::string> param_names = { "frame_id", "subs_topic_name", "pub_topic_name",
                                           "gains",    "ctrl_period",     "turtle_init" };
  for (auto &&i : param_names)
    declare_parameter(i);

  // Get the ros parameters
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  // Assign the ros parameters to member variables
  frame_id_ = params.at(0).as_string();
  subs_topic_name_ = params.at(1).as_string();
  pub_topic_name_ = params.at(2).as_string();
  k = params.at(3).as_double_array();
  ctrl_period_ = params.at(4).as_int();
  turtle_init_ = params.at(5).as_double_array();

  // Populate the gain Matrix
  for (size_t i = 0; i < 3; i++)
  {
    K_(i, i) = k.at(i);
  }

  RCLCPP_INFO_STREAM(get_logger(), "K:  \n" << K_);
}

void TurtleCtrl::topic_callback(const TurtleStateShPt msg)
{
  //   std::lock_guard<std::mutex> guard(data_mutex_);

  // The variable turtle_cmd_msg_ will be shared between the subscriber thread and the main control thread
  // We need to protect the reading/writing process using mutex
  data_mutex_.lock();
  turtle_cmd_msg_ = msg;
  data_mutex_.unlock();

  // TF message to visualize the commanded Turtle State
  std::vector<geometry_msgs::msg::TransformStamped> v_ts;
  geometry_msgs::msg::TransformStamped ts;

  // TF to populate the TF message
  tf2::Transform tf;

  // Auxiliary quaternion to define the orientation of the TF
  tf2::Quaternion qtf;

  // Get pose from msg
  // Convert a basic rotation in z to quaterion
  qtf.setRPY(0, 0, msg->pose.theta);
  // Set the position of the TF
  tf.setOrigin(tf2::Vector3(msg->pose.x, msg->pose.y, 0));
  // Set the orientation of the TF 
  tf.setRotation(qtf);

  // Transform the TF to TF message
  ts.transform = tf2::toMsg(tf);
  // Set the reference frame
  ts.header.frame_id = "/world";
  // Set the time stamp for the message
  ts.header.stamp = now();
  // Define the name of the TF
  ts.child_frame_id = frame_id_;

  // Create ts msg
  v_ts.push_back(ts);

  // Publish the TF
  tf_broadcaster_->sendTransform(v_ts);

  // Flag to control when we have received a commanded turtle pose
  // This flag will activate the controller
  first_message_ = true;
}

void TurtleCtrl::timer_callback()
{
  // When we have received a commanded Turtle state, we start the controller
  if (first_message_)
  {
    // turtle_cmd_msg_ is shared between two threads, therfore we need to protect it, using mutex.
    data_mutex_.lock();
    // We make a local copy of the shared variable. From this point on, we will work ONLY with the local copy, 
    // and not with the shared variable. This is to avoid locking the other thread.
    TurtleStateShPt local_turtle_cmd = turtle_cmd_msg_;
    data_mutex_.unlock();

    // Control
    // Vector for the commanded turtle pose
    Eigen::Vector3d turtle_cmd;
    // Pose error 
    Eigen::Vector3d deltaX;

    // We initialize the controller internal state with the first turtle state message
    if (init_ctrl_)
    {
      // Set the pose vector with the pose from the message
      turtle_current_ << turtle_init_.at(0), turtle_init_.at(1), turtle_init_.at(2);
      // The pose initialization is done only the first time
      init_ctrl_ = false;
    }

    //  Set the commanded turtle pose using the turtle state message information
    turtle_cmd << local_turtle_cmd->pose.x, local_turtle_cmd->pose.y, local_turtle_cmd->pose.theta;

    // Calculate the error between the commanded pose and the current pose
    deltaX = turtle_cmd - turtle_current_;

    // Update the position of the turtle based on this error (simple Proportional control)
    turtle_current_ += K_ * deltaX;

    // Publish the current turtle state as a TurtleState message
    // This is important to debug the controller
    // Setting the time stamp for the message
    local_turtle_cmd->header.stamp = now();
    // Setting the current pose
    local_turtle_cmd->pose.x = turtle_current_(0);
    local_turtle_cmd->pose.y = turtle_current_(1);
    local_turtle_cmd->pose.theta = turtle_current_(2);

    // Publishing the current TurtleState message
    pub_current_pose_->publish(*local_turtle_cmd.get());
  }
}

}  // namespace turtle_examples