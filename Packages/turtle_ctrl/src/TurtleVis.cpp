#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <turtle_ctrl/TurtleVis.h>

using std::placeholders::_1;

namespace turtle_examples
{
TurtleVis::TurtleVis(/* args */) : Node("turtle_visualizer")
{
  // Init Parameters. Load node parameters
  init();

  // Subscription to turtle topic
  subscription_ = create_subscription<turtle_msgs::msg::TurtleStateStamped>(
      subs_topic_name_, 10, std::bind(&TurtleVis::topic_callback, this, _1));

  // Marker publisher
  v_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_name_, 10);

  // Instantiate tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Marker
  turtle_marker_.header.frame_id = frame_id_;
  //   turtle_marker_.header.stamp = It must be defined before publishing the
  //   message;
  turtle_marker_.ns = marker_namespace_;
  turtle_marker_.id = 3452;
  turtle_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  turtle_marker_.action = visualization_msgs::msg::Marker::ADD;
  turtle_marker_.pose.position.x = 0.0;
  turtle_marker_.pose.position.y = 0.0;
  turtle_marker_.pose.position.z = 0.05;
  turtle_marker_.pose.orientation.x = 0;
  turtle_marker_.pose.orientation.y = 0;
  turtle_marker_.pose.orientation.z = 0.7071067811865475;
  turtle_marker_.pose.orientation.w = 0.7071067811865475;
  turtle_marker_.scale.x = 0.06;
  turtle_marker_.scale.y = 0.06;
  turtle_marker_.scale.z = 0.06;
  // This is the CAD model we want to visualize
  turtle_marker_.mesh_resource = "package://turtle_ctrl/urdf/meshes/turtle2.dae";
  turtle_marker_.mesh_use_embedded_materials = true;

  turtle_marker_.color.r = 0.0;
  turtle_marker_.color.g = 0.7;
  turtle_marker_.color.b = 0.5;
  turtle_marker_.color.a = 1.0;
  turtle_marker_.lifetime = rclcpp::Duration(0);
  turtle_marker_.frame_locked = true;
}

TurtleVis::~TurtleVis()
{
}

void TurtleVis::init()
{

  // Manual parameter setting
  frame_id_ = "/turtle_frame";
  subs_topic_name_ = "/turtle_pose";
  marker_topic_name_ = "/turtle_marker";
  marker_namespace_ = "turtle_marker";

  // Using ROS parameters
  // Declare the ros parameters
  std::vector<std::string> param_names = { "frame_id", "subs_topic_name", "marker_topic_name", "marker_namespace" };
  for (auto &&i : param_names)
    declare_parameter(i);
  
  // Get the ros parameters
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  // Assign the ros parameters to member variables
  frame_id_ = params.at(0).as_string();
  subs_topic_name_ = params.at(1).as_string();
  marker_topic_name_ = params.at(2).as_string();
  marker_namespace_ = params.at(3).as_string();
}

void TurtleVis::topic_callback(const turtle_msgs::msg::TurtleStateStamped::SharedPtr msg)
{
  // RCLCPP_INFO_STREAM(get_logger(), "Received msg" <<
  // msg->header.stamp.nanosec);

  // Define the marker array container to publish as a message. 
  // This is needed to visualize the CAD model in Rviz
  visualization_msgs::msg::MarkerArray m_marker_msg;

  // Each marker is connected with a TF (frame_id)
  geometry_msgs::msg::TransformStamped ts;
  
  // In this example, we will use a TF vector to show how to publish multiple TFs
  // However, we will only need one TF for the Marker
  std::vector<geometry_msgs::msg::TransformStamped> v_ts;

  // Get the current time
  rclcpp::Time aux_time = now();

  // TF object to populate our TF message
  tf2::Transform tf;

  // Auxiliary quaternion object to populate the TF message
  tf2::Quaternion qtf;

  // Populate the Marker Array to visualize the ATR as a mesh
  turtle_marker_.header.stamp = aux_time;

  // Get pose from msg
  // We define a quaternion using a basic rotation in z (Yaw)
  qtf.setRPY(0, 0, msg->pose.theta);

  // Set the position of the TF
  tf.setOrigin(tf2::Vector3(msg->pose.x, msg->pose.y, 0));

  // Set the orientation of the TF
  tf.setRotation(qtf);

  // Transform the TF object to TF message
  ts.transform = tf2::toMsg(tf);

  // Set the reference frame for the TF (parent link)
  ts.header.frame_id = "/world";
  // Set the time stamp for the message
  ts.header.stamp = aux_time;
  // Set the name for the TF
  ts.child_frame_id = frame_id_;

  //// To visualize objects in Rviz, we need to publish the marker (object) and its corresponding TF
  // Create TF msg
  v_ts.push_back(ts);
  // Publish transformations
  tf_broadcaster_->sendTransform(v_ts);
  
  // Create MarkerArray msg
  m_marker_msg.markers.push_back(turtle_marker_);
  // Publish Polygons as Line Markers
  v_marker_publisher_->publish(m_marker_msg);


}

}  // namespace turtle_examples