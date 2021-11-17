/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
 *
 * \copyright Copyright 2021 Chalmers
 *
 * #### License
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef TURTLE_VIS_H
#define TURTLE_VIS_H

/*! \file TurtleVis.h
 *   \brief Allows to visualize the CAD model of the turtle
 *
 *   Provides the following functionalities:
 *     - Subscriber for TurtleStateStamped topic
 *     - Publisher for the visual markers (to see the CAD model in rviz)
 *     - TF frames
 */

// Eigen Library
#include <Eigen/Dense>
#include <Eigen/StdVector>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// Custom ros messages
#include <turtle_msgs/msg/turtle_state_stamped.hpp>

// Standard ros messages
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ros library
#include "rclcpp/rclcpp.hpp"

namespace turtle_examples
{
class TurtleVis : public rclcpp::Node
{
private:
  visualization_msgs::msg::Marker turtle_marker_;                                         ///< Visualization marker
  rclcpp::Subscription<turtle_msgs::msg::TurtleStateStamped>::SharedPtr subscription_;    ///< topic subscription to get
                                                                                          ///< the Turtle state
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr v_marker_publisher_; ///< marker publisher for rviz
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                         ///< tf publisher need for the
                                                                                          ///< markers

  std::string marker_topic_name_;  ///< topic name for the marker array
  std::string marker_namespace_;   ///< namespace for the marker array
  std::string frame_id_;           ///< reference frame for all the objects, usually "map"
  std::string subs_topic_name_;    ///< the name of the ATRStateList topic which we
                                   ///< will subscribe to

public:
 
  /**
   * @brief Standar constructor
   * 
   */
  TurtleVis(/* args */);

  /**
   * @brief Destroy the Turtle Vis object
   * 
   */
  ~TurtleVis();

  /**
   * @brief initialize the parameters for the turtle visualization
   * 
   */
  void init();

private:
  /**
   * @brief callback function for the topic subscriber
   * 
   * @param msg message with the current Turtle state (pose and velocity)
   */
  void topic_callback(const turtle_msgs::msg::TurtleStateStamped::SharedPtr msg);
};

}  // namespace turtle_examples

#endif