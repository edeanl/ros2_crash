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

#ifndef TURTLE_CTRL_H
#define TURTLE_CTRL_H

/*! \file TurtleCtrl.h
 *   \brief Controls the turtle to move from the current state to the desired state
 *
 *   Provides the following functionalities:
 *     - Subscriber to a TurtleStateStamped topic (desired state)
 *     - Publisher of a TurtleStateStamped topic (for the turtle visualizer)
 *     - TF frames
 */

// Eigen Library
#include <Eigen/Dense>
#include <Eigen/StdVector>

// Custom ros messages
#include <turtle_msgs/msg/turtle_state_stamped.hpp>

// ros library
#include "rclcpp/rclcpp.hpp"

// tf publisher
#include <tf2_ros/transform_broadcaster.h>

namespace turtle_examples {

class TurtleCtrl : public rclcpp::Node {
public:
  using SubTurtleState =
      rclcpp::Subscription<turtle_msgs::msg::TurtleStateStamped>::SharedPtr;  ///< Definition for TurtleState subscriber
  using PubTurtleState =
      rclcpp::Publisher<turtle_msgs::msg::TurtleStateStamped>::SharedPtr;     ///< Definition for TurtleState publisher
  using TurtleStateShPt = turtle_msgs::msg::TurtleStateStamped::SharedPtr;    ///< Definition for TurtleState shared ptr

private:
  SubTurtleState sub_cmd_pose_;                                     ///< Subscriber commanded turtle pose
  PubTurtleState pub_current_pose_;                                 ///< Publisher current turtle pose 
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;   ///< Publisher for TF (to display commanded pose)

  std::string frame_id_;            ///< frame id for the commanded pose
  std::string subs_topic_name_;     ///< topic name for the subscriber (commanded pose)
  std::string pub_topic_name_;      ///< topic name for the turtle state publisher (for visualization)
  Eigen::Matrix3d K_;               ///< Control gain matrix
  Eigen::Vector3d turtle_current_;  ///< Current turtle position vector 3X1
  std::vector<double> turtle_init_; ///< Initial position of the turtle
  int ctrl_period_;                 ///< Control period in ms

  std::mutex data_mutex_;           ///< Mutex to protect the reading/writing process
  TurtleStateShPt turtle_cmd_msg_;  ///< Shared object to share the turtle state between the control loop and the
                                    ///< subscriber
  bool first_message_;              ///< Flag to control when we have the turtle state
  bool init_ctrl_;                  ///< Flag to set the turtle initial position using the TurtleState

public:
  /**
   * @brief Default constructor
   * 
   */
  TurtleCtrl(/* args */);

  /**
   * @brief Destroy the Turtle Ctrl object
   * 
   */
  ~TurtleCtrl();

  /**
   * @brief Control initialization, sets the control and node parameters
   * 
   */
  void init();
  
  /**
   * @brief Callback function to receive the TurtleState topic
   * 
   * @param msg TurtleState
   */
  void topic_callback(const TurtleStateShPt msg);

  /**
   * @brief Callback function for the control loop. Internal thread for the controller.
   * 
   */
  void timer_callback();
};

} // namespace turtle_examples

#endif