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

// Turtle control class
#include <turtle_ctrl/TurtleCtrl.h>

#include <exception>

int main(int argc, char *argv[]) {
  try {
    // Initialize the ros node
    rclcpp::init(argc, argv);

    // We created an object of the class TurtleVis and run the spin
    // This will trigger the threads in the node
    rclcpp::spin(std::make_shared<turtle_examples::TurtleCtrl>());

    // Stop the ros node
    rclcpp::shutdown();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}