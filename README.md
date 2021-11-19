# ROS2 Crash course


## Task 1 Create your new workspace

	mkdir -p /home/usr/ros2/workspaces/ros2_crash_ws/src
	cd /home/usr/ros2/workspaces/ros2_crash_ws/src

## Task 2 Get teleop ros package and modify it
	
	1) Clone the ros package repo
		cd /home/usr/ros2/workspaces/ros2_crash_ws/src/
		git clone git@github.com:ros2/teleop_twist_keyboard.git
		source /opt/ros/foxy/setup.bash
	2) Compile the workspace
		cd /home/usr/ros2/workspaces/ros2_crash_ws/
		colcon build
	3) Run the binary teleop_twist_keyboard.py and play with it! 
		ros2 run teleop_twist_keyboard teleop_twist_keyboard
**Q1**: How can we test our program? Hint: ros topics!

	4) Add a customized print message in the file teleop_twist_keyboard.py and run it again

**Q2**: What happened? Hint: underlay vs overlay
	
	5) Modify the binary teleop_twist_keyboard.py to generate the following motions:

		Linear motions: 

		u: left-front
		i: front
		o: right-front
		j: left
		k: stop
		l: right
		m: left-back
		,: back
		.: right-back

		Angular Motions:
		w: counter-clockwise
		e: stop
		r: clock-wise

		Speed:

		+ increase linear speed +0.1
		- decrease linear speed -0.1
		* increase angular speed +0.1
		/ decrease angular speed -0.1

	6) Test your new binary
		Open a new terminal and source your workspace
		cd /home/usr/ros2/workspaces/ros2_crash_ws/
		source install/setup.bash
		ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=turtle_twist
**Q3**: Does it work? 
		
## Task 3 Create a new ros2 cpp package named "turtle_msg"
	1) Use the ros2 create package function 
		cd /home/usr/ros2/workspaces/ros2_crash_ws/src
		ros2 pkg create --build-type ament_cmake turtle_msgs
	2) Compile the workspace
		colcon build --symlink-install --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
**Q4**: Any problems? Do you notice any difference with the additional compiling arguments?
	
	3) Add msg/TurtleState.msg and modify CMakeLists.txt, package.xml
	4) Compile your workspace
	5) Verify that your new message has been created, using ros2 interface.
**Q5**: Can you find your new message? Hint: overlay

## Task 4 Create a new ros2 cpp package "turtle_ctrl"
	1) Use the ros2 create package function 
		cd /home/dean/ros2/workspaces/ros2_crash_ws/src
		ros2 pkg create --build-type ament_cmake turtle_ctrl
	2) Compile your workspace
	3) Copy the TurtleVis class files TurtleVis.h and TurtleVis.cpp to their corresponding folders
	4) Modify the CMakeFile to create a shared library with the TurtleVis class
	5) Copy the application file turtle_vis.cpp
	6) Modify the CMakeFile to create a new binary (Exec)
	7) Compile your workspace (or single ros package)
	8) Copy the file "turtle.rviz" into turtle_ctr/rviz/turtle.rviz
	9) Test your application. You will need 3 terminals
		T1) Run the turtle visualization 
			ros2 run turtle_ctrl turtle_vis
		T2) Run rviz
			ros2 run rviz2 rviz2 -d src/turtle_ctrl/rviz/turtle.rvi
		T3) Publish a TurtleState message and check the rviz window
			ros2 topic pub --rate 1 /turtle_pose turtle_msgs/msg/TurtleStateStamped "{pose: {x: 1, y: 1, theta: 0}}"
**Q6**: What can you see? 

## Task 5 Let's create a lunch file to automate the above process
	1) Copy the file "turtle_vis_test_launch.py" into the folder turtle_ctrl/launch/
	2) Run the new launch file
		ros2 launch turtle_ctrl turtle_vis_test_launch.py
**Q7**: Does it work? What happened? 


## Task 6 Create the controller
	1) Copy the class files  TurtleCtrl.cpp and TurtleCtrl.h to its corresponding folders
	2) Modify the CMakeFile to create a shared library with the control class
	3) Copy the application file turtle_control.cpp
	4) Modify the CMakeFile to create a new binary (Exec)
	5) Compile your workspace (or single ros package) 
	6) Test your application
		T1) Copy the launch file turtle_vis_test_launch.py and renamed as turtle_test_launch.py
			Modify the new launch file to include the control node
**Q8**: How can you test your application? 

## Task 7 Trajectory generator
	We will implement two methods to generate the commanded turtle pose for the controller.
	First, transform the Twist (turtle velocity) message generated from the node teleop_twist_keyboard to continuous 
	commanded turtle poses. 
	Second, we will create a node that provides a service to request a desired turtle pose. This node will use the
	requested turtle pose and generate a continuous smooth trajectory for the commanded turtle pose. 
	The output of both nodes is a TurtleStateStamped message, which is connected to the controller to change the 
	pose of the turtle
	1) Create a new ros python package "turtle_trajectory_generator"
		ros2 pkg create --build-type ament_python turtle_trajectory_generator
	2) Fix the ros package configuration files to use the new package, e.g. package.xml, resource, setup.py, etc.
	3) Copy the file teleop_twist_keyboard.py from the ros package teleop_twist_keyboard to this new package
	4) Compile the workspace. This is needed to create the symbolic links to the python files. 
		colcon build --symlink-install --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
	5) Create the trajectory generator module. Use the file trajectory_generator_class.py 
	6) Compile again the workspace
	7) Create the application nodes
		a) applications/poly_trajectories.py: Generates the smooth trajectory for the commanded turtle pose (TurtleStateStamped message)
		b) applications/twist2cmd.py: Converts from Twist message to commanded turtle pose (TurtleStateStamped message)
	8) Compile your workspace 
	7) Test your applications
**Q9**: How can you test the trajectory generator nodes only? Hint: ros2 run rqt_plot rqt_plot
**Q9**: Does it work? Hint: did you configure your setup.py file? 



## Running Demo

### Turtle Ctrl + Vis
T1)
	cd /home/dean/ros2/workspaces/ros2_crash_ws/
	source install/setup.bash
	ros2 launch turtle_ctrl turtle_test_launch.py
T2) 
	ros2 topic pub --rate 1 /turtle_pose turtle_msgs/msg/TurtleStateStamped "{pose: {x: 0, y: 1, theta: 0}}"

### Turtle Trajectory + Ctrl + Vis

ros2 launch turtle_ctrl turtle_test_launch.py

**Keyboard commands (Velocity reference)**

T1) 
ros2 run turtle_trajectory_generator twist2cmd

T2) 
ros2 run turtle_trajectory_generator teleop_twist_keyboard  --ros-args --remap /cmd_vel:=turtle_twist

**Spline (Smooth Position Commands)**

T1)
ros2 run turtle_trajectory_generator poly_trajectory

T2)
ros2 service call /set_desired_pose turtle_msgs/srv/SetDesiredPose "{ turtle_d:{x: 1, y: 0, theta: 0}, time: 1}"
