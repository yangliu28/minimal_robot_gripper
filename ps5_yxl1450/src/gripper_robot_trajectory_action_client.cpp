// trajectory action client for the gripper robot
// this node demonstrate how to pick up a beer and place it with a robot gripper
// this movement has been decomposed below: (in gazebo there is a table with a beer)
	// 1.move the gripper to the safe point
		// (get current joint positions of the gripper robot from gazebo)
	// 2.move the gripper to the top area of the beer
		// (get the position of the beer from gazebo)
	// 3.move the gripper around the beer
	// 4.clamp the gripper to grasp the beer
	// 5.move the gripper up with the beer
	// 6.move the gripper to the above of target area (center of table)
		// (get the position of the table from gazebo)
	// 7.move the gripper down to place the beer on table
	// 8.unclamp the gripper and release the beer
	// 9.move the gripper up from table
	// 10.move the gripper back to the safe point

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ps5_yxl1450/trajAction.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>

// there are repeated code in calculating the goal message, may use a function instead

// inverse kinematics function of the gripper robot
std::vector<double> inverseKinematics(geometry_msgs::Point gripper_pos, double gripper_dist) {
	// input the disired gripper position and gripper paddle distance, return the joint positions

	// since this gripper robot is with simple design, the inverse kinematics could be done in one function
	// to even simplify the degree of freedom of this robot
		// 1.the gripper is set to be horizontal (means joint4 is decided by joint2 and joint3)
		// 2.the two gripper paddle moves inward or outward simulatiously
	// so now it's an 4 = (6-2) degree of freedom robot

	// the joint position limit for this gripper robot:
		// joint1(revolute):	-pi/2	pi/2
		// joint2(revolute):	0		pi/2
		// joint3(revolute):	0		pi
		// joint4(revolute):	-pi/2	pi/2
		// joint5(prismatic):	-0.16	0
		// joint6(prismatic):	0		0.16

	// the center of the gripper is defined at the center of the two paddles
	// the height of the table is around 1.00m, beer is around 0.23m
	// some other necessary dimensions
	double link1_height = 1.1; // distance from top of link1 to ground
	double link2_3_length = 1.0; // length of link2 & 3
	double gripper_center = 0.15; // distance from gripper center to link4
	double gripper_radius = 0.04; // the radius of the gripper paddle
	double gripper_range = 0.16; // distance from the gripper paddle axis to gripper center

	// convert input position to cylindric coordinates (cyl_r, cyl_theta, cyl_z)
	double cyl_r = sqrt(pow(gripper_pos.x, 2) + pow(gripper_pos.y, 2));
	double cyl_theta = atan(gripper_pos.y/gripper_pos.x);
	double cyl_z = gripper_pos.z;

	// middle variables, draw a graph may help to understand these angles
	double theta1 = atan((cyl_z - link1_height)/(cyl_r - gripper_center));
	double theta2 = acos(sqrt(pow(cyl_z - link1_height, 2) + pow(cyl_r - gripper_center, 2))/2);
	double half_dist = gripper_dist/2 + gripper_radius;

	// output joints vector
	std::vector<double> cmd_jnts;
	cmd_jnts.resize(6); // it's a 6 joints robot

	// start inverse kinematics
	// revolute joints: joint1 & 2 & 3 & 4
	cmd_jnts[0] = cyl_theta; // joint1, the easiest one...
	cmd_jnts[1] = M_PI/2 - (theta1 + theta2); // joint2
	cmd_jnts[2] = 2*theta2; // joint3
	cmd_jnts[3] = M_PI/2 - cmd_jnts[1] - cmd_jnts[2]; // joint4, theta1 - theta2
	// prismatic joints: joint5 & 6
	cmd_jnts[4] = -(gripper_range - half_dist); // joint5
	cmd_jnts[5] = (gripper_range - half_dist); // joint6

	return cmd_jnts;
}

// callback to get "result" message from action server
void doneCb(const actionlib::SimpleClientGoalState& state,
		const ps5_yxl1450::trajResultConstPtr& result) {
	ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "gripper_robot_trajectory_action_client_node");
	ros::NodeHandle nh;

	// initialize an action client
	actionlib::SimpleActionClient<ps5_yxl1450::trajAction> action_client(
		"gripper_robot_trajectory_action", true);
	// try to connect the client to action server
	bool server_exist = action_client.waitForServer(ros::Duration(5.0));
	ros::Duration sleep1s(1);
	if(!server_exist) {
		ROS_WARN("could not connect to server; retrying");
		bool server_exist = action_client.waitForServer(ros::Duration(1.0));
		sleep1s.sleep();
	}
	// if here, then connected to the server
	ROS_INFO("connected to action server");

	ps5_yxl1450::trajGoal goal;
	// instantiate goal message
	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectory_points;
	// joint_names field
	trajectory.joint_names.resize(6);
	trajectory.joint_names[0] = "joint1";
	trajectory.joint_names[1] = "joint2";
	trajectory.joint_names[2] = "joint3";
	trajectory.joint_names[3] = "joint4";
	trajectory.joint_names[4] = "joint5";
	trajectory.joint_names[5] = "joint6";
	// positions and velocities field
	trajectory_points.positions.resize(6);

	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;

	// parameters for flow control, time assignment
	double dt_sample = 1.0; // really coarse, let action server to interpolate
	int time_1 = 5; // time for task 1, move to safe, integer to be divided by dt_sample
	int time_2 = 5; // task 2, move to beer top
	int time_3 = 3; // task 3, move to around beer
	int time_4 = 2; // task 4, clamp the gripper
	int time_5 = 3; // task 5, move up the gripper
	int time_6 = 5; // task 6, move to the above of target
	int time_7 = 3; // task 7, place the beer
	int time_8 = 2; // task 8, unclamp the gripper
	int time_9 = 3; // task 9, move up the gripper
	int time_10 = 5; // task 10, move to safe
	double time_delay = 1.0; // delay between every task
	double lift_height = 0.3; // height to lift the beer up
	std::vector<double> start_jnts; // start joints for each move task
	std::vector<double> end_jnts; // end joints for each move task
	double fraction_of_range;
	bool finish_before_timeout;
	start_jnts.resize(6);
	end_jnts.resize(6);
	// double beer_height = 0.23; // the height of the beer
	double beer_height = 0.28; // 0.23 is real height, use 0.30 to lift the grasp point up a little
	double table_height = 1.0; // the height of the table
	double gripper_open = 0.24; // distance of two paddles when gripper is open
	double gripper_close = 0.08; // distance of two paddles when grasping the beer

	///////////////////////////////////////
	// 1.move the gripper to the safe point
	///////////////////////////////////////

	ROS_INFO("task 1: move the gripper to the safe point.");

	// get the original joint positions when this node is invoked
	std::vector<double> origin_jnts;
	origin_jnts.resize(6);
	for (int i=0; i<6; i++) {
		get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
		get_jnt_state_client.call(get_joint_state_srv_msg);
		origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
	}
	// assign current joints to start joints
	start_jnts = origin_jnts;

	// define the safe point, avoid singularity at origin
	std::vector<double> safe_jnts;
	safe_jnts.resize(6);
	safe_jnts[0] = 0; // joint1, at its origin
	safe_jnts[1] = 30.0/180.0*M_PI; // joint2, a little bit forward
	safe_jnts[2] = 75.0/180.0*M_PI; // joint3, a little bit forward
	safe_jnts[3] = M_PI/2 - safe_jnts[1] - safe_jnts[2]; // joint4, parallel to the ground
	safe_jnts[4] = 0; // joint5, at its origin
	safe_jnts[5] = 0; // joint6, at its origin
	// assign the safe joints to end joints
	end_jnts = safe_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_1+1; i++) { // there are time_1+1 points, including start and end
		fraction_of_range = (double)i/time_1; // cautious, convert to double
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_1 + 12.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 1 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 1 is done.");
	}
	// if here, task 1 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	/////////////////////////////////////////////////
	// 2.move the gripper to the top area of the beer
	/////////////////////////////////////////////////

	ROS_INFO("task 2: move the gripper to the top area of the beer.");

	// get the position of the beer from gazebo
	// the position from gazebo is the bottom center of the beer
	geometry_msgs::Point beer_pos; // {float64 x, float64 y, float z}
	get_model_state_srv_msg.request.model_name = "beer";
	get_model_state_srv_msg.request.relative_entity_name = "link";
	// "link" is the entity name when I add a beer in gazebo
	get_model_state_client.call(get_model_state_srv_msg);
	beer_pos = get_model_state_srv_msg.response.pose.position;

	// calculate robot joints when the gripper is above the beer
	geometry_msgs::Point hover_open_start_pos;
	hover_open_start_pos = beer_pos;
	hover_open_start_pos.z = hover_open_start_pos.z + beer_height/2 + lift_height;
	std::vector<double> hover_open_start_jnts; // the corresponding joint position
	hover_open_start_jnts.resize(6);
	hover_open_start_jnts = inverseKinematics(hover_open_start_pos, gripper_open); // calculate the robot joints

	// assign the start joints and end joints
	start_jnts = safe_jnts; // start with last joints
	end_jnts = hover_open_start_jnts;

	// //////////////////////////////////////////////////////////////////////////////////////////////
	// ROS_INFO("start_jnts: %f, %f, %f, %f, %f, %f", start_jnts[0], start_jnts[1], start_jnts[2],
	// 	start_jnts[3], start_jnts[4], start_jnts[5]);
	// ROS_INFO("end_jnts: %f, %f, %f, %f, %f, %f", end_jnts[0], end_jnts[1], end_jnts[2],
	// 	end_jnts[3], end_jnts[4], end_jnts[5]);
	// //////////////////////////////////////////////////////////////////////////////////////////////

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_2+1; i++) { // there are time_2+1 points, including start and end
		fraction_of_range = (double)i/time_2;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_2 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 2 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 2 is done.");
	}
	// if here, task 2 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	/////////////////////////////////////
	// 3.move the gripper around the beer
	/////////////////////////////////////

	ROS_INFO("task 3: move the gripper around the beer.");

	// calculate robot joints when around the beer
	geometry_msgs::Point around_open_start_pos;
	around_open_start_pos = beer_pos;
	around_open_start_pos.z = around_open_start_pos.z + beer_height/2;
	std::vector<double> around_open_start_jnts; // the corresponding joint position
	around_open_start_jnts.resize(6);
	around_open_start_jnts = inverseKinematics(around_open_start_pos, gripper_open);

	// assign the start joints and end joints
	start_jnts = hover_open_start_jnts;
	end_jnts = around_open_start_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_3+1; i++) { // there are time_3+1 points, including start and end
		fraction_of_range = (double)i/time_3;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_3 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 3 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 3 is done.");
	}
	// if here, task 3 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	////////////////////////////////////////
	// 4.clamp the gripper to grasp the beer
	////////////////////////////////////////

	ROS_INFO("task 4: clamp the gripper to grasp the beer.");

	// calculate robot joints when grasp the beer
	geometry_msgs::Point grasp_close_start_pos;
	grasp_close_start_pos = around_open_start_pos;
	std::vector<double> grasp_close_start_jnts; // the corresponding joint position
	grasp_close_start_jnts.resize(6);
	grasp_close_start_jnts = inverseKinematics(grasp_close_start_pos, gripper_close);

	// assign the start joints and end joints
	start_jnts = around_open_start_jnts;
	end_jnts = grasp_close_start_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_4+1; i++) { // there are time_4+1 points, including start and end
		fraction_of_range = (double)i/time_4;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_4 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 4 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 4 is done.");
	}
	// if here, task 4 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	//////////////////////////////////////
	// 5.move the gripper up with the beer
	//////////////////////////////////////

	ROS_INFO("task 5: move the gripper up with the beer.");

	// calculate the robot joint when grasp the beer and lift up
	geometry_msgs::Point hover_close_start_pos;
	hover_close_start_pos = grasp_close_start_pos;
	hover_close_start_pos.z = hover_close_start_pos.z + lift_height;
	std::vector<double> hover_close_start_jnts;
	hover_close_start_jnts.resize(6);
	hover_close_start_jnts = inverseKinematics(hover_close_start_pos, gripper_close);

	// assign the start joints and end joints
	start_jnts = grasp_close_start_jnts;
	end_jnts = hover_close_start_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_5+1; i++) { // there are time_5+1 points, including start and end
		fraction_of_range = (double)i/time_5;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_5 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 5 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 5 is done.");
	}
	// if here, task 5 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	/////////////////////////////////////////////////
	// 6.move the gripper to the above of target area
	/////////////////////////////////////////////////

	ROS_INFO("task 6: move the gripper to the above of target area.");

	// get the position of the table from gazebo
	geometry_msgs::Point table_pos;
	get_model_state_srv_msg.request.model_name = "table";
	get_model_state_srv_msg.request.relative_entity_name = "link";
	get_model_state_client.call(get_model_state_srv_msg);
	table_pos = get_model_state_srv_msg.response.pose.position;

	// calculate the target position of the beer
	geometry_msgs::Point target_pos;
	target_pos = table_pos;
	target_pos.z = target_pos.z + table_height;

	// calculate the robot joints at the above of target area
	geometry_msgs::Point hover_close_end_pos;
	hover_close_end_pos = target_pos;
	hover_close_end_pos.z = hover_close_end_pos.z + beer_height/2 + lift_height;
	std::vector<double> hover_close_end_jnts;
	hover_close_end_jnts.resize(6);
	hover_close_end_jnts = inverseKinematics(hover_close_end_pos, gripper_close);

	// assign the start joints and end joints
	start_jnts = hover_close_start_jnts;
	end_jnts = hover_close_end_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_6+1; i++) { // there are time_6+1 points, including start and end
		fraction_of_range = (double)i/time_6;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_6 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 6 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 6 is done.");
	}
	// if here, task 6 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	/////////////////////////////////////////////////////
	// 7.move the gripper down to place the beer on table
	/////////////////////////////////////////////////////

	ROS_INFO("task 7: move the gripper down to place the beer on table.");

	// calculate the robot joints when move the gripper down
	geometry_msgs::Point grasp_close_end_pos;
	grasp_close_end_pos = target_pos;
	grasp_close_end_pos.z = grasp_close_end_pos.z + beer_height/2;
	std::vector<double> grasp_close_end_jnts;
	grasp_close_end_jnts.resize(6);
	grasp_close_end_jnts = inverseKinematics(grasp_close_end_pos, gripper_close);

	// assign the start joints and end joints
	start_jnts = hover_close_end_jnts;
	end_jnts = grasp_close_end_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_7+1; i++) { // there are time_7+1 points, including start and end
		fraction_of_range = (double)i/time_7;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_7 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 7 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 7 is done.");
	}
	// if here, task 7 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	/////////////////////////////////////////////
	// 8.unclamp the gripper and release the beer
	/////////////////////////////////////////////

	ROS_INFO("task 8: unclamp the gripper and release the beer.");

	// calculate the robot joints after release the beer
	geometry_msgs::Point around_open_end_pos;
	around_open_end_pos = target_pos;
	around_open_end_pos.z = around_open_end_pos.z + beer_height/2;
	std::vector<double> around_open_end_jnts;
	around_open_end_jnts.resize(6);
	around_open_end_jnts = inverseKinematics(around_open_end_pos, gripper_open);

	// assign the start joints and end joints
	start_jnts = grasp_close_end_jnts;
	end_jnts = around_open_end_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_8+1; i++) { // there are time_8+1 points, including start and end
		fraction_of_range = (double)i/time_8;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_8 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 8 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 8 is done.");
	}
	// if here, task 8 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	///////////////////////////////////
	// 9.move the gripper up from table
	///////////////////////////////////

	ROS_INFO("task 9: move the gripper up from table.");

	// calculate the robot joints after lift up the gripper
	geometry_msgs::Point hover_open_end_pos;
	hover_open_end_pos = target_pos;
	hover_open_end_pos.z = hover_open_end_pos.z + beer_height/2 + lift_height;
	std::vector<double> hover_open_end_jnts;
	hover_open_end_jnts = inverseKinematics(hover_open_end_pos, gripper_open);

	// assign the start joints and end joints
	start_jnts = around_open_end_jnts;
	end_jnts = hover_open_end_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_9+1; i++) { // there are time_9+1 points, including start and end
		fraction_of_range = (double)i/time_9;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_9 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 9 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 9 is done.");
	}
	// if here, task 9 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	/////////////////////////////////////////////
	// 10.move the gripper back to the safe point
	/////////////////////////////////////////////

	ROS_INFO("task 10: move the gripper back to the safe point.");

	// assign the start joints and end joints
	start_jnts = hover_open_end_jnts;
	end_jnts = safe_jnts;

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_10+1; i++) { // there are time_10+1 points, including start and end
		fraction_of_range = (double)i/time_10;
		for (int j=0; j<6; j++) { // there are 6 joints
			trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &doneCb);
	// wait for expected duration plus some tolerance (2 seconds)
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_10 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 10 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 10 is done.");
	}
	// if here, task 10 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task


	ROS_INFO("move-the-beer task is finished!");

	return 0;
}

