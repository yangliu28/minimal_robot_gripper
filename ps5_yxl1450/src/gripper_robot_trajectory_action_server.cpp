// trajectory action server for the gripper robot
// similar to example_trajectory_action_server, except there are six joints here
// the class "TrajectoryActionServer" here supports unlimited joints

// only position in the trajectory_msgs is used
	// because our controller only accept position command for control

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ps5_yxl1450/trajAction.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>

// interpolation parameters
const double dt = 0.005; // resolution for interpolating the trajectory
const double dt_min = 0.0000001; // for examining the time step

// there is one pos_cmd publisher for each joint in this class
class TrajectoryActionServer {
public:
	TrajectoryActionServer(ros::NodeHandle* nodehandle); // constructor
	~TrajectoryActionServer(void) {}
private:
	void send_joint_commands_(std::vector<double> cmd_jnts);
	void executeCB(const actionlib::SimpleActionServer<ps5_yxl1450::trajAction>::GoalConstPtr& goal);

	ros::NodeHandle nh_; // a nodehandle is needed
	// each joint in the robot needs a pos_cmd_publisher
	std::vector<ros::Publisher> pos_cmd_publisher_;  // initialization will be in executeCB()
	actionlib::SimpleActionServer<ps5_yxl1450::trajAction> as_;

	// message types for the action
	ps5_yxl1450::trajActionGoal goal_; // goal message
	ps5_yxl1450::trajActionResult result_; // result message
	ps5_yxl1450::trajActionFeedback feedback_; // feedback message, not used

};

TrajectoryActionServer::TrajectoryActionServer(ros::NodeHandle* nodehandle):
	nh_(*nodehandle), // dereference the pointer and pass the value
	as_(nh_, "gripper_robot_trajectory_action", boost::bind(&TrajectoryActionServer::executeCB, this, _1), false)
{
	ROS_INFO("in constructor of TrajectoryActionServer...");
	as_.start(); // start the "gripper_robot_trajectory_action_server"
}

// publish commands to all joint command topic
// this function will only be invoked in executeCB()
void TrajectoryActionServer::send_joint_commands_(std::vector<double> cmd_jnts) {
	int npublishers = pos_cmd_publisher_.size();
	if (cmd_jnts.size() == npublishers) { // dimension of pos_cmd and publisher is same
		for (int i=0; i<npublishers; i++) {
			std_msgs::Float64 cmd_msg;
			cmd_msg.data = cmd_jnts[i];
			pos_cmd_publisher_[i].publish(cmd_msg);
		}
	}
	else
		ROS_WARN("joint commanders and publishers are not consistent!");
}

// substantial work will be done here
void TrajectoryActionServer::executeCB(const actionlib::SimpleActionServer<ps5_yxl1450::trajAction>::GoalConstPtr& goal) {
	ROS_INFO("in executecb...");
	// avoid using "goal->trajectory" too much
	trajectory_msgs::JointTrajectory trajectory = goal -> trajectory;
	int njnts = trajectory.joint_names.size(); // get the number of joints
	int npts = trajectory.points.size(); // get the number of points

	ROS_INFO("trajectory message commands %d joint(s)", njnts);
	//initialize command publishers for each joints
	pos_cmd_publisher_.resize(njnts);
	for (int i=0; i<njnts; i++) {
		pos_cmd_publisher_[i] = nh_.advertise<std_msgs::Float64>(
			trajectory.joint_names[i] + "_pos_cmd", 1, true);
	}

	// joint array positions in the calculation of interpolation
	std::vector<double> prev_jnts; // previous joints in the interpolation
	std::vector<double> next_jnts; // next joints in the interpolation
	std::vector<double> cmd_jnts; // interpolated joints to be published
	prev_jnts.resize(njnts); // resize to the number of joints
	next_jnts.resize(njnts);
	cmd_jnts.resize(njnts);

	// time control parameters
		// final time to finish these points
	double t_final = trajectory.points[npts - 1].time_from_start.toSec();
		// the first point should be the current point in the gazebo
		// so the first time_from_start should be 0.0
	double t_previous = trajectory.points[0].time_from_start.toSec();
	double t_next = trajectory.points[1].time_from_start.toSec();
	double t_stream = t_previous; // initializee to t_previous
	double fraction_of_range = 0.0; // a ratio describing fraction of current segment completed

	// check the validality of time range
	double t_range = t_next - t_previous;
	if (t_range < dt_min) {
		ROS_WARN("time step invalid in trajectory! (begining)");
		// as_.setAborted(result_);
		as_.setAborted();
		return;
	}

	int ipt_next = 1; // index these points, start from the first one
	// initialize joint values
	prev_jnts = trajectory.points[ipt_next - 1].positions;
	next_jnts = trajectory.points[ipt_next].positions;

	// start interpolation, use time to control this process
	ros::Rate rate_timer(1/dt);
	while (t_stream < t_final) { // check if current time exceeds final time
		if (t_stream > t_next) { // time to change segment of points
			// an improvement here: supporting interpolation on combined segments
				// because it's possible the time resolution is higher than that in this interpolation
				// after jumping to next segment, if t_stream > t_next
				// only t_next and next_jnts will continue jumping, making this segment bigger
			ipt_next = ipt_next + 1; // index next point
			// refresh time stamp
			t_previous = t_next;
			t_next = trajectory.points[ipt_next].time_from_start.toSec();
			t_range = t_next - t_previous;
			if (t_range < dt_min) {
				ROS_WARN("time step invalid in trajectory! (middle)");
				// as_.setAborted(result_);
				as_.setAborted();
				return;
			}
			// refresh points stamp
			prev_jnts = next_jnts;
			next_jnts = trajectory.points[ipt_next].positions;
			// check if current segment still fails to cover t_stream
			while (t_stream > t_next) { // extend the end of this segment
				ipt_next = ipt_next + 1;
				t_next = trajectory.points[ipt_next].time_from_start.toSec();
				// no need to examine the range again
				next_jnts = trajectory.points[ipt_next].positions;
			}
		}

		// if here, have valid range t_previous < t_stream < t_next
		fraction_of_range = (t_stream - t_previous)/(t_next - t_previous);
		// linearly interpolating points for each joints
		for (int i=0; i<njnts; i++) {
			cmd_jnts[i] = prev_jnts[i] + fraction_of_range*(next_jnts[i] - prev_jnts[i]);
		}
		send_joint_commands_(cmd_jnts); // send these joint position to controller
		// for debug of these calculation, may comment out later
		// ROS_INFO("prev_jnts[2], next_jnts[2], cmd_jnts[2]: %f, %f, %f",
		// 	prev_jnts[2], next_jnts[2], cmd_jnts[2]);

		// time control
		rate_timer.sleep();
		t_stream = t_stream + dt;
	}

	// output the final point
	cmd_jnts = trajectory.points[npts - 1].positions;
	send_joint_commands_(cmd_jnts);
	// as_.setSucceeded(result_); // finally successful on this action...
	as_.setSucceeded();
}

//the main program instantiates a TrajectoryActionServer, then lets the callbacks do all the work
int main(int argc, char** argv) {
	ros::init(argc, argv, "gripper_robot_trajectory_action_server_node");
	ros::NodeHandle nh;	// to be passed in the instantiation of class

	ROS_INFO("instantiating an object of class TrajectoryActionServer...");
	TrajectoryActionServer as_object(&nh);

	ROS_INFO("going into spin...");

	while (ros::ok()) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return 0;
}

