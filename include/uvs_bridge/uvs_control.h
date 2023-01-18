/*
 * lpetrich 01/07/18
 */

#ifndef UVS_CONTROL_H
#define UVS_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "wam_control/misc_utilities.h"
#include "wam_control/arm_control.h"
#include "wam_control/bhand_control.h"
#include "uvs_bridge/PointVector2D.h"
#include "uvs_bridge/utilities.h"


class UVSControl
{
  public:
	ArmControl *arm;
	BHandControl *bhand;
	bool reset;
	bool move_now;
	bool is_spread = false;
	bool grip_closed = false;
	bool teleop_move;
	int dof;
	int total_joints;
	double image_tol;
	double default_lambda;
	double lambda;
	double control_radius;
	double yaw_offset = 0.0;
	std::string robot_namespace;
	std::string msg;
	std::string prefix;
	std::string filename;
	Eigen::Vector3d temp_object_position{0.5, 0.0, 0.0};
	Eigen::Vector3d object_position{0.5, 0.0, 0.0};
	Eigen::VectorXd previous_eef_position;
	Eigen::VectorXd previous_joint_positions;
	Eigen::MatrixXd previous_jacobian;
	Eigen::MatrixXd initial_jacobian;
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd jacobian_inverse;
	std::vector<int> active_joints = {1, 1, 1, 1, 1, 1, 1};
	UVSControl(ros::NodeHandle nh);
	~UVSControl();
	Eigen::VectorXd calculate_delta_q();
	Eigen::VectorXd calculate_target(const Eigen::VectorXd &pos, const Eigen::VectorXd &delta);
	Eigen::VectorXd calculate_step(const Eigen::VectorXd &current_error_value);
	Eigen::Matrix<double, 7, 1> goal_joint_angles;
	bool convergence_check(const Eigen::VectorXd &current_error);
	void converge(double alpha, int max_iterations, bool continous_motion);
	int move_step(bool continous_motion);
	bool broyden_update(double alpha);
	bool jacobian_estimate(double perturbation_delta);
	void loop();

  private:
	// Callbacks
	ros::Subscriber error_sub;
	ros::Subscriber eef_sub;
	Eigen::VectorXd image_error_vector;
	Eigen::VectorXd image_eef_pos;
	bool new_error;
	bool new_eef;

	bool ready()
	{
		if (get_error().size() == 0 || get_eef_position().size() == 0) {
			std::cout << "please initialize trackers" << std::endl;
			return false;
		} else {
			return true;
		}
	}

	Eigen::VectorXd get_error()
	{
		while (!new_error) {
			continue;
		}
		new_error = false;
		return image_error_vector;
	}

	Eigen::VectorXd get_eef_position()
	{
		while (!new_eef) {
			continue;
		} // TODO fix so doesn't get stuck in iteration 6
		new_eef = false;
		return image_eef_pos;
	}

	void error_cb(uvs_bridge::PointVector2D::ConstPtr error)
	{
		uvs_bridge::PointVector2D current_error = *error;
		int sz = current_error.points.size();
		Eigen::VectorXd e(sz * 2);
		int idx = 0;
		for (int i = 0; i < sz; ++i) {
			e[idx++] = current_error.points[i].x;
			e[idx++] = current_error.points[i].y;
		}
		// for (int i = 0; i < e.size(); ++i) {
		// 	std::cout << e[i] << " ";
		// }
		// std::cout << std::endl;
		image_error_vector = e;
		new_error = true;
	}

	void eef_cb(uvs_bridge::PointVector2D::ConstPtr eef)
	{
		uvs_bridge::PointVector2D current_eef = *eef;
		int sz = current_eef.points.size();
		Eigen::VectorXd eef_pos(sz * 2);
		int j = 0;
		for (int i = 0; i < sz; ++i) {
			eef_pos[j++] = current_eef.points[i].x;
			eef_pos[j++] = current_eef.points[i].y;
		}
		image_eef_pos = eef_pos;
		new_eef = true;
	}

};

#endif // UVS_CONTROL_H
