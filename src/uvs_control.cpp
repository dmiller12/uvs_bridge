/*
 * lpetrich 27/06/18
 */
#include <fstream>
#include "uvs_bridge/uvs_control.h"
using namespace std;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

UVSControl::UVSControl(ros::NodeHandle nh_)
{
	dof = 7;
	total_joints = 0;
	image_tol = 100.0;
	default_lambda = 0.15;
	reset = false;
	move_now = false;
	arm = new ArmControl(nh_);
	// Get DOF of arm
	do {
		ROS_INFO_STREAM("Waiting to find robot DOF");
		ros::Duration(1.0).sleep();
		dof = arm->get_dof();
	} while (dof == 0);
	if (dof == 7) {
		total_joints = 7;
		bhand = new BHandControl("/zeus", nh_);
		bhand->set_spread_velocity(25);
		bhand->set_grasp_velocity(60);
	}
	else if (dof == 4) {
		total_joints = 4;
	} else {
		ROS_WARN_STREAM("Invalid DOF, reset and try again");
		exit(EXIT_FAILURE);
	}
	ROS_INFO_STREAM("Robot has " << dof << " DOF");
	error_sub = nh_.subscribe("/user_interface/image_error", 1, &UVSControl::error_cb, this);
	eef_sub = nh_.subscribe("/user_interface/end_effector", 1, &UVSControl::eef_cb, this);
}

UVSControl::~UVSControl()
{ // shutdown ROS subscribers properly
	error_sub.shutdown();
	eef_sub.shutdown();
}

Eigen::VectorXd UVSControl::calculate_delta_q()
{ // calculates the actual motion change in joint space to use in Broyden's update
	Eigen::VectorXd total_dq;
	Eigen::VectorXd dq(dof);
	Eigen::VectorXd current_joint_positions = arm->get_positions();
	total_dq = current_joint_positions - previous_joint_positions;
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			dq[j] = total_dq[i];
			j++;
		}
	}
	return dq;
}

Eigen::VectorXd UVSControl::calculate_target(const Eigen::VectorXd &current_state, const Eigen::VectorXd &delta)
{ // calculates target vector in joint space to move to with given delta added to active joints
	Eigen::VectorXd target_state(total_joints);
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			target_state[i] = (current_state[i] + delta[j]);
			j++;
		} else {
			target_state[i] = current_state[i];
		}
	}
	return target_state;
}

Eigen::VectorXd UVSControl::calculate_step(const Eigen::VectorXd &current_error_value)
{ // calculates new motion step to take with the control law: step = −λJ+e
	Eigen::VectorXd step;
	Eigen::VectorXd new_joints;
	step = -(jacobian_inverse * current_error_value).transpose();
	new_joints = arm->get_positions() + step;
	step *= 0.1;
	return step;
}

bool UVSControl::convergence_check(const Eigen::VectorXd &current_error)
{ // should we make lambda larger as we get closer to the target? Test
	double pixel_step_size = 30.0;
	double n = current_error.norm();
	Eigen::Vector3d vertical(0, 0, 1);
	// if (n < 300 && !(ready_to_grasp)) {
	// 	bhand->open_grasp();
	// 	bhand->open_spread();
	// 	ready_to_grasp = true;
	// 	std::cout << "ready to grasp object" << std::endl;
	// }
	if (n < image_tol) {
		std::cout << "current error norm is less than image tolerance -- we have arrived at our destination" << std::endl;
		// // for sphere movements
		// object_position = temp_object_position;
		// Eigen::Vector3d tool_position = getToolPosition(arm->get_positions(), total_joints);
		// std::cout << "Object position:" << object_position[0] << ", " << object_position[1] << ", " << object_position[2] << std::endl;
		// spherical_position = cartesian_to_spherical(tool_position - object_position);
		return true;
	}
	lambda = std::max(0.1, pixel_step_size / n);
	if ((1.0 - lambda) * n < image_tol) {
		lambda = 1.0 - image_tol / n;
		std::cout << "Next move places EEF too close to target - adjusting lambda to " << lambda << std::endl;
	}
	std::cout << "current error norm: " << n << std::endl;
	std::cout << "lambda: " << lambda << std::endl;
	return false;
}

bool UVSControl::broyden_update(double alpha)
{ // update jacobian
	Eigen::MatrixXd update(jacobian.rows(), jacobian.cols());
	Eigen::VectorXd current_eef_position;
	Eigen::VectorXd dy;
	Eigen::VectorXd dq;
	double dq_norm;

	dq = calculate_delta_q();
	dq_norm = dq.norm();
	if (dq_norm < 1e-3) {
		cout << "Small dq - no update" << endl;
		std::cout << "dq: \n" << dq.format(CleanFmt) << std::endl;
		return true;
	}
	current_eef_position = get_eef_position();
	dy = current_eef_position - previous_eef_position;
	if (dy.norm() < 5) {
		cout << "Small dy - no update" << endl;
		std::cout << "dy: \n" << dy.format(CleanFmt) << std::endl;
		return true;
	}
	update = ((dy - jacobian * dq) * dq.transpose()) / (dq_norm * dq_norm);
	previous_jacobian = jacobian;
	jacobian = jacobian + (alpha * update);
	if (!pseudoInverse(jacobian, jacobian_inverse)){
		return false;
	}
	std::cout << "current_eef_position: \n" << current_eef_position.format(CleanFmt) << std::endl;
	std::cout << "dq: \n" << dq.format(CleanFmt) << std::endl;
	// std::cout << "dq_norm: \n" << dq_norm << std::endl;
	std::cout << "dy: \n" << dy.format(CleanFmt) << std::endl;
	std::cout << "update: \n" << update.format(CleanFmt) << std::endl;
	std::cout << "jacobian: \n" << jacobian.format(CleanFmt) << std::endl;
	std::cout << "jacobian_inverse: \n" << jacobian_inverse.format(CleanFmt) << std::endl;
	// log(filename, "previous eef position: ", previous_eef_position, false);
	// log(filename, "current eef position: ", current_eef_position, false);
	// log(filename, "dq: ", dq, false);
	// log(filename, "dq norm: ", dq_norm, false);
	// log(filename, "dy: ", dy, false);
	// log(filename, "dy norm: ", dy.norm(), false);
	// log(filename, "broyden update: ", update, false);
	// log(filename, "new jacobian: ", jacobian, false);
	// log(filename, "inverse jacobian: ", jacobian_inverse, false);
	return true;
}

int UVSControl::move_step(bool continous_motion)
{ // step through one loop of VS
	Eigen::VectorXd current_error;
	Eigen::VectorXd current_joint_positions;
	Eigen::VectorXd step_delta;
	Eigen::VectorXd target_position;
	Eigen::VectorXd predicted_times;
	Eigen::VectorXd current_velocity;
	double sleep_time = 1.0;
	// grab and use current error, check for convergence
	current_error = get_error();
	if (convergence_check(current_error)) {
		return 0;
	}
	step_delta = calculate_step(current_error);
	// grab and use current joint positions, check if valid
	current_joint_positions = arm->get_positions();
	target_position = calculate_target(current_joint_positions, step_delta);
	if (!limit_check(target_position, total_joints)) {
		return 1;
	}
	// calculate move run time
	// current_velocity = arm->get_velocities();
	// write to screen for debugging
	std::cout << "current_error: \n" << current_error.format(CleanFmt) << std::endl;
	std::cout << "previous_joint_positions: \n" << previous_joint_positions.format(CleanFmt) << std::endl;
	std::cout << "current_joint_positions: \n" << current_joint_positions.format(CleanFmt) << std::endl;
	std::cout << "previous_eef_position: \n" << previous_eef_position.format(CleanFmt) << std::endl;
	std::cout << "step_delta: \n" << step_delta.format(CleanFmt) << std::endl;
	std::cout << "target_position: \n" << target_position.format(CleanFmt) << std::endl;
	// std::cout << "Predicted ramp-down time: " << predicted_times[1] << std::endl;
	// std::cout << "Predicted end time: " << predicted_times[2] << std::endl;
	// save previous state before move
	previous_joint_positions = current_joint_positions;
	previous_eef_position = get_eef_position();
	arm->call_move_joints(target_position, false);
	// check for continuous motion and adjust sleep times accordingly
	// if (continous_motion) {
	// 	// sleep_time = std::max(0.2, predicted_times[1] - 0.05);
	// 	// sleep_time = std::max(0.3, predicted_times[1] - 0.01);
	// 	sleep_time = std::min(1.0, std::max(0.3, (predicted_times[1] + predicted_times[2]) * 0.5)); // range between [0.3, 1.0]
	// } else {
	// 	sleep_time = 1.0;
	// }
	std::cout << "// sleep time: " << sleep_time << std::endl;
	ros::Duration(sleep_time).sleep();
	return 2;
}


void UVSControl::converge(double alpha, int max_iterations, bool continous_motion)
{
	int c;
	std::cout << "\n**************************************" << std::endl;
	for (int i = 0; i < max_iterations; ++i) {
		std::cout << "iteration: " << i << std::endl;
		ros::Time begin = ros::Time::now();
		c = move_step(continous_motion);
		switch (c) {
			case 0: // convergence - return early
				return;
			case 1: // joints out of limit, reset jacobian
				jacobian = previous_jacobian;
				std::cout << "target not within joint limits, resetting jacobian to: \n" << jacobian.format(CleanFmt) << std::endl;
				break;
			case 2: // step completed successfully
				std::cout << "BROYDEN UPDATE:" << std::endl;
				if (!broyden_update(alpha)) { // condition number failed, reset to previous jacobian
					jacobian = previous_jacobian;
					std::cout << "condition number failed, resetting jacobian to: \n" << jacobian.format(CleanFmt) << std::endl;
				}
				break;
		}
		std::cout << "loop duration: " << ros::Time::now() - begin << "\n**************************************" << std::endl;
	}
	return;
}


bool UVSControl::jacobian_estimate(double perturbation_delta)
{ // perturb each active joint for the initial jacobian estimation
	Eigen::VectorXd e1;
	Eigen::VectorXd e2;
	Eigen::VectorXd target;
	Eigen::VectorXd position;
	jacobian.resize(get_error().size(), dof);
	initial_jacobian.resize(get_error().size(), dof);
	jacobian_inverse.resize(dof, get_error().size());
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			ros::Duration(0.2).sleep();
			e1 = get_eef_position();
			position = arm->get_positions();
			target = vector_target(position, i, perturbation_delta);
			arm->call_move_joints(target, true);
			ros::Duration(0.2).sleep();
			e2 = get_eef_position();
			ros::Duration(0.2).sleep();
			arm->call_move_joints(position, true);
			jacobian.col(j) = (e2 - e1) / perturbation_delta;
			j++;
		}
	}
	initial_jacobian = jacobian;
	previous_joint_positions = arm->get_positions();
	previous_eef_position = get_eef_position();
	if (!pseudoInverse(jacobian, jacobian_inverse)) {
		std::cout << "Initial jacobian estimate failed -- condition number too large" << std::endl;
		return false;
	}
	std::cout << "initial jacobian: \n" << initial_jacobian.format(CleanFmt) << std::endl;
	std::cout << "inverse jacobian: \n" << jacobian_inverse.format(CleanFmt) << std::endl;

	// log(filename, "initial jacobian: ", initial_jacobian, false);
	// log(filename, "inverse jacobian: ", jacobian_inverse, false);
	return true;
}

void UVSControl::loop()
{ // main loop for user interaction
	bool jacobian_initialized = false;
	bool exit_loop = false;
	bool continous_motion = true;
	double perturbation_delta = 0.0875;
	double alpha = 1.0; // update rate
	int max_iterations = 25;
	double d;
	int c;
	Eigen::VectorXd pose;
	std::string line;
	std::string s;
	lambda = default_lambda; // convergence rate
	while (ros::ok() && !exit_loop) {
		// if (move_now && ready() && jacobian_initialized)
		// {
		// 	converge(alpha, 100, continous_motion);
		// 	lambda = default_lambda;
		// 	move_now = false;
		// }
		std::cout << "************************************************************************************************"
				  << "\nSelect option:"
				  << "\n\tp: Lock joint position"
				  << "\n\tu: Unlock joint position"
				  // << "\n\td: Set Jacobian delta movement (current = " << perturbation_delta << ")"
				  // << "\n\tl: Set step convergence lambda value (current = " << lambda << ")"
				  // << "\n\ta: Set alpha value for broyden update (current = " << alpha << ")"
				  // << "\n\tt: Set image_tol - prevents collision in image space (current = " << image_tol << ")"
				  // << "\n\tc: Set max iteration for convergence (current = " << max_iterations << ")"
				  << "\n\tj: Compute Jacobian"
				  // << "\n\tx: Compute Jacobian with chosen joints"
				  << "\n\tv: Complete VS convergence with set max iterations"
				  << "\n\ts: Compute and move one step"\
				  << "\n\ti: Move to initial position"
				  << "\n\th: Move to home position"
				  << "\n\to: Open grasp"
				  << "\n\tg: Close grasp"
				  << "\n\tz: Close spread"
				  << "\n\tq: quit"
				  << "\n\t>> " << std::endl;
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'p':
			arm->lock_joint_position(true);
			break;
		case 'u':
			arm->lock_joint_position(false);
			break;
		case 'j':
			if (ready()) {
				for (int i = 0; i < dof; ++i) {
					active_joints[i] = 1;
				}
				jacobian_initialized = jacobian_estimate(perturbation_delta);
			}
			break;
		// case 'x':
		// 	if (ready())
		// 	{
		// 		set_active_joints();
		// 		jacobian_initialized = jacobian_estimate(perturbation_delta);
		// 	}
		// 	break;
		case 'i':
			// arm->stop_visual_fix();
			arm->move_to_initial_position();
			bhand->open_grasp();
			bhand->close_spread();
			// grip_closed = false;
			// is_spread = false;
			break;
		// case 'd':
		// 	perturbation_delta = degreesToRadians(double_input(1, 20));
		// 	break;
		// case 'l':
		// 	lambda = double_input(0, 1);
		// 	break;
		// case 'a':
		// 	alpha = double_input(0, 1);
		// 	break;
		// case 'c':
		// 	max_iterations = double_input(0, 500);
		// 	break;
		// case 't':
		// 	image_tol = double_input(0, 500);
		// 	lambda = default_lambda;
		// 	break;
		case 'v':
			if (ready() && jacobian_initialized) {
				converge(alpha, max_iterations - 1, continous_motion);
				lambda = default_lambda;
			} else {
				ROS_WARN_STREAM("Jacobian is not initialized");
			}
			break;
		case 's':
			if (ready() && jacobian_initialized) {
				converge(alpha, 1, false);
				lambda = default_lambda;
			} else {
				ROS_WARN_STREAM("Jacobian is not initialized");
			}
			break;
		case 'h':
			// arm->stop_visual_fix();
			arm->move_to_home_position();
			break;
		case 'o':
			bhand->open_grasp();
			bhand->open_spread();
			break;
		case 'g':
			bhand->close_grasp();
			break;
		case 'z':
			bhand->close_spread();
			break;
		case 'q':
			exit_loop = true;
			break;
		default:
			ROS_WARN_STREAM("Unknown option");
		}
	}
	// ros::Rate r(60); 
	// while (ros::ok()) {
	// 	Eigen::VectorXd e;
	// 	e = get_error();
	// 	std::cout << "Error: ";
	// 	for (int i = 0; i < e.size(); ++i) {
	// 		std::cout << e[i] << " ";
	// 	}
	// 	e = get_eef_position();
	// 	std::cout << "End Effector: ";
	// 	for (int i = 0; i < e.size(); ++i) {
	// 		std::cout << e[i] << " ";
	// 	}
	// 	std::cout << std::endl;
	// 	ros::spinOnce();
	// 	r.sleep();
	// }
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "UVSControl");
	ros::NodeHandle nh_("~");
	ros::AsyncSpinner spinner(0);
	spinner.start();
	UVSControl uvs(nh_);
	uvs.loop();
	return 0;
}
