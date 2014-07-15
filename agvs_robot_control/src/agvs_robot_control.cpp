/*
 * agvs_robot_control
 * Copyright (c) 2014, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik
 * \brief Controller for the AGVS robot Ackerman Drive
 * \brief (will include dual odometry measurement)
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robotnik_msgs/set_mode.h>
#include <robotnik_msgs/get_mode.h>
#include <robotnik_msgs/set_odometry.h>
#include <robotnik_msgs/ptz.h>

#include "ackermann_msgs/AckermannDriveStamped.h"

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <std_srvs/Empty.h>


#define PI 3.1415926535
#define AGVS_MIN_COMMAND_REC_FREQ     5.0
#define AGVS_MAX_COMMAND_REC_FREQ     150.0

#define AGVS_WHEEL_DIAMETER	          0.2195      // Default wheel diameter
#define DEFAULT_DIST_CENTER_TO_WHEEL  0.479       // Default distance center to motorwheel
    
#define MAX_ELEVATOR_POSITION	0.05		// meters

using namespace std;

class AGVSControllerClass {

public:

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  double desired_freq_;

  // Diagnostics
  diagnostic_updater::Updater diagnostic_;			// General status diagnostic updater
  diagnostic_updater::FrequencyStatus freq_diag_;		         // Component frequency diagnostics
  diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
  ros::Time last_command_time_;					// Last moment when the component received a command
  diagnostic_updater::FunctionDiagnosticTask command_freq_;

  // Robot model 
  std::string robot_model_;
  
  // Velocity and position references to low level controllers
  ros::Publisher ref_vel_fwd_;
  ros::Publisher ref_vel_bwd_;
  ros::Publisher ref_pos_fwd_;
  ros::Publisher ref_pos_bwd_;
  ros::Publisher ref_pos_elevator_;

  // Joint states published by the joint_state_controller of the Controller Manager
  ros::Subscriber joint_state_sub_;

  // High level robot command
  ros::Subscriber cmd_sub_;
	
  // Services
  ros::ServiceServer srv_SetOdometry_;
  ros::ServiceServer srv_SetMode_;
  ros::ServiceServer srv_GetMode_;
  ros::ServiceServer srv_RaiseElevator_;
  ros::ServiceServer srv_LowerElevator_;
  
  // Topics - Ackerman - velocity
  std::string fwd_vel_topic_;
  std::string bwd_vel_topic_;
  
  // Joint names - Ackerman - velocity 
  std::string joint_front_wheel;
  std::string joint_back_wheel;

  // Topics - Ackerman - position
  std::string fwd_pos_topic_;
  std::string bwd_pos_topic_;
  std::string elevator_pos_topic_;
  
  std::string imu_topic_;
    
  // Joint names - Ackerman - position
  std::string joint_front_motor_wheel;
  std::string joint_back_motor_wheel;
    
  // Indexes to joint_states
  int fwd_vel_, bwd_vel_;
  int fwd_pos_, bwd_pos_;

  // Robot Speeds
  double linearSpeedXMps_;
  double linearSpeedYMps_;
  double angularSpeedRads_;

  // Robot Positions
  double robot_pose_px_;
  double robot_pose_py_;
  double robot_pose_pa_;
  double robot_pose_vx_;
  double robot_pose_vy_;
  
  // Robot Joint States
  sensor_msgs::JointState joint_state_;
  
  // Command reference
  // geometry_msgs::Twist base_vel_msg_;
  ackermann_msgs::AckermannDriveStamped base_vel_msg_;

  // External references
  double v_ref_;
  double a_ref_;

  double v_mps_;  // Measured real robot speed traction wheel speed
  
  // Flag to indicate if joint_state has been read
  bool read_state_; 
  
  // Robot configuration parameters 
  double agvs_wheel_diameter_; 
  double agvs_dist_to_center_;

  // IMU values
  double ang_vel_x_;
  double ang_vel_y_;
  double ang_vel_z_;

  double lin_acc_x_;
  double lin_acc_y_;
  double lin_acc_z_;

  double orientation_diff_x_;
  double orientation_diff_y_;
  double orientation_diff_z_;
  double orientation_diff_w_;

  // Parameter that defines if odom tf is published or not
  bool publish_odom_tf_;

  ros::Subscriber imu_sub_; 
  
  // Publisher for odom topic
  ros::Publisher odom_pub_; 

  // Broadcaster for odom tf  
  tf::TransformBroadcaster odom_broadcaster;


/*!	\fn AGVSControllerClass::AGVSControllerClass()
 * 	\brief Public constructor
*/
AGVSControllerClass(ros::NodeHandle h) : diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  desired_freq_(100.0),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  command_freq_("Command frequency check", boost::bind(&AGVSControllerClass::check_command_subscriber, this, _1))
  {

  ROS_INFO("agvs_robot_control_node - Init ");
  
  ros::NodeHandle agvs_robot_control_node_handle(node_handle_, "agvs_robot_control");

  // Get robot model from the parameters
  if (!private_node_handle_.getParam("model", robot_model_)) {
	  ROS_ERROR("Robot model not defined.");
	  exit(-1);
	  }
  else ROS_INFO("Robot Model : %s", robot_model_.c_str());

  // Ackerman configuration - topics (control actions)
  private_node_handle_.param<std::string>("fwd_vel_topic", fwd_vel_topic_, "/agvs/joint_front_wheel_controller/command");
  private_node_handle_.param<std::string>("bwd_vel_topic", bwd_vel_topic_, "/agvs/joint_back_wheel_controller/command");
  private_node_handle_.param<std::string>("fwd_pos_topic", fwd_pos_topic_, "/agvs/joint_front_motor_wheel_controller/command");
  private_node_handle_.param<std::string>("bwd_pos_topic", bwd_pos_topic_, "/agvs/joint_back_motor_wheel_controller/command");
  private_node_handle_.param<std::string>("elevator_pos_topic", elevator_pos_topic_, "/agvs/joint_elevator_controller/command");
  private_node_handle_.param<std::string>("imu_topic", imu_topic_, "/agvs/imu_data");

  // Ackerman configuration - joint names 
  private_node_handle_.param<std::string>("joint_front_wheel", joint_front_wheel, "joint_front_wheel");
  private_node_handle_.param<std::string>("joint_back_wheel", joint_back_wheel, "joint_back_wheel");
  private_node_handle_.param<std::string>("joint_front_motor_wheel", joint_front_motor_wheel, "joint_front_motor_wheel");
  private_node_handle_.param<std::string>("joint_back_motor_wheel", joint_back_motor_wheel, "joint_back_motor_wheel");

  // Robot kinematic parameters 
  if (!private_node_handle_.getParam("agvs_wheel_diameter", agvs_wheel_diameter_))
		agvs_wheel_diameter_ = AGVS_WHEEL_DIAMETER; 
  if (!private_node_handle_.getParam("agvs_dist_to_center", agvs_dist_to_center_))
		agvs_dist_to_center_ = DEFAULT_DIST_CENTER_TO_WHEEL;
  //ROS_INFO("agvs_wheel_diameter_ = %5.2f", agvs_wheel_diameter_);
  //ROS_INFO("agvs_dist_to_center_ = %5.2f", agvs_dist_to_center_);

  private_node_handle_.param("publish_odom_tf", publish_odom_tf_, true);
  if (publish_odom_tf_) ROS_INFO("PUBLISHING odom->base_footprint tf");
  else ROS_INFO("NOT PUBLISHING odom->base_footprint tf");
  
  // Robot Speeds
  linearSpeedXMps_   = 0.0;
  linearSpeedYMps_   = 0.0;
  angularSpeedRads_  = 0.0;

  // Robot Positions
  robot_pose_px_ = 0.0;
  robot_pose_py_ = 0.0;
  robot_pose_pa_ = 0.0;
  robot_pose_vx_ = 0.0;
  robot_pose_vy_ = 0.0;

  // External speed references
  v_ref_ = 0.0;
  a_ref_ = 0.0;

  // Imu variables
  ang_vel_x_ = 0.0; ang_vel_y_ = 0.0; ang_vel_z_ = 0.0;
  lin_acc_x_ = 0.0; lin_acc_y_ = 0.0; lin_acc_z_ = 0.0;
  orientation_diff_x_ = 0.0; orientation_diff_y_ = 0.0; orientation_diff_z_ = 0.0; orientation_diff_w_ = 1.0;

  // Advertise services
  srv_SetOdometry_ = private_node_handle_.advertiseService("set_odometry",  &AGVSControllerClass::srvCallback_SetOdometry, this);
  srv_RaiseElevator_ = private_node_handle_.advertiseService("raise_elevator",  &AGVSControllerClass::srvCallback_RaiseElevator, this);
  srv_LowerElevator_ = private_node_handle_.advertiseService("lower_elevator",  &AGVSControllerClass::srvCallback_LowerElevator, this);

  // Subscribe to joint states topic
  joint_state_sub_ = agvs_robot_control_node_handle.subscribe<sensor_msgs::JointState>("/agvs/joint_states", 1, &AGVSControllerClass::jointStateCallback, this);
  //joint_state_sub_ = private_node_handle_.subscribe<sensor_msgs::JointState>("joint_states", 1, &AGVSControllerClass::jointStateCallback, this);
  //joint_state_sub_ = summit_xl_robot_control_node_handle.subscribe<sensor_msgs::JointState>("/summit_xl/joint_states", 1, &SummitXLControllerClass::jointStateCallback, this);


  // Subscribe to imu data
  // imu_sub_ = agvs_robot_control_node_handle.subscribe("/agvs/imu_data", 1, &AGVSControllerClass::imuCallback, this);
  imu_sub_ = private_node_handle_.subscribe(imu_topic_, 1, &AGVSControllerClass::imuCallback, this);

  // Adevertise reference topics for the controllers 
  ref_vel_fwd_ = private_node_handle_.advertise<std_msgs::Float64>( fwd_vel_topic_, 50);
  ref_vel_bwd_ = private_node_handle_.advertise<std_msgs::Float64>( bwd_vel_topic_, 50);
  ref_pos_fwd_ = private_node_handle_.advertise<std_msgs::Float64>( fwd_pos_topic_, 50);
  ref_pos_bwd_ = private_node_handle_.advertise<std_msgs::Float64>( bwd_pos_topic_, 50);	  
  ref_pos_elevator_ = private_node_handle_.advertise<std_msgs::Float64>( elevator_pos_topic_, 50);	  
  	  
  // Subscribe to command topic
  cmd_sub_ = private_node_handle_.subscribe<ackermann_msgs::AckermannDriveStamped>("command", 1, &AGVSControllerClass::commandCallback, this ); 
    
  // TODO odom topic as parameter
  // Publish odometry 
  odom_pub_ = private_node_handle_.advertise<nav_msgs::Odometry>("odom", 1000);

  // Component frequency diagnostics
  diagnostic_.setHardwareID("agvs_robot_control - simulation");
  diagnostic_.add( freq_diag_ );
  diagnostic_.add( command_freq_ );
    
  // Topics freq control 
  // For /agvs_robot_control/command
  double min_freq = AGVS_MIN_COMMAND_REC_FREQ; // If you update these values, the
  double max_freq = AGVS_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
  subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/agvs_robot_control/command", diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
  subs_command_freq->addTask(&command_freq_); // Adding an additional task to the control
  
  // Flag to indicate joint_state has been read
  read_state_ = false;
  
  // Robot ackermann measured speed.
  v_mps_ = 0;
}

/// Controller startup in realtime
int starting()
{

  ROS_INFO("AGVSControllerClass::starting");

// name: ['joint_back_motor_wheel', 'joint_back_wheel', 'joint_front_motor_wheel', 'joint_front_wheel']
// position: [6.283185307179586, -3.14159, 6.283185307179586, -3.14159]
// velocity: [nan, nan, nan, nan]

  // Initialize joint indexes according to joint names 
  if (read_state_) {
    vector<string> joint_names = joint_state_.name;
    fwd_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_wheel)) - joint_names.begin();
    bwd_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_wheel)) - joint_names.begin();
    fwd_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_motor_wheel)) - joint_names.begin();
    bwd_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_back_motor_wheel)) - joint_names.begin();
    return 0;
    }
  else return -1;
}

/*!	\fn void UpdateOdometry()
 *	\brief Updates the values of the odometry
 *  Ackerman's odometry calculation (using motor speed and position of the motor direction)
*/
void UpdateOdometry(){

    // TODO UpdateOdometry differential drive
	
   // double v_left_mps, v_right_mps;
   // Calculate its own velocities for realize the motor control 
   // v_left_mps = ((joint_state_.velocity[blw_vel_] + joint_state_.velocity[flw_vel_]) / 2.0) * (summit_xl_wheel_diameter_ / 2.0);
   // v_right_mps = -((joint_state_.velocity[brw_vel_] + joint_state_.velocity[frw_vel_]) / 2.0) * (summit_xl_wheel_diameter_ / 2.0); 
   // sign according to urdf (if wheel model is not symetric, should be inverted)
   // angularSpeedRads_ = (v_right_mps - v_left_mps) / summit_xl_d_tracks_m_;    // rad/s


	double fBetaRads = 0.0;
	double v_mps = 0.0;

	// Compute Position
	double fSamplePeriod = 1.0 / desired_freq_;

	// Linear speed of each wheel
	double v_fwd, v_bwd; 
	v_fwd = joint_state_.velocity[fwd_vel_] * (agvs_wheel_diameter_ / 2.0);
	v_bwd = joint_state_.velocity[bwd_vel_] * (agvs_wheel_diameter_ / 2.0);
	//ROS_INFO("v_fwd = %.3lf, v_bwd = %.3lf", v_fwd, v_bwd);
	v_mps = -(v_fwd + v_bwd) / 2.0;

	// Angle of fwd and bwd motorwheels
	double a_fwd, a_bwd;
	a_fwd = radnorm2( joint_state_.position[fwd_pos_] );
	a_bwd = radnorm2( joint_state_.position[bwd_pos_] );
	fBetaRads = a_fwd; // consider to get a mean, but both angles are antisimetric
	
	// Filter noise
	if(fabs(v_mps) < 0.00001) v_mps = 0.0;

	// Compute Odometry
	double w = (v_mps / agvs_dist_to_center_) * sin(fBetaRads);
	robot_pose_pa_ += w * fSamplePeriod;

	// normalize
	radnorm(&robot_pose_pa_);
	//ROS_INFO("Orientation = %.3lf", robot_pose_pa_);
	// Velocities
	robot_pose_vx_ = v_mps * cos(fBetaRads) * cos(robot_pose_pa_);
	robot_pose_vy_ = v_mps * cos(fBetaRads) * sin(robot_pose_pa_);
	
	// Positions
	robot_pose_px_ += robot_pose_vx_ * fSamplePeriod;
	robot_pose_py_ += robot_pose_vy_ * fSamplePeriod;
}

// Publish robot odometry tf and topic depending 
void PublishOdometry()
{
	ros::Time current_time = ros::Time::now();
	
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = robot_pose_px_;
    odom_trans.transform.translation.y = robot_pose_py_;
    odom_trans.transform.translation.z = 0.0;

	// Convert theta from yaw (rads) to quaternion. note that this is only 2D !!!
    double theta = robot_pose_pa_;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw( theta );
    odom_trans.transform.rotation.x = quat.x;
	odom_trans.transform.rotation.y = quat.y;
	odom_trans.transform.rotation.z = quat.z;
	odom_trans.transform.rotation.w = quat.w;

	
    // send the transform over /tf
	// activate / deactivate with param
	// this tf in needed when not using robot_pose_ekf
    if (publish_odom_tf_) odom_broadcaster.sendTransform(odom_trans);  
        
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
	// Position
    odom.pose.pose.position.x = robot_pose_px_;
    odom.pose.pose.position.y = robot_pose_py_;
    odom.pose.pose.position.z = 0.0;
	// Orientation
    odom.pose.pose.orientation.x = orientation_diff_x_;
	odom.pose.pose.orientation.y = orientation_diff_y_;
	odom.pose.pose.orientation.z = orientation_diff_z_;
	odom.pose.pose.orientation.w = orientation_diff_w_;
	
    // Pose covariance
    for(int i = 0; i < 6; i++)
      		odom.pose.covariance[i*6+i] = 0.1;  // test 0.001

    //set the velocity
    odom.child_frame_id = "base_footprint";
	// Linear velocities
    odom.twist.twist.linear.x = robot_pose_vx_;
    odom.twist.twist.linear.y = robot_pose_vy_;
	odom.twist.twist.linear.z = 0.0;
	// Angular velocities
	odom.twist.twist.angular.x = ang_vel_x_;
	odom.twist.twist.angular.y = ang_vel_y_;
    odom.twist.twist.angular.z = ang_vel_z_;
	// Twist covariance
	for(int i = 0; i < 6; i++)
     		odom.twist.covariance[6*i+i] = 0.1;  // test 0.001

    //publish the message
    odom_pub_.publish(odom);
}

void UpdateControl()
{
  // Ackerman reference messages
  std_msgs::Float64 vel_ref_msg; 
  std_msgs::Float64 pos_ref_msg;
  static double ev_ant = 0.0;

  // Note that the controllers are not in speed mode, but in effort. Therefore the speed ref will be Nm.
  // Open loop - works well but there is some inertia in the whole system and just setting torque to 0 does not stop the robot.
  // vel_ref_msg.data = -v_ref_ * 20.0;
  
  // If using a JointEffortController Try a velocity control loop
  // After 1.9.2 VelocityControllers in agvs_control should work 
  /*
  double ev = v_mps_ - v_ref_;
  double Kp = 30.0;
  double Kd = 30.0;
  vel_ref_msg.data = Kp * ev;  //+ Kd * (ev - ev_ant);
  ev_ant = ev;
  */

  // Reference for velocity controllers
  double Kp = 10.0;  // ref is in [m/s] while VelocityController expects ?
  vel_ref_msg.data = -v_ref_ * Kp;

  pos_ref_msg.data = a_ref_;
	   	  
  // Publish references 
  ref_vel_fwd_.publish( vel_ref_msg );
  ref_vel_bwd_.publish( vel_ref_msg );
  
  ref_pos_fwd_.publish( pos_ref_msg );
  pos_ref_msg.data = -a_ref_; // symetric angle
  ref_pos_bwd_.publish( pos_ref_msg );
}

// Sets the motor position to the desired value
void SetElevatorPosition(double val){
	
	std_msgs::Float64 ref_msg; 
	
	ref_msg.data = val;
	
	ref_pos_elevator_.publish( ref_msg );
}

/// Controller stopping
void stopping()
{}

/*
 *	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
 *
 */
void check_command_subscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();

	double diff = (current_time - last_command_time_).toSec();

	if(diff > 1.0){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
		//ROS_INFO("check_command_subscriber: %lf seconds without commands", diff);
		// TODO: Set Speed References to 0
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
}

// Service SetOdometry 
bool srvCallback_SetOdometry(robotnik_msgs::set_odometry::Request &request, robotnik_msgs::set_odometry::Response &response )
{
	// ROS_INFO("summit_xl_odometry::set_odometry: request -> x = %f, y = %f, a = %f", req.x, req.y, req.orientation);
	robot_pose_px_ = request.x;
	robot_pose_py_ = request.y;
	robot_pose_pa_ = request.orientation;

	response.ret = true;
	return true;
}

// Service Raise Elevator 
bool srvCallback_RaiseElevator(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
	
	SetElevatorPosition(MAX_ELEVATOR_POSITION);
	
	return true;
}

// Service Lower Elevator 
bool srvCallback_LowerElevator(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
	SetElevatorPosition(0.0);
	
	return true;
}


// Topic command
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{	
  joint_state_ = *msg;
  read_state_ = true;
}

// Topic command
void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
  // Safety check
  last_command_time_ = ros::Time::now();
  subs_command_freq->tick();			// For diagnostics

  double speed_limit = 2.0;  // m/s
  double angle_limit = PI;   
  v_ref_ = saturation(msg->drive.speed, -speed_limit, speed_limit);  
  a_ref_ = saturation(msg->drive.steering_angle, -angle_limit, angle_limit);
}

// Imu callback
void imuCallback( const sensor_msgs::Imu& imu_msg){

	orientation_diff_x_ = imu_msg.orientation.x;
	orientation_diff_y_ = imu_msg.orientation.y;
	orientation_diff_z_ = imu_msg.orientation.z;
	orientation_diff_w_ = imu_msg.orientation.w;

	ang_vel_x_ = imu_msg.angular_velocity.x;
	ang_vel_y_ = imu_msg.angular_velocity.y;
	ang_vel_z_ = imu_msg.angular_velocity.z;

	lin_acc_x_ = imu_msg.linear_acceleration.x;
	lin_acc_y_ = imu_msg.linear_acceleration.y;
    lin_acc_z_ = imu_msg.linear_acceleration.z;
}

double saturation(double u, double min, double max)
{
 if (u>max) u=max;
 if (u<min) u=min;
 return u; 
}



//! Normalize in rad 
static inline void radnorm(double* radians)
{
	while (*radians >= (PI)) {
		*radians -= 2.0 * PI;
		}
	while (*radians <= (-PI)) {
		*radians += 2.0 * PI;
		}
}

static inline double radnorm2( double value ) 
{
  while (value > 2.0*PI) value -= 2.0*PI;
  while (value < -2.0*PI) value += 2.0*PI;
  return value;
}

bool spin()
{
    ROS_INFO("agvs_robot_control::spin()");
    ros::Rate r(desired_freq_);  // 50.0 

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (starting() == 0)
      {
	    while(ros::ok() && node_handle_.ok()) {
          UpdateControl();
          UpdateOdometry();
          PublishOdometry();
          diagnostic_.update();
          ros::spinOnce();
	      r.sleep();
          }
	      ROS_INFO("END OF ros::ok() !!!");
      } else {
       // No need for diagnostic here since a broadcast occurs in start
       // when there is an error.
       usleep(1000000);
       ros::spinOnce();
      }
   }

   ROS_INFO("agvs_robot_control::spin() - end");
   return true;
}

}; // Class AGVSControllerClass

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_robot_control");

	ros::NodeHandle n;		
  	AGVSControllerClass sxlrc(n);

    sxlrc.spin();

	return (0);

	
}

