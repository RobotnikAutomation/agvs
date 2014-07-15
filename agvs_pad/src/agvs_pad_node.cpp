/*
 * agvs_pad
 * Copyright (c) 2013, Robotnik Automation, SLL
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
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the robot controller, sending the messages received from the joystick device
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>
#include <robotnik_msgs/enable_disable.h>
#include <robotnik_msgs/set_digital_output.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
// #include <agvs_controller/AckermannDriveStamped.h>
//#include <agvs_robot_control/AckermannDriveStamped.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <std_srvs/Empty.h>

#define DEFAULT_MAX_SKID_LINEAR_SPEED	2.0 //m/s
#define DEFAULT_MAX_ANGULAR_POSITION	2.0 // rads/s

#define MAX_NUM_OF_BUTTONS			16
#define MAX_NUM_OF_AXES				8
#define MAX_NUM_OF_BUTTONS_PS3		19
#define MAX_NUM_OF_AXES_PS3			20

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_NUM_OF_AXES			8

#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_ANGULAR		0	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0


#define DEFAULT_JOY			"/joy"

#define DEFAULT_HZ			50.0

//! Class to save the state of the buttons
class Button{
	int iPressed;
	bool bReleased;
	
	public:
	
	Button(){
		iPressed = 0;
		bReleased = false;
	}
	//! Set the button as 'pressed'/'released'
	void Press(int value){		
		if(iPressed and !value){
			bReleased = true;
			
		}else if(bReleased and value)
			bReleased = false;
			
		iPressed = value;
			
	}
	
	int IsPressed(){
		return iPressed;
	}
	
	bool IsReleased(){
		bool b = bReleased;
		bReleased = false;
		return b;
	}
};

////////////////////////////////////////////////////////////////////////
//                               		                                //
////////////////////////////////////////////////////////////////////////
class AgvsPad
{
	public:
	
	AgvsPad();
	
	void ControlLoop();
	int SetStateMode(int state, int arm_mode, int platform_mode);
	
	private:
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	char * StateToString(int state);
	int SwitchToState(int new_state);
	
	void PublishState();
	//! Enables/Disables the joystick
	bool EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res );
	void Update();
	
private:	
	
	ros::NodeHandle nh_;
	
	int axis_linear_speed_, axis_angular_position_;
	double l_scale_, a_scale_;
	double current_speed_lvl;
	//! Set the max speed sent to the robot
	double max_linear_speed_, max_angular_position_;
	//! Desired component's freq
	double desired_freq_;
	
	// TOPICS
	//! It will publish into command velocity (for the robot)
	ros::Publisher vel_pub_;
	
	//! they will be suscribed to the joysticks
	ros::Subscriber joy_sub_;
	//! // Name of the joystick's topic
	std::string  joy_topic_;	
	//! Name of the topic where it will be publishing the velocity
	std::string cmd_topic_vel;
	//! Name of the service where it will be modifying the digital outputs
	std::string cmd_service_io_;
	//! topic name for the state
	std::string topic_state_;
	//! Topic to publish the state
	ros::Publisher state_pub_;

	//! Name of the service called to raise the elevator
	std::string service_raise_elevator_;
	//! Name of the service called to lower the elevator
	std::string service_lower_elevator_;

	// SERVICES
	//! Service clients
	ros::ServiceServer enable_disable_srv_;	
	ros::ServiceClient set_digital_outputs_client_;  
	ros::ServiceClient raise_elevator_client_;  
	ros::ServiceClient lower_elevator_client_;  

	
	// JOYSTICK
	//! Current number of buttons of the joystick
	int num_of_buttons_;
	int num_of_axes_;
	
	//! Vector to save the axis values
	std::vector<float> fAxes;
	//! Vector to save and control the axis values
	std::vector<Button> vButtons;
	//! Number of the DEADMAN button
	int button_dead_man_;
	//! Number of the button for increase or decrease the speed max of the joystick	
	int button_speed_up_, button_speed_down_;
	int button_up_car_, button_down_car_;
	int output_1_, output_2_;
	bool bOutput1, bOutput2;
		
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; 	
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;
};


AgvsPad::AgvsPad():
  axis_linear_speed_(1),
  axis_angular_position_(2), nh_("~")
{
	
	current_speed_lvl = 0.1;
	// JOYSTICK CONFIG
	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	nh_.param("num_of_axes", num_of_axes_, DEFAULT_NUM_OF_AXES);
	nh_.param("desired_freq", desired_freq_, DEFAULT_HZ);
	
	if(num_of_axes_ > MAX_NUM_OF_AXES){
		num_of_axes_ = MAX_NUM_OF_AXES;
		ROS_INFO("AgvsPad::AgvsPad: Limiting the max number of axes to %d", MAX_NUM_OF_AXES);
	}
	if(num_of_buttons_ > MAX_NUM_OF_BUTTONS){
		num_of_buttons_ = MAX_NUM_OF_BUTTONS;
		ROS_INFO("AgvsPad::AgvsPad: Limiting the max number of buttons to %d", MAX_NUM_OF_BUTTONS);
	}
	
	nh_.param("topic_joy", joy_topic_, std::string(DEFAULT_JOY));	
	
	// MOTION CONF
	nh_.param("cmd_topic_vel", cmd_topic_vel, std::string("/agvs_controller/command"));
	
	nh_.param("button_dead_man", button_dead_man_, button_dead_man_);
	nh_.param("button_speed_up", button_speed_up_, button_speed_up_);
	nh_.param("button_speed_down", button_speed_down_, button_speed_down_); 
	
	nh_.param("max_angular_position", max_angular_position_, DEFAULT_MAX_ANGULAR_POSITION); 
	nh_.param("max_linear_speed_", max_linear_speed_, DEFAULT_MAX_SKID_LINEAR_SPEED); 
	nh_.param("axis_linear_speed", axis_linear_speed_, DEFAULT_AXIS_LINEAR_X); 
	nh_.param("axis_angular_position", axis_angular_position_, DEFAULT_AXIS_ANGULAR); 
	ROS_INFO("axis_linear_speed_ = %d, axis_angular = %d", axis_linear_speed_, axis_angular_position_);
	ROS_INFO("max_linear_speed = %lf, max_angular_speed = %lf", max_linear_speed_, max_angular_position_);
	
	// DIGITAL OUTPUTS CONF
	nh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
	nh_.param("button_up_car", button_up_car_, button_up_car_);
	nh_.param("button_down_car", button_down_car_, button_down_car_);
	nh_.param("output_1", output_1_, output_1_);
	nh_.param("output_2", output_2_, output_2_);
	
	nh_.param("topic_state", topic_state_, std::string("/agvs_pad/state"));
	
	nh_.param("service_raise_elevator", service_raise_elevator_, std::string("/agvs_robot_control/raise_elevator"));
	nh_.param("service_lower_elevator", service_lower_elevator_, std::string("/agvs_robot_control/lower_elevator"));
	
	ROS_INFO("AgvsPad num_of_buttons_ = %d, axes = %d, topic controller: %s, hz = %.2lf", num_of_buttons_, num_of_axes_, cmd_topic_vel.c_str(), desired_freq_);	
	
	for(int i = 0; i < MAX_NUM_OF_BUTTONS_PS3; i++){
		Button b;
		vButtons.push_back(b);
	}
	
	for(int i = 0; i < MAX_NUM_OF_AXES_PS3; i++){
		fAxes.push_back(0.0);
	}
	
	//
  	// Publish through the node handle Twist type messages to the guardian_controller/command topic
  	//this->vel_pub_ = nh_.advertise<agvs_controller::AckermannDriveStamped>(this->cmd_topic_vel, 1);
  	this->vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(this->cmd_topic_vel, 1);
	
	//
	// Publishes the state
	//state_pub_ = nh_.advertise<agvs_pad::rescuer_pad_state>(topic_state_, 1);
	
 	// Listen through the node handle sensor_msgs::Joy messages from joystick 
	// (these are the references that we will sent to rescuer_controller/command)
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_, 1, &AgvsPad::joyCallback, this);
	
 	// Request service to activate / deactivate digital I/O
	set_digital_outputs_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(cmd_service_io_);

    // Request raise or lower the elevator	
	raise_elevator_client_ = nh_.serviceClient<std_srvs::Empty>(service_raise_elevator_);
	lower_elevator_client_ = nh_.serviceClient<std_srvs::Empty>(service_lower_elevator_);
	
	
	bOutput1 = bOutput2 = false;

   
	// Diagnostics
	updater_pad.setHardwareID("AGVS-PAD");
	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel.c_str(), updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));

	// Advertises new service to enable/disable the pad
	enable_disable_srv_ = nh_.advertiseService("/agvs_pad/enable_disable",  &AgvsPad::EnableDisable, this);
	//
	bEnable = true;	// Communication flag enabled by default
	
}


/*
 *	\brief Updates the diagnostic component. Diagnostics
 * 		   Publishes the state
 *
 */
void AgvsPad::Update(){
	PublishState();
}

//! 
void AgvsPad::PublishState(){
	/*agvs_pad::rescuer_pad_state pad_state;
	
	
	pad_state.state = StateToString(iState);
	pad_state.arm_mode = ModeToString(iArmMode);
	pad_state.platform_mode = ModeToString(iPlatformMode);
	pad_state.speed_level = current_speed_lvl;
	pad_state.deadman_active = (bool) vButtons[button_dead_man_].IsPressed();
	
	state_pub_.publish(pad_state);*/
	
}

/*
 *	\brief Enables/Disables the pad
 *
 */
bool AgvsPad::EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res )
{
	bEnable = req.value;

	ROS_INFO("AgvsPad::EnablaDisable: Setting to %d", req.value);
	res.ret = true;
	return true;
}


void AgvsPad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	
	// First joystick being saved
	for(int i = 0; i < joy->axes.size(); i++){
		this->fAxes[i] = joy->axes[i];
	}
	for(int i = 0; i < joy->buttons.size(); i++){
		this->vButtons[i].Press(joy->buttons[i]);
	}
	
	//ROS_INFO("AgvsPad::joyCallback: num_of_axes = %d, buttons = %d", (int)(joy->axes.size()), (int)(joy->buttons.size()));
}


//! Controls the actions and states
void AgvsPad::ControlLoop(){
	double desired_linear_speed = 0.0, desired_angular_position = 0.0;
	//agvs_controller::AckermannDriveStamped ref_msg;
	ackermann_msgs::AckermannDriveStamped ref_msg;
	
	
	ros::Rate r(desired_freq_);   

    while(ros::ok()) {
		
		Update();
			
		if(bEnable){
						
			if(vButtons[button_dead_man_].IsPressed()){
				ref_msg.header.stamp = ros::Time::now();
				ref_msg.drive.jerk = 0.0; 
				ref_msg.drive.acceleration = 0.0; 
				ref_msg.drive.steering_angle_velocity = 0.0;
			
				desired_linear_speed = max_linear_speed_ * current_speed_lvl * fAxes[axis_linear_speed_];
				desired_angular_position = max_angular_position_ * fAxes[axis_angular_position_];
				
				ref_msg.drive.steering_angle = desired_angular_position;
				ref_msg.drive.speed = desired_linear_speed;
				
				// Publish into command_vel topic
				vel_pub_.publish(ref_msg);
			
				if(vButtons[button_speed_up_].IsReleased()){
					current_speed_lvl += 0.1;
					if(current_speed_lvl > 1.0)
						current_speed_lvl = 1.0;
				}
				if(vButtons[button_speed_down_].IsReleased()){
					current_speed_lvl -= 0.1;
					if(current_speed_lvl < 0.0)
						current_speed_lvl = 0.0;
				}				
				if (vButtons[button_up_car_].IsReleased()){
					std_srvs::Empty empty_srv;
					raise_elevator_client_.call( empty_srv );
					ROS_INFO("Raise elevator");
				}
				if (vButtons[button_down_car_].IsReleased()){
					std_srvs::Empty empty_srv;
					lower_elevator_client_.call( empty_srv );
					ROS_INFO("Lower elevator");
				}
							
			}else if(vButtons[button_dead_man_].IsReleased()){
				ref_msg.header.stamp = ros::Time::now();
				ref_msg.drive.jerk = 0.0; 
				ref_msg.drive.acceleration = 0.0; 
				ref_msg.drive.steering_angle_velocity = 0.0;
				
				ref_msg.drive.steering_angle = 0.0;
				ref_msg.drive.speed = 0.0;
				//ROS_INFO("AgvsPad::ControlLoop: Deadman released!");
				vel_pub_.publish(ref_msg);// Publish into command_vel topic
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}
    	
}


///////////////////////// MAIN /////////////////////////////////
int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_pad");
	AgvsPad pad;
	
	pad.ControlLoop();
	
}

