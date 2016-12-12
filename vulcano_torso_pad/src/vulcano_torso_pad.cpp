/*
 * vulcano_torso_pad
 * Copyright (c) 2012, Robotnik Automation, SLL
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

#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_AXIS_LINEAR_Z       3	
#define DEFAULT_AXIS_ANGULAR_Z      2
#define DEFAULT_AXIS_PAN            0
#define DEFAULT_AXIS_TILT		    1
#define DEFAULT_SCALE_LINEAR_Z      0.1 
#define DEFAULT_SCALE_ANGULAR_Z     0.1 
#define DEFAULT_SCALE_PAN		    2.0
#define DEFAULT_SCALE_TILT          2.0

#define ITERATIONS_AFTER_DEADMAN    3.0

class VulcanoTorsoPad
{
    public:
        VulcanoTorsoPad();
        void Update();

    private:
        void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states);

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        // TODO: improve by vectorizing
        int axis_linear_z_;
        double scale_linear_z_;
        double current_linear_z_;
        std::string joint_linear_z_;

        int axis_angular_z_;
        double scale_angular_z_;
        double current_angular_z_;
        std::string joint_angular_z_;

        int axis_pan_;
        double scale_pan_;
        double current_pan_;
        std::string joint_pan_;

        int axis_tilt_;
        double scale_tilt_;
        double current_tilt_;
        std::string joint_tilt_;

        //! It will publish into 
        ros::Publisher guidance_command_pub_;
        ros::Publisher pantilt_command_pub_;
        //! It will be suscribed to the joystick
        ros::Subscriber pad_sub_;
        //! It will be subscribed to the joint_states
        ros::Subscriber joint_states_sub_;

        sensor_msgs::JointState current_joint_states_;
        double current_vel;

        //! Number of the DEADMAN button
        int dead_man_button_;

        //! Number of the button for increase or decrease the speed max of the joystick	
        int speed_up_button_, speed_down_button_;

        //! Number of buttons of the joystick
        int num_of_buttons_;

        //! Pointer to a vector for controlling the event when pushing the buttons
        bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];

        // DIAGNOSTICS
        //! Diagnostic to control the frequency of the published command velocity topic
        diagnostic_updater::HeaderlessTopicDiagnostic *guidance_pub_command_freq; 
        diagnostic_updater::HeaderlessTopicDiagnostic *pantilt_pub_command_freq; 
        //! Diagnostic to control the reception frequency of the subscribed joy topic 
        diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
        //! General status diagnostic updater
        diagnostic_updater::Updater updater_pad;	
        //! Diagnostics min freq
        double min_freq_command, min_freq_joy; // 
        //! Diagnostics max freq
        double max_freq_command, max_freq_joy; // 	
        //! Guidance command topic
        std::string guidance_topic_;
        //! Pantilt command topic
        std::string pantilt_topic_;
        //! Checks if the joint states have been received
        bool joint_state_received_;
};

VulcanoTorsoPad::VulcanoTorsoPad()
{
    current_vel = 0.1;
    pnh_ = ros::NodeHandle("~");
    // 
    pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

    // MOTION CONF
    pnh_.param("axis_linear_z", axis_linear_z_, DEFAULT_AXIS_LINEAR_Z);
    pnh_.param("scale_linear_z", scale_linear_z_, DEFAULT_SCALE_LINEAR_Z);
    pnh_.param("axis_angular_z", axis_angular_z_, DEFAULT_AXIS_ANGULAR_Z);
    pnh_.param("scale_angular_z", scale_angular_z_, DEFAULT_SCALE_ANGULAR_Z);

    pnh_.param("axis_pan", axis_pan_, DEFAULT_AXIS_PAN);	
    pnh_.param("scale_pan", scale_pan_, DEFAULT_SCALE_PAN);
    pnh_.param("axis_tilt", axis_tilt_, DEFAULT_AXIS_TILT);	
    pnh_.param("axis_tilt", scale_tilt_, DEFAULT_SCALE_TILT);	

    pnh_.param("button_dead_man_torso", dead_man_button_, dead_man_button_);
    pnh_.param("button_speed_up", speed_up_button_, speed_up_button_);  
    pnh_.param("button_speed_down", speed_down_button_, speed_down_button_); 


    ROS_INFO("VulcanoTorsoPad::num_of_buttons_ = %d", num_of_buttons_);	
    for(int i = 0; i < num_of_buttons_; i++){
        bRegisteredButtonEvent[i] = false;
        ROS_INFO("bREG %d", i);
    }

    /*ROS_INFO("Service I/O = [%s]", cmd_service_io_.c_str());
      ROS_INFO("Topic PTZ = [%s]", guidance_topic_ptz_.c_str());
      ROS_INFO("Service I/O = [%s]", guidance_topic_vel_.c_str());
      ROS_INFO("Axis linear = %d", linear_);
      ROS_INFO("Axis angular = %d", angular_);
      ROS_INFO("Scale angular = %d", a_scale_);
      ROS_INFO("Deadman button = %d", dead_man_button_);
      ROS_INFO("OUTPUT1 button %d", button_output_1_);
      ROS_INFO("OUTPUT2 button %d", button_output_2_);
      ROS_INFO("OUTPUT1 button %d", button_output_1_);
      ROS_INFO("OUTPUT2 button %d", button_output_2_);*/	

    // Publish through the node handle the commands to the guidance controller
    nh_.param<std::string>("guidance_topic", guidance_topic_, "torso_guidance_position_joint_trajectory_controller/command");
    guidance_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(guidance_topic_, 1);
    nh_.param<std::string>("pantilt_topic", pantilt_topic_, "torso_pantilt_controller/command");
    pantilt_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(pantilt_topic_, 1);

    // Listen through the node handle sensor_msgs::Joy messages from joystick 
    // (these are the references that we will sent to cmd_vel)
    pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &VulcanoTorsoPad::padCallback, this);

    // Listen through the node handle to the sensor_msgs::JointState from the robot
    // (these are used as references for the commands)
    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &VulcanoTorsoPad::jointStatesCallback, this);

    // Diagnostics
    updater_pad.setHardwareID("None");
    // Topics freq control 
    min_freq_command = min_freq_joy = 5.0;
    max_freq_command = max_freq_joy = 50.0;
    sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
            diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

    guidance_pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(guidance_topic_.c_str(), updater_pad,
            diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));
    pantilt_pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(pantilt_topic_.c_str(), updater_pad,
            diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));

    joint_linear_z_ = "torso_guidance_elevation_joint";
    joint_angular_z_ = "torso_guidance_revolution_joint";
    joint_pan_ = "torso_pantilt_pan_joint";
    joint_tilt_ = "torso_pantilt_tilt_joint";
}

/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void VulcanoTorsoPad::Update(){
    updater_pad.update();
}

void VulcanoTorsoPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joint_state_received_ == false)
        return;

    static int send_iterations_after_dead_man;
    
    trajectory_msgs::JointTrajectory guidance_cmd;
    trajectory_msgs::JointTrajectory pantilt_cmd;
    
    guidance_cmd.header.stamp = pantilt_cmd.header.stamp = ros::Time::now();
    //  is frame_id needed??
    //guidance_cmd.header.frame_id = pantilt_cmd.header.frame_id = std::string("base_link");

    guidance_cmd.joint_names.push_back(joint_linear_z_);
    guidance_cmd.joint_names.push_back(joint_angular_z_);
    guidance_cmd.points.resize(1);
    guidance_cmd.points[0].time_from_start = ros::Duration(0.1); // TODO: Check this value!

    pantilt_cmd.joint_names.push_back(joint_pan_);
    pantilt_cmd.joint_names.push_back(joint_tilt_);
    pantilt_cmd.points.resize(1);
    pantilt_cmd.points[0].time_from_start = ros::Duration(0.1); // TODO: Check this value!

    // Actions dependant on dead-man button
    if (joy->buttons[dead_man_button_] == 1) {
        // Set the current velocity level
        if ( joy->buttons[speed_down_button_] == 1 ) {
            if(!bRegisteredButtonEvent[speed_down_button_]) {
                if(current_vel > 0.1){
                    current_vel = current_vel - 0.1;
                    bRegisteredButtonEvent[speed_down_button_] = true;
                    ROS_INFO("VulcanoTorsoPad::Velocity: %f%%", current_vel*100.0);	
                }
            }
        } else {
            bRegisteredButtonEvent[speed_down_button_] = false;
        }

        if (joy->buttons[speed_up_button_] == 1) {
            if(!bRegisteredButtonEvent[speed_up_button_]) {
                if(current_vel < 0.9){
                    current_vel = current_vel + 0.1;
                    bRegisteredButtonEvent[speed_up_button_] = true;
                    ROS_INFO("VulcanoTorsoPad::Velocity: %f%%", current_vel*100.0);
                }
            }
        } else {
            bRegisteredButtonEvent[speed_up_button_] = false;
        }

        guidance_cmd.points[0].positions.push_back(current_linear_z_ + current_vel*scale_linear_z_*joy->axes[axis_linear_z_]);
        guidance_cmd.points[0].positions.push_back(current_angular_z_ + current_vel*scale_angular_z_*joy->axes[axis_angular_z_]);

        pantilt_cmd.points[0].positions.push_back(current_pan_ + current_vel*scale_pan_*joy->axes[axis_pan_]);
        pantilt_cmd.points[0].positions.push_back(current_tilt_ + current_vel*scale_tilt_*joy->axes[axis_tilt_]);
    }

    if (joy->buttons[dead_man_button_] == 0) { // it is not pressed
        guidance_cmd.points[0].positions.push_back(current_linear_z_);
        guidance_cmd.points[0].positions.push_back(current_angular_z_);

        pantilt_cmd.points[0].positions.push_back(current_pan_);
        pantilt_cmd.points[0].positions.push_back(current_tilt_);
    }

    sus_joy_freq->tick();	// Ticks the reception of joy events

    // Publish only with deadman button pushed
    if (joy->buttons[dead_man_button_] == 1) {                
        send_iterations_after_dead_man = ITERATIONS_AFTER_DEADMAN;
        
        guidance_command_pub_.publish(guidance_cmd);
        guidance_pub_command_freq->tick();
        
        pantilt_command_pub_.publish(pantilt_cmd);
        pantilt_pub_command_freq->tick();
    }
    else { // send some 0 if deadman is released
        if (send_iterations_after_dead_man >0) {
            send_iterations_after_dead_man--;
            
            guidance_command_pub_.publish(guidance_cmd);
            guidance_pub_command_freq->tick();
            
            pantilt_command_pub_.publish(pantilt_cmd);
            pantilt_pub_command_freq->tick();
        }
    }

}

void VulcanoTorsoPad::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
    static bool received[4] = {false, false, false, false};
    for (size_t i = 0; i < joint_states->name.size(); i++) {
        if (joint_states->name[i] == joint_linear_z_) {
            if (joint_states->position.size() > i) {
                current_linear_z_ = joint_states->position[i];
                received[0] = true;
            }
        }
        if (joint_states->name[i] == joint_angular_z_) {
            if (joint_states->position.size() > i) {
                current_angular_z_ = joint_states->position[i];
                received[1] = true;
            }
        }
        if (joint_states->name[i] == joint_pan_) {
            if (joint_states->position.size() > i) {
                current_pan_ = joint_states->position[i];
                received[2] = true;
            }
        }
        if (joint_states->name[i] == joint_tilt_) {
            if (joint_states->position.size() > i) {
                current_tilt_ = joint_states->position[i];
                received[3] = true;
            }
        }
    }

    joint_state_received_ = true;
    for (size_t i = 0; i < 4; i++)
        joint_state_received_ &= received[i];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vulcano_torso_pad");
    VulcanoTorsoPad vulcano_torso_pad;

    ros::Rate r(50.0);

    while( ros::ok() ){
        // UPDATING DIAGNOSTICS
        vulcano_torso_pad.Update();
        ros::spinOnce();
        r.sleep();
    }
}

