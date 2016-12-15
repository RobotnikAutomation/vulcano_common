/*
 * vulcano_arm_pad
 * Copyright (c) 2011, Robotnik Automation, SLL
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
 * \author Robotnik Automation SLL
 * \brief Allows to use a pad with a robot using the joint_trajectory_controller interface
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define DEFAULT_NUM_OF_BUTTONS      20
#define DEFAULT_AXIS_ANGULAR         1
#define DEFAULT_SCALE_ANGULAR        1.0
#define DEFAULT_MAX_ANGULAR_SPEED    2.0     // rad/s

#define JOINT_MIN_ANGLE             -2.9
#define JOINT_MAX_ANGLE             2.9


class VulcanoArmPad
{
        public:
    VulcanoArmPad();
    //! Updates diagnostics
    void Update();

    private:
    void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void publishJointTrajectory(std::vector<double> joint_positions, double time_from_start);
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    int angular_;
    double angular_scale_;
    
    ros::Publisher joint_trajectory_pub_;
    //! It will be suscribed to the joystick
    ros::Subscriber pad_sub_;

    ros::Subscriber joint_states_sub_;

    double current_vel;
    //! Number of the DEADMAN button
    int dead_man_button_;
    //! Number of the button for increase or decrease the selected joint
    int next_joint_button_; 
    int previous_joint_button_;
    //! Number of the button for increase or decrease the speed max of the joystick	
    int speed_up_button_, speed_down_button_;
    //! Number of buttons of the joystick
    int num_of_buttons_;
    //! Pointer to a vector for controlling the event when pushing the buttons
    bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
    // DIAGNOSTICS
    //! Diagnostic to control the frequency of the published command velocity topic
    diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
    //! Diagnostic to control the reception frequency of the subscribed joy topic 
    diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
    //! General status diagnostic updater
    diagnostic_updater::Updater updater_pad;    
    //! Diagnostics min freq
    double min_freq_command, min_freq_joy; // 
    //! Diagnostics max freq
    double max_freq_command, max_freq_joy; //   
    //! Flag to enable/disable the communication with the publishers topics
    bool bEnable;
    
    std::vector<std::string> joint_names_;
    
    int selected_joint_;
    sensor_msgs::JointState joint_states_msg_;
    
    struct joint_limit
    {
        double min;
        double max;
    };
    
    std::vector<joint_limit> joint_limits_;

    std::string arm_prefix_;
};


VulcanoArmPad::VulcanoArmPad()
{

    pnh_ = ros::NodeHandle("~");
    current_vel = 0.1;

    pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

    // MOTION CONF
    pnh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
    pnh_.param("scale_angular", angular_scale_, DEFAULT_SCALE_ANGULAR);
    pnh_.param("button_dead_man", dead_man_button_, dead_man_button_);
    pnh_.param("button_next_joint", next_joint_button_, 4);
    pnh_.param("button_previous_joint", previous_joint_button_, 5);
    pnh_.param("button_speed_up", speed_up_button_, speed_up_button_);  
    pnh_.param("button_speed_down", speed_down_button_, speed_down_button_); 
    pnh_.param<std::string>("arm_prefix", arm_prefix_, "");
    

    for(int i = 0; i < DEFAULT_NUM_OF_BUTTONS; i++){
        bRegisteredButtonEvent[i] = false;
    }


    // Initialize joint_names_
    joint_names_.push_back(arm_prefix_ + "elbow_joint");
    joint_names_.push_back(arm_prefix_ + "shoulder_lift_joint");
    joint_names_.push_back(arm_prefix_ + "shoulder_pan_joint");
    joint_names_.push_back(arm_prefix_ + "wrist_1_joint");
    joint_names_.push_back(arm_prefix_ + "wrist_2_joint");
    joint_names_.push_back(arm_prefix_ + "wrist_3_joint");
    
    // Initialize joint_limits_
    for(int i=0; i<joint_names_.size(); i++)
    {
        joint_limit limit;
        limit.min = JOINT_MIN_ANGLE;
        limit.max = JOINT_MAX_ANGLE;
        joint_limits_.push_back(limit);
    }
    
    selected_joint_ = 0;
    ROS_INFO("VulcanoArmPad::%s::padCallback: selected joint %s", arm_prefix_.c_str(), joint_names_[selected_joint_].c_str());



    // Listen through the node handle sensor_msgs::Joy messages from joystick (these are the orders that we will send to vulcano_arm_controller/command)
    pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &VulcanoArmPad::padCallback, this);
    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &VulcanoArmPad::jointStatesCallback, this);
    
    joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(arm_prefix_ + "position_joint_trajectory_controller/command", 1);

    // Diagnostics
    updater_pad.setHardwareID("None");
    // Topics freq control 
    min_freq_command = min_freq_joy = 5.0;
    max_freq_command = max_freq_joy = 50.0;
    sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
                        diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));


    bEnable = true; // Communication flag enabled by default
    
}

/*
 *  \brief Updates the diagnostic component. Diagnostics
 *
 */
void VulcanoArmPad::Update(){
    updater_pad.update();
}

/*
 *  \brief Method call when receiving a message from the topic /joy
 *
 */
void VulcanoArmPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    std::vector<double> joint_positions; // positions ordered as joint_names_ vector
    static bool bSentStop = false; 
    static std::vector<double> trajectory;

    // Check availavility of joint_states msg
    //ROS_INFO("now: %d, joint_states_: %d", ros::Time::now().sec, joint_states_msg_.header.stamp.sec);
    if( (ros::Time::now() - joint_states_msg_.header.stamp) > ros::Duration(1) )
    {
        ROS_ERROR_THROTTLE(1, "VulcanoArmPad::%s::padCallback: joint_states msg is too old", arm_prefix_.c_str());
        return;
    }
    
    // Check if we have a JointState msg with all the defined joints
    for(int i=0; i<joint_names_.size(); i++)
    {
        bool joint_found = false;
        for(int j=0; j<joint_states_msg_.name.size(); j++)
        {
            if(joint_states_msg_.name[j] == joint_names_[i])
            {
                joint_found = true;
                joint_positions.push_back(joint_states_msg_.position[j]);
                break;
            }
        }
        if(!joint_found)
        {
            ROS_ERROR_THROTTLE(1, "VulcanoArmPad::%s::padCallback: one or more joints can't be found in joint_states topic", arm_prefix_.c_str());
            return;
        }
    }
    //ROS_INFO("joint_positions size %d", (int)joint_positions.size());
 
    if(joy->buttons[dead_man_button_] == 1)
    { 

        if (trajectory.size() == 0) // to get it initialized at least once
            trajectory = joint_positions;

        // Next joint button
        if(joy->buttons[next_joint_button_] == 1 && !bRegisteredButtonEvent[next_joint_button_])
        {
            bRegisteredButtonEvent[next_joint_button_] = true;
            selected_joint_++;
            if(selected_joint_ == joint_names_.size())
                selected_joint_ = 0;
            ROS_INFO("VulcanoArmPad::%s::padCallback: selected joint %s", arm_prefix_.c_str(), joint_names_[selected_joint_].c_str());
            
            trajectory = joint_positions; // whenever we switch the selected joint, update trajectory
        }
        else if( joy->buttons[next_joint_button_] != 1) {
            bRegisteredButtonEvent[next_joint_button_] = false;
        }
        
        // Previous joint button
        if(joy->buttons[previous_joint_button_] == 1 && !bRegisteredButtonEvent[previous_joint_button_])
        {
            bRegisteredButtonEvent[previous_joint_button_] = true;
            selected_joint_--;
            if(selected_joint_ < 0)
                selected_joint_ = joint_names_.size()-1;
            ROS_INFO("VulcanoArmPad::%s::padCallback: selected joint %s", arm_prefix_.c_str(), joint_names_[selected_joint_].c_str());
            
            trajectory = joint_positions; // whenever we switch the selected joint, update trajectory
        }
        else if(joy->buttons[previous_joint_button_] != 1) {
            bRegisteredButtonEvent[previous_joint_button_] = false;
        }
        
        // Set the current velocity level
        if ( joy->buttons[speed_down_button_] == 1 ) {
            if(!bRegisteredButtonEvent[speed_down_button_]) {
                if(current_vel > 0.1){
                    current_vel = current_vel - 0.1;
                    bRegisteredButtonEvent[speed_down_button_] = true;
                    ROS_INFO("VulcanoArmPad::%s::Velocity: %f%%", arm_prefix_.c_str(), current_vel*100.0);	
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
                    ROS_INFO("VulcanoArmPad::%s::Velocity: %f%%", arm_prefix_.c_str(), current_vel*100.0);
                }
            }
        } else {
            bRegisteredButtonEvent[speed_up_button_] = false;
        }
            
            
        float desired_speed = angular_scale_*joy->axes[angular_];
        if (desired_speed == 0.0)
        {
            if (bSentStop == false) {
                publishJointTrajectory(joint_positions, 0.1); // Send current position
                // Stop movement
                bSentStop = true;
            }
            return; 
        }
            
        
        double distance;
        double desired_position;
        if(desired_speed > 0)
            desired_position = joint_limits_[selected_joint_].max;
        else if(desired_speed < 0)
            desired_position = joint_limits_[selected_joint_].min;
        distance = desired_position - joint_positions[selected_joint_];
        
        double time_from_start;
        if(desired_speed != 0.0) 
            time_from_start = 5/current_vel ; // distance / (current_vel * desired_speed); 
        else
            time_from_start = 0.0;
        
        if(time_from_start < 0.1)
            time_from_start = 0.1;
        //ROS_INFO("Desired position: %f, current position: %f, distance: %f", desired_position, joint_positions[selected_joint_], distance);
        //ROS_INFO("Desired_speed: %f, time_from_start: %f", desired_speed, time_from_start);
        
        // Publish trajectory
        //trajectory = joint_positions; 
        trajectory[selected_joint_] = desired_position; //update trajectory only for the desired joint
        publishJointTrajectory(trajectory, time_from_start);
        bSentStop = false;
    }
    else
    {
        if (bSentStop == false) { // to send the stop command only once when the deadman button is released
            // Stop movement
            publishJointTrajectory(joint_positions, 0.1); // Send current position
            bSentStop = true;
        }
        //if deadman is not pressed update trajectory with the current joint states
        trajectory = joint_positions;
    }

}

void VulcanoArmPad::publishJointTrajectory(std::vector<double> joint_positions, double time_from_start)
{
    if(!bEnable)
        return;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = ros::Time::now();
    
    joint_trajectory.joint_names = joint_names_;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = joint_positions;
    for (int i=0; i<joint_names_.size(); i++)
    {
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
        point.effort.push_back(0.0);
    }
    
    if(time_from_start < 0.0)
        time_from_start = 0.0;
    point.time_from_start = ros::Duration(time_from_start);
    joint_trajectory.points.push_back(point);
    
    // Publish joint trajectory
    joint_trajectory_pub_.publish(joint_trajectory);
    
    return;
}

void VulcanoArmPad::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_states_msg_ = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vulcano_arm_pad");
    VulcanoArmPad vulcano_arm_pad;

    ros::Rate r(50.0);

    while( ros::ok() ){

        // UPDATING DIAGNOSTICS
        vulcano_arm_pad.Update();
        ros::spinOnce();
        r.sleep();
    }


}

