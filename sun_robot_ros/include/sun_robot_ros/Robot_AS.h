/*

    Robot Action Server Class

    Copyright 2019 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef SUN_ROBOT_AS_H
#define SUN_ROBOT_AS_H

#include <actionlib/server/simple_action_server.h>
#include <sun_robot_msgs/MoveJointsAction.h>
#include <sun_robot_msgs/SimpleMoveJointsAction.h>
#include <sun_robot_msgs/MoveLineSegmentAction.h>
#include <sun_robot_msgs/MoveCircumferenceAction.h>
#include "Traj_Generators/Vector_Independent_Traj.h"
#include "Traj_Generators/Cartesian_Independent_Traj.h"
#include "Traj_Generators/Line_Segment_Traj.h"
#include "Traj_Generators/Position_Circumference_Traj.h"
#include "Traj_Generators/Rotation_Const_Axis_Traj.h"
#include "Traj_Generators/Quintic_Poly_Traj.h"
#include "sun_robot_msgs/ClikStatus.h"
#include "sun_robot_msgs/ClikSetMode.h"
#include "sun_robot_msgs/JointPositionVelocityStamped.h"
#include "sun_robot_msgs/PoseTwistStamped.h"

class Robot_AS {

private:

Robot_AS(); //Avoid default constructor

protected:

/*
    Node Handle
*/
ros::NodeHandle _nh;

/*
    Rate of actions
*/
double _hz;

/*
    True if an action is running
*/
bool _b_action_running;

/*
    True if an action has been preempted
*/
bool _b_action_preempted;

/*
    Num joints (used for checks...)
*/
unsigned int _num_joints;

/*
    Clik communication
*/
ros::ServiceClient _serviceGetClikStatus;
ros::ServiceClient _serviceSetClikMode;
ros::Publisher _pub_joints;
ros::Publisher _pub_cartesian;
std::string _clik_set_mode_error_str;

/*************************************
    Action MoveJoint
***************************************/
/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<sun_robot_msgs::MoveJointsAction> _move_joint_as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
/*
    Name of the joint action
*/
std::string _move_joint_action_str;
sun_robot_msgs::MoveJointsFeedback _move_joints_feedback;

/*
    SimpleActionServer 
*/
actionlib::SimpleActionServer<sun_robot_msgs::SimpleMoveJointsAction> _simple_move_joint_as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
/*
    Name of the action
*/
std::string _simple_move_joint_action_str;
sun_robot_msgs::SimpleMoveJointsFeedback _simple_move_joints_feedback;

/*************************************
    END Action MoveJoint
***************************************/

/*************************************
    Action MoveCartesian
***************************************/
/*
    SimpleActionServer
*/
actionlib::SimpleActionServer<sun_robot_msgs::MoveLineSegmentAction> _move_line_segment_as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
/*
    Name of the action
*/
std::string _move_line_segment_action_str;
sun_robot_msgs::MoveLineSegmentFeedback _move_line_segment_feedback;

/*
    SimpleActionServer
*/
actionlib::SimpleActionServer<sun_robot_msgs::MoveCircumferenceAction> _move_circumference_as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
/*
    Name of the action
*/
std::string _move_circumference_action_str;
sun_robot_msgs::MoveCircumferenceFeedback _move_circumference_feedback;


/*************************************
    END Action MoveCartesian
***************************************/

public:

Robot_AS(
    unsigned int num_joints,
    const ros::NodeHandle& nh,
    double hz,
    const std::string& service_clik_status,
    const std::string& service_set_clik_mode,
    const std::string& topic_joints_command,
    const std::string& topic_cartesian_command,
    const std::string& action_move_joints,
    const std::string& action_simple_move_joints,
    const std::string& action_move_line_segment,
    const std::string& action_move_circumference//,
    //const std::string& action_move_line,
    //const std::string& action_move_circular
);

/*
Robot_AS(
    unsigned int num_joints,
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_for_parmas
);
*/

/*
    Start the server
*/
void start();

protected:

/*************************************
    Common functions
***************************************/

TooN::Vector<3> _actual_position;
UnitQuaternion _actual_quaternion;
TooN::Vector<> _actual_qR;
TooN::Vector<6> _actual_clik_error;
bool updateActualStatus( bool b_refresh = false );

bool clikSetMode( uint8_t clik_mode );

void releaseResource();

/*************************************
    END Common functions
***************************************/

/*************************************
    Action MoveJoint
***************************************/

/*
#goal definition
float64[] desired_positions
float64[] durations
float64[] start_times
float64[] initial_velocities
float64[] final_velocities
float64[] initial_accelerations
float64[] final_accelerations
float64 start_delay
float64 steady_state_thr
---
#result definition
bool success
std::string msg
---
#feedback
float64[] positions
float64[] velocities
*/

void moveJointAbort( const std::string& msg );

void moveJointPublishFeedback( const TooN::Vector<>& qR, const TooN::Vector<>& dqR );

bool moveJointIsPreemptRequested();

void moveJointPreempted();

void moveJointSuccess();

void executeMoveJointCB( const sun_robot_msgs::MoveJointsGoalConstPtr &goal );

/*
#goal definition
float64[] desired_positions
float64 max_cruise_speed
float64 start_delay
float64 steady_state_thr
---
#result definition
bool success
std::string msg
---
#feedback
float64[] positions
float64[] velocities
*/

void simpleMoveJointAbort( const std::string& msg );

void simpleMoveJointPublishFeedback( const TooN::Vector<>& qR, const TooN::Vector<>& dqR );

bool simpleMoveJointIsPreemptRequested();

void simpleMoveJointPreempted();

void simpleMoveJointSuccess();

void executeSimpleMoveJointCB( const sun_robot_msgs::SimpleMoveJointsGoalConstPtr &goal );

/*
    CB that execute the joints trajectory
*/
void executeMoveJointGeneralCB( 
                                Vector_Traj_Interface& traj,
                                double start_delay,
                                double steady_state_thr,
                                const boost::function< void() >& successFcn,
                                const boost::function< void(const TooN::Vector<>&,const TooN::Vector<>&) >& publishFeedbackFcn,
                                const boost::function< bool() >& isPreemptRequestedFcn,
                                const boost::function< void() >& preemptedFcn,
                                const boost::function< void(const std::string&) >& abortFcn 
                                );

void publishQR( const TooN::Vector<>& qR, const TooN::Vector<>& dqR );

/*************************************
    END Action MoveJoint
***************************************/


/*************************************
    Action MoveCartesian
***************************************/

/*
#goal definition

uint8 MODE_ABS_BASE=0
uint8 MODE_REL_BASE=1
uint8 MODE_REL_TOOL=2
uint8 mode

geometry_msgs/Vector3 translation
float64 translation_duration
float64 translation_initial_velocity
float64 translation_final_velocity
float64 translation_initial_acceleration
float64 translation_final_acceleration
float64 translation_start_time

geometry_msgs/Vector3 rotation_axis
float64 rotation_angle
float64 rotation_duration
float64 rotation_initial_velocity
float64 rotation_final_velocity
float64 rotation_initial_acceleration
float64 rotation_final_acceleration
float64 rotation_start_time

float64 start_delay
float64 steady_state_thr
---
#result definition
bool success
std::string msg
---
#feedback
geometry_msgs/Pose pose
geometry_msgs/Twist twist
*/

void moveLineSegmentAbort( const std::string& msg );

void moveLineSegmentPublishFeedback(    
                                        double time_left,
                                        const TooN::Vector<3>& pos,
                                        const UnitQuaternion& quat,
                                        const TooN::Vector<3>& vel,
                                        const TooN::Vector<3>& w);

bool moveLineSegmentIsPreemptRequested();

void moveLineSegmentPreempted();

void moveLineSegmentSuccess();

/*
#goal definition

uint8 MODE_ABS_BASE=0
uint8 MODE_REL_BASE=1
uint8 MODE_REL_TOOL=2
uint8 mode

geometry_msgs/Vector3 normal
geometry_msgs/Vector3 point_on_normal
float64 circumference_duration
float64 circumference_initial_velocity
float64 circumference_final_velocity
float64 circumference_initial_acceleration
float64 circumference_final_acceleration
float64 circumference_start_time

geometry_msgs/Vector3 rotation_axis
float64 rotation_angle
float64 rotation_duration
float64 rotation_initial_velocity
float64 rotation_final_velocity
float64 rotation_initial_acceleration
float64 rotation_final_acceleration
float64 rotation_start_time

float64 start_delay
float64 steady_state_thr
---
#result definition
bool success
string msg
---
#feedback
geometry_msgs/Pose pose
geometry_msgs/Twist twist
*/

void moveCircumferenceAbort( const std::string& msg );

void moveCircumferencePublishFeedback(  
                                        double time_left,  
                                        const TooN::Vector<3>& pos,
                                        const UnitQuaternion& quat,
                                        const TooN::Vector<3>& vel,
                                        const TooN::Vector<3>& w);

bool moveCircumferenceIsPreemptRequested();

void moveCircumferencePreempted();

void moveCircumferenceSuccess();

void executeMoveCircumferenceCB( const sun_robot_msgs::MoveCircumferenceGoalConstPtr &goal );

void executeMoveLineSegmentCB( const sun_robot_msgs::MoveLineSegmentGoalConstPtr &goal );

/*
    CB that execute the joints trajectory
*/
void executeMoveCartesianGeneralCB( 
                                Cartesian_Traj_Interface& traj,
                                double start_delay,
                                double steady_state_thr,
                                const boost::function< void() >& successFcn,
                                const boost::function< void(double, const TooN::Vector<3>&,const UnitQuaternion&,const TooN::Vector<3>&,const TooN::Vector<3>&) >& publishFeedbackFcn,
                                const boost::function< bool() >& isPreemptRequestedFcn,
                                const boost::function< void() >& preemptedFcn,
                                const boost::function< void(const std::string&) >& abortFcn 
                                );

void publishCartesian(  const TooN::Vector<3>& pos, 
                        const UnitQuaternion& quat, 
                        const TooN::Vector<3>& vel, 
                        const TooN::Vector<3>& w
                        );


/*************************************
    END Action MoveCartesian
***************************************/


};

#endif