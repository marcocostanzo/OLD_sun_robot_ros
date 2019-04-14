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

#include "sun_robot_ros/Robot_AS.h"

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*===============================*/

#endif

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "][ROBOT_AS] " CRESET 

using namespace std;
using namespace TooN;

Robot_AS::Robot_AS(
    unsigned int num_joints,
    const ros::NodeHandle& nh,
    double hz,
    const string& service_clik_status,
    const string& service_set_clik_mode,
    const string& topic_joints_command,
    const string& topic_cartesian_command,
    const string& action_move_joints,
    const string& action_simple_move_joints,
    const string& action_move_line_segment,
    const string& action_move_circumference
)
:_num_joints(num_joints),
_nh(nh),
_hz(hz),
_move_joint_action_str(action_move_joints),
_move_joint_as(_nh, action_move_joints, boost::bind(&Robot_AS::executeMoveJointCB, this, _1), false),
_simple_move_joint_action_str(action_simple_move_joints),
_simple_move_joint_as(_nh, action_simple_move_joints, boost::bind(&Robot_AS::executeSimpleMoveJointCB, this, _1), false),
_move_line_segment_action_str(action_move_line_segment),
_move_line_segment_as(_nh, action_move_line_segment, boost::bind(&Robot_AS::executeMoveLineSegmentCB, this, _1), false),
_move_circumference_action_str(action_move_circumference),
_move_circumference_as(_nh, action_move_circumference, boost::bind(&Robot_AS::executeMoveCircumferenceCB, this, _1), false),
_b_action_running(false),
_b_action_preempted(false),
_actual_qR(Zeros(_num_joints))
{
    _serviceGetClikStatus = _nh.serviceClient<sun_robot_msgs::ClikStatus>(service_clik_status);
    _serviceSetClikMode = _nh.serviceClient<sun_robot_msgs::ClikSetMode>(service_set_clik_mode);
    _pub_joints = _nh.advertise<sun_robot_msgs::JointPositionVelocityStamped>(topic_joints_command, 1);
    _pub_cartesian = _nh.advertise<sun_robot_msgs::PoseTwistStamped>(topic_cartesian_command, 1);
    
    _move_joints_feedback.positions.resize(_num_joints);
    _move_joints_feedback.velocities.resize(_num_joints);
    _simple_move_joints_feedback.positions.resize(_num_joints);
    _simple_move_joints_feedback.velocities.resize(_num_joints);
}

/*Robot_AS::Robot_AS(
    unsigned int num_joints,
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_for_parmas
)
:_num_joints(num_joints),
_nh(nh),
_b_action_running(false),
_b_action_preempted(false),
_actual_qR(Zeros(_num_joints))
{

    nh_for_parmas.param("hz" , _hz, 1000.0 );
    nh_for_parmas.param("action_move_joint" , _move_joint_action_str, string("move_joint") );
    nh_for_parmas.param("action_simple_move_joint" , _simple_move_joint_action_str, string("simple_move_joint") );
    nh_for_parmas.param("action_line_segment" , _move_line_segment_action_str, string("line_segment") );

    string service_clik_status;    
    nh_for_parmas.param("service_get_clik_status" , service_clik_status, string("clik/get_status") );
    string service_set_clik_mode;    
    nh_for_parmas.param("service_set_clik_mode" , service_set_clik_mode, string("clik/set_mode") );
    string topic_joints_command;    
    nh_for_parmas.param("topic_joints_command" , topic_joints_command, string("joints_command") );
    string topic_cartesian_command;  
    nh_for_parmas.param("topic_cartesian_command" , topic_cartesian_command, string("cartesian_command") );

    _move_joint_as = actionlib::SimpleActionServer<sun_robot_msgs::MoveJointsAction>(_nh, _move_joint_action_str, boost::bind(&Robot_AS::executeMoveJointCB, this, _1), false)
    _simple_move_joint_as = actionlib::SimpleActionServer<sun_robot_msgs::SimpleMoveJointsAction>(_nh, _simple_move_joint_action_str, boost::bind(&Robot_AS::executeSimpleMoveJointCB, this, _1), false)
    _move_line_segment_as = actionlib::SimpleActionServer<sun_robot_msgs::MoveLineSegmentAction>(_nh, _move_line_segment_action_str, boost::bind(&Robot_AS::executeMoveLineSegmentCB, this, _1), false)

    _serviceGetClikStatus = _nh.serviceClient<sun_robot_msgs::ClikStatus>(service_clik_status);
    _serviceSetClikMode = _nh.serviceClient<sun_robot_msgs::ClikSetMode>(service_set_clik_mode);
    _pub_joints = _nh.advertise<sun_robot_msgs::JointPositionVelocityStamped>(topic_joints_command, 1);
    _pub_cartesian = _nh.advertise<sun_robot_msgs::PoseTwistStamped>(topic_cartesian_command, 1);
    
    _move_joints_feedback.positions.resize(_num_joints);
    _move_joints_feedback.velocities.resize(_num_joints);
    _simple_move_joints_feedback.positions.resize(_num_joints);
    _simple_move_joints_feedback.velocities.resize(_num_joints);
}*/

/*
    Start the server
*/
void Robot_AS::start(){
    cout << HEADER_PRINT YELLOW "Wait for servers..." CRESET << endl;
    _serviceGetClikStatus.waitForExistence();
    _serviceSetClikMode.waitForExistence();
    cout << HEADER_PRINT GREEN "Servers online!" CRESET << endl;
    _move_joint_as.start();
    _simple_move_joint_as.start();
    _move_line_segment_as.start();
    _move_circumference_as.start();
    cout << HEADER_PRINT GREEN "STARTED!" CRESET << endl;
}

/*************************************
    Common functions
***************************************/

bool Robot_AS::updateActualStatus( bool b_refresh ){

    sun_robot_msgs::ClikStatus status_msg;

    status_msg.request.refresh = b_refresh;

    bool b_result = _serviceGetClikStatus.call(status_msg);
    b_result = b_result && status_msg.response.success;

    if(!b_result){
        return false;
    }

    if( status_msg.response.qR.size() != _num_joints ){
        cout << HEADER_PRINT BOLDRED "ERROR! NumJoints size mismatch" CRESET << endl;
        return false;
    }

    _actual_position[0] = status_msg.response.pose.position.x;
    _actual_position[1] = status_msg.response.pose.position.y;
    _actual_position[2] = status_msg.response.pose.position.z;

    _actual_quaternion = UnitQuaternion(
        status_msg.response.pose.orientation.w, 
        makeVector(
            status_msg.response.pose.orientation.x,
            status_msg.response.pose.orientation.y,
            status_msg.response.pose.orientation.z
        )
    );

    for( int i=0; i<_num_joints; i++ ){
        _actual_qR[i] = status_msg.response.qR[i];
    }

    _actual_clik_error[0] = status_msg.response.clik_error[0];
    _actual_clik_error[1] = status_msg.response.clik_error[1];
    _actual_clik_error[2] = status_msg.response.clik_error[2];
    _actual_clik_error[3] = status_msg.response.clik_error[3];
    _actual_clik_error[4] = status_msg.response.clik_error[4];
    _actual_clik_error[5] = status_msg.response.clik_error[5];

    return true;
    
}

bool Robot_AS::clikSetMode( uint8_t clik_mode ){

    _clik_set_mode_error_str = "Failed to set Clik Mode";

    sun_robot_msgs::ClikSetMode setmode_msg;

    setmode_msg.request.mode = clik_mode;
    bool b_result = _serviceSetClikMode.call(setmode_msg);
    b_result = b_result && setmode_msg.response.success;

    return b_result;

}

void Robot_AS::releaseResource(){

    if( !clikSetMode(sun_robot_msgs::ClikSetMode::Request::MODE_STOP) ){
        cout << HEADER_PRINT BOLDRED "Failed to STOP Clik in releaseResource()" CRESET << endl;
    }

    _b_action_preempted = false;
    _b_action_running = false;

}

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
string msg
---
#feedback
float64[] positions
float64[] velocities
*/

_ROBOT_AS_GENERATE_JOINT_FCNS_ALL( moveJoint , MoveJoints , _move_joints_feedback , _move_joint_as )

void Robot_AS::executeMoveJointCB( const sun_robot_msgs::MoveJointsGoalConstPtr &goal ){

    if(_b_action_running){
        moveJointAbort("Another action is running");
        return;
    }

    _b_action_running = true;

    //CHECK
    if( (goal->desired_positions.size()!=_num_joints) ||
        (goal->durations.size()!=_num_joints) ||
        (goal->start_times.size()!=_num_joints) ||
        (goal->initial_velocities.size()!=_num_joints) ||
        (goal->final_velocities.size()!=_num_joints) ||
        (goal->initial_accelerations.size()!=_num_joints) ||
        (goal->final_accelerations.size()!=_num_joints) ){

        cout << HEADER_PRINT BOLDRED "Joint size mismatch" CRESET << endl;
        moveJointAbort("Joint size mismatch");
        releaseResource();
        return;

    }

    //GET INITIAL POSITION!
    if( !updateActualStatus(true) ){
        cout << HEADER_PRINT BOLDRED "Failed to update Status" CRESET << endl;
        moveJointAbort("Failed to update Status");
        releaseResource();
        return;
    }

    //Build trajectory
    Vector_Independent_Traj total_traj;
    for(int i = 0; i<_num_joints; i++){
        total_traj.push_back_traj(
            Quintic_Poly_Traj(   
                goal->durations[i], //duration
                _actual_qR[i], //pi
                goal->desired_positions[i], //pf
                goal->start_times[i], //ti                           
                goal->initial_velocities[i], //vi
                goal->final_velocities[i],  //vf
                goal->initial_accelerations[i], //ai 
                goal->final_accelerations[i] //af
            )
        );
    }

    executeMoveJointGeneralCB( 
                                total_traj,
                                goal->start_delay,
                                goal->steady_state_thr,
                                boost::bind(&Robot_AS::moveJointSuccess, this),
                                boost::bind(&Robot_AS::moveJointPublishFeedback, this, _1, _2),
                                boost::bind(&Robot_AS::moveJointIsPreemptRequested, this),
                                boost::bind(&Robot_AS::moveJointPreempted, this),
                                boost::bind(&Robot_AS::moveJointAbort, this, _1) 
                                );

}

/*
#goal definition
float64[] desired_positions
float64 max_cruise_speed
float64 start_delay
float64 steady_state_thr
---
#result definition
bool success
string msg
---
#feedback
float64[] positions
float64[] velocities
*/

_ROBOT_AS_GENERATE_JOINT_FCNS_ALL( simpleMoveJoint , SimpleMoveJoints , _simple_move_joints_feedback , _simple_move_joint_as )

void Robot_AS::executeSimpleMoveJointCB( const sun_robot_msgs::SimpleMoveJointsGoalConstPtr &goal ){

    if(_b_action_running){
        simpleMoveJointAbort("Another action is running");
        return;
    }

    _b_action_running = true;

    //CHECK
    if( goal->desired_positions.size()!=_num_joints ){

        cout << HEADER_PRINT BOLDRED "Joint size mismatch" CRESET << endl;
        simpleMoveJointAbort("Joint size mismatch");
        releaseResource();
        return;

    }

    //GET INITIAL POSITION!
    if( !updateActualStatus(true) ){
        cout << HEADER_PRINT BOLDRED "Failed to update Status" CRESET << endl;
        simpleMoveJointAbort("Failed to update Status");
        releaseResource();
        return;
    }

    Vector<> q_desired = wrapVector( goal->desired_positions.data(), goal->desired_positions.size() );
    double duration = max( abs( _actual_qR - q_desired ) )/goal->max_cruise_speed;

    //Build trajectory
    Vector_Independent_Traj total_traj;
    for(int i = 0; i<_num_joints; i++){
        total_traj.push_back_traj(
            Quintic_Poly_Traj(   
                duration, //duration
                _actual_qR[i], //pi
                goal->desired_positions[i], //pf
                0.0, //ti                           
                0.0, //vi
                0.0,  //vf
                0.0, //ai 
                0.0 //af
            )
        );
    }

    executeMoveJointGeneralCB( 
                                total_traj,
                                goal->start_delay,
                                goal->steady_state_thr,
                                boost::bind(&Robot_AS::simpleMoveJointSuccess, this),
                                boost::bind(&Robot_AS::simpleMoveJointPublishFeedback, this, _1, _2),
                                boost::bind(&Robot_AS::simpleMoveJointIsPreemptRequested, this),
                                boost::bind(&Robot_AS::simpleMoveJointPreempted, this),
                                boost::bind(&Robot_AS::simpleMoveJointAbort, this, _1) 
                                );

}

/*
    CB that execute the joints trajectory
*/
void Robot_AS::executeMoveJointGeneralCB( 
                                Vector_Traj_Interface& traj,
                                double start_delay,
                                double steady_state_thr,
                                const boost::function< void() >& successFcn,
                                const boost::function< void(const Vector<>&,const Vector<>&) >& publishFeedbackFcn,
                                const boost::function< bool() >& isPreemptRequestedFcn,
                                const boost::function< void() >& preemptedFcn,
                                const boost::function< void(const string&) >& abortFcn 
                                ){

    //SET CLIK MODE JOINTS
    if( !clikSetMode(sun_robot_msgs::ClikSetMode::Request::MODE_JOINT) ){
        cout << HEADER_PRINT BOLDRED << _clik_set_mode_error_str << CRESET << endl;
        abortFcn(_clik_set_mode_error_str);
        releaseResource();
        return;
    }

    ros::Rate loop_rate(_hz);
    double time_now = ros::Time::now().toSec();

    traj.changeInitialTime( time_now + start_delay );
    cout << HEADER_PRINT GREEN "Start Joint Trajectory..." CRESET << endl;
    while( ros::ok() && !_b_action_preempted && !traj.isCompleate(time_now) ){

        ros::spinOnce();

        //check preempt
        if (isPreemptRequestedFcn()){
            _b_action_preempted = true;
            break;
        }

        time_now = ros::Time::now().toSec();

        Vector<> qR = traj.getPosition(  time_now );
        Vector<> dqR = traj.getVelocity(  time_now );

        publishQR(qR,dqR);

        publishFeedbackFcn(qR,dqR);

        loop_rate.sleep();

    }

    //whait steady state
    if( steady_state_thr!=0 && !_b_action_preempted ){
        cout << HEADER_PRINT YELLOW "Joint Trajectory compleate, waiting for steady state..." CRESET << endl;

        double error_steady = INFINITY;

        Vector<> qR_final = traj.getPosition(  time_now );
        Vector<> dqR_final = traj.getVelocity(  time_now );

        while(ros::ok() && !_b_action_preempted && fabs(error_steady)>steady_state_thr ){

            ros::spinOnce();

            //check preempt
            if (isPreemptRequestedFcn()){
                _b_action_preempted = true;
                break;
            }

            if( !updateActualStatus(true) ){
                cout << HEADER_PRINT BOLDRED "Failed to update Status" CRESET << endl;
                abortFcn("Failed to update Status");
                releaseResource();
                return;
            }

            error_steady = norm( qR_final - _actual_qR );

            publishQR(qR_final,dqR_final);
            
            publishFeedbackFcn(qR_final,qR_final);

            loop_rate.sleep();

        }

    }

    if(_b_action_preempted){
        cout << HEADER_PRINT BOLDYELLOW "Joint Trajectory PREEMPTED!" CRESET << endl;
        preemptedFcn();
    } else{
        cout << HEADER_PRINT BOLDGREEN "Joint Trajectory Compleate!" CRESET << endl;
        successFcn();
    }

    releaseResource();

}

void Robot_AS::publishQR( const Vector<>& qR, const Vector<>& dqR ){

    sun_robot_msgs::JointPositionVelocityStamped out_msg;

    out_msg.position.resize(_num_joints);
    out_msg.velocity.resize(_num_joints);

    for(int i=0; i<_num_joints; i++){
        out_msg.position[i] = qR[i];
        out_msg.velocity[i] = dqR[i];
    }

    out_msg.header.frame_id = "robot_as_joints_trajectory";
    out_msg.header.stamp = ros::Time::now();

    _pub_joints.publish(out_msg);

}

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
string msg
---
#feedback
geometry_msgs/Pose pose
geometry_msgs/Twist twist
*/

_ROBOT_AS_GENERATE_CARTESIAN_FCNS_ALL( moveLineSegment, MoveLineSegment, _move_line_segment_feedback, _move_line_segment_as )

void Robot_AS::executeMoveLineSegmentCB( const sun_robot_msgs::MoveLineSegmentGoalConstPtr &goal ){

    if(_b_action_running){
        moveLineSegmentAbort("Another action is running");
        return;
    }

    _b_action_running = true;

    //GET INITIAL POSITION!
    if( !updateActualStatus(true) ){
        cout << HEADER_PRINT BOLDRED "Failed to update Status" CRESET << endl;
        moveLineSegmentAbort("Failed to update Status");
        releaseResource();
        return;
    }

    //Set Traj Mode
    Vector<3> pf = makeVector(
                    goal->translation.x,
                    goal->translation.y,
                    goal->translation.z
                    );
    Vector<3> rotation_axis = makeVector(
                                goal->rotation_axis.x,
                                goal->rotation_axis.y,
                                goal->rotation_axis.z
                                );
    double rotation_angle = goal->rotation_angle;
    switch(goal->mode){
        case sun_robot_msgs::MoveLineSegmentGoal::MODE_ABS_BASE:{
            //OK
            //future: change rotation? w. absolute rotation?
            AngVec out_ang_vec = (UnitQuaternion( AngVec(rotation_axis,rotation_angle) )/_actual_quaternion).toangvec();
            cout << HEADER_PRINT MAGENTA "vec = " << out_ang_vec.getVec() << " ang = " << out_ang_vec.getAng() << CRESET << endl;
            break;
        }
        case sun_robot_msgs::MoveLineSegmentGoal::MODE_REL_BASE:{
            pf = _actual_position + pf;            
            break;
        }
        case sun_robot_msgs::MoveLineSegmentGoal::MODE_REL_TOOL:{
            Matrix<4,4> b_T_e = Identity;
            b_T_e.slice<0,0,3,3>() = _actual_quaternion.torot();
            b_T_e.T()[3].slice<0,3>() = _actual_position;

            Vector<4> pf_tilde = Ones;
            pf_tilde.slice<0,3>() = pf;
            pf_tilde = b_T_e*pf_tilde;
            pf = pf_tilde.slice<0,3>();

            rotation_axis = b_T_e.slice<0,0,3,3>() * rotation_axis;

            break;
        }
        default:{
            cout << HEADER_PRINT BOLDRED "MoveLineSegment Invalid Mode" CRESET << endl;
            moveLineSegmentAbort("Invalid Mode");
            releaseResource();
            return;
        }
    }

    //Build Traj
    double scalar_traj_scale = norm(pf-_actual_position);
    if( scalar_traj_scale < 10.0*std::numeric_limits<double>::epsilon()  ){
        scalar_traj_scale = 1.0;
    }
    Cartesian_Independent_Traj cart_traj( 
            //LineTraj
            Line_Segment_Traj( 
                _actual_position,//pi
                pf,//pf
                //Scalar Traj
                Quintic_Poly_Traj(   
                    goal->translation_duration,//duration
                    0.0,//double initial_position,
                    1.0,//double final_position,
                    goal->translation_start_time,                            
                    goal->translation_initial_velocity/scalar_traj_scale, 
                    goal->translation_final_velocity/scalar_traj_scale,
                    goal->translation_initial_acceleration/scalar_traj_scale, 
                    goal->translation_final_acceleration/scalar_traj_scale
                    )
            )   
            , 
            //Rotation Traj
            Rotation_Const_Axis_Traj( 
                _actual_quaternion,// initial_quat, 
                rotation_axis,//axis,
                Quintic_Poly_Traj(   
                    goal->rotation_duration,//duration
                    0.0,//double initial_position,
                    rotation_angle,//double final_position,
                    goal->rotation_start_time,                            
                    goal->rotation_initial_velocity, 
                    goal->rotation_final_velocity,
                    goal->rotation_initial_acceleration, 
                    goal->rotation_final_acceleration
                    )
            )
    );

    executeMoveCartesianGeneralCB( 
                                cart_traj,
                                goal->start_delay,
                                goal->steady_state_thr,
                                boost::bind(&Robot_AS::moveLineSegmentSuccess, this),
                                boost::bind(&Robot_AS::moveLineSegmentPublishFeedback, this, _1, _2,_3,_4, _5),
                                boost::bind(&Robot_AS::moveLineSegmentIsPreemptRequested, this),
                                boost::bind(&Robot_AS::moveLineSegmentPreempted, this),
                                boost::bind(&Robot_AS::moveLineSegmentAbort, this, _1) 
                                );

}

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

_ROBOT_AS_GENERATE_CARTESIAN_FCNS_ALL( moveCircumference, MoveCircumference, _move_circumference_feedback, _move_circumference_as )

void Robot_AS::executeMoveCircumferenceCB( const sun_robot_msgs::MoveCircumferenceGoalConstPtr &goal ){

    if(_b_action_running){
        moveCircumferenceAbort("Another action is running");
        return;
    }

    _b_action_running = true;

    //GET INITIAL POSITION!
    if( !updateActualStatus(true) ){
        cout << HEADER_PRINT BOLDRED "Failed to update Status" CRESET << endl;
        moveCircumferenceAbort("Failed to update Status");
        releaseResource();
        return;
    }

    //Set Traj Mode
    Vector<3> normal = makeVector(
                    goal->normal.x,
                    goal->normal.y,
                    goal->normal.z
                    );
    Vector<3> point_on_normal = makeVector(
                                goal->point_on_normal.x,
                                goal->point_on_normal.y,
                                goal->point_on_normal.z
                                );
    Vector<3> rotation_axis = makeVector(
                                goal->rotation_axis.x,
                                goal->rotation_axis.y,
                                goal->rotation_axis.z
                                );

    double rotation_angle = goal->rotation_angle;
    switch(goal->mode){
        case sun_robot_msgs::MoveCircumferenceGoal::MODE_ABS_BASE:{
            //OK
            //future: change rotation? w. absolute rotation?
            AngVec out_ang_vec = (UnitQuaternion( AngVec(rotation_axis,rotation_angle) )/_actual_quaternion).toangvec();
            cout << HEADER_PRINT MAGENTA "vec = " << out_ang_vec.getVec() << " ang = " << out_ang_vec.getAng() << CRESET << endl;
            break;
        }
        case sun_robot_msgs::MoveCircumferenceGoal::MODE_REL_BASE:{
            point_on_normal = _actual_position + point_on_normal;          
            break;
        }
        case sun_robot_msgs::MoveCircumferenceGoal::MODE_REL_TOOL:{
            Matrix<4,4> b_T_e = Identity;
            b_T_e.slice<0,0,3,3>() = _actual_quaternion.torot();
            b_T_e.T()[3].slice<0,3>() = _actual_position;

            Vector<4> point_on_normal_tilde = Ones;
            point_on_normal_tilde.slice<0,3>() = point_on_normal;
            point_on_normal_tilde = b_T_e*point_on_normal_tilde;
            point_on_normal = point_on_normal_tilde.slice<0,3>();

            normal = b_T_e.slice<0,0,3,3>() * normal;
            rotation_axis = b_T_e.slice<0,0,3,3>() * rotation_axis;

            break;
        }
        default:{
            cout << HEADER_PRINT BOLDRED "MoveCircumference Invalid Mode" CRESET << endl;
            moveCircumferenceAbort("Invalid Mode");
            releaseResource();
            return;
        }
    }

    cout << HEADER_PRINT "normal: " << normal << endl;
    cout << HEADER_PRINT "Point on normal: " << point_on_normal << endl;
    cout << HEADER_PRINT "Angle: " << goal->circumference_angle << endl;  

    //Build Traj
    Cartesian_Independent_Traj cart_traj( 
            //LineTraj
            Position_Circumference_Traj(    
                normal, 
                point_on_normal,
                _actual_position, 
                Quintic_Poly_Traj(   
                    goal->circumference_duration,//duration
                    0.0,//double initial_position,
                    goal->circumference_angle,//double final_position,
                    goal->circumference_start_time,                            
                    goal->circumference_initial_velocity, 
                    goal->circumference_final_velocity,
                    goal->circumference_initial_acceleration, 
                    goal->circumference_final_acceleration
                    )
            )   
            , 
            //Rotation Traj
            Rotation_Const_Axis_Traj( 
                _actual_quaternion,// initial_quat, 
                rotation_axis,//axis,
                Quintic_Poly_Traj(   
                    goal->rotation_duration,//duration
                    0.0,//double initial_position,
                    goal->rotation_angle,//double final_position,
                    goal->rotation_start_time,                            
                    goal->rotation_initial_velocity, 
                    goal->rotation_final_velocity,
                    goal->rotation_initial_acceleration, 
                    goal->rotation_final_acceleration
                    )
            )
    );

    executeMoveCartesianGeneralCB( 
                                cart_traj,
                                goal->start_delay,
                                goal->steady_state_thr,
                                boost::bind(&Robot_AS::moveCircumferenceSuccess, this),
                                boost::bind(&Robot_AS::moveCircumferencePublishFeedback, this, _1, _2,_3,_4, _5),
                                boost::bind(&Robot_AS::moveCircumferenceIsPreemptRequested, this),
                                boost::bind(&Robot_AS::moveCircumferencePreempted, this),
                                boost::bind(&Robot_AS::moveCircumferenceAbort, this, _1) 
                                );

}

/*
    CB that execute the joints trajectory
*/
void Robot_AS::executeMoveCartesianGeneralCB( 
                                Cartesian_Traj_Interface& traj,
                                double start_delay,
                                double steady_state_thr,
                                const boost::function< void() >& successFcn,
                                const boost::function< void(double, const Vector<3>&,const UnitQuaternion&,const Vector<3>&,const Vector<3>&) >& publishFeedbackFcn,
                                const boost::function< bool() >& isPreemptRequestedFcn,
                                const boost::function< void() >& preemptedFcn,
                                const boost::function< void(const string&) >& abortFcn 
                                ){

    //SET CLIK MODE JOINTS
    if( !clikSetMode(sun_robot_msgs::ClikSetMode::Request::MODE_CLIK) ){
        cout << HEADER_PRINT BOLDRED << _clik_set_mode_error_str << CRESET << endl;
        abortFcn(_clik_set_mode_error_str);
        releaseResource();
        return;
    }

    ros::Rate loop_rate(_hz);
    double time_now = ros::Time::now().toSec();

    traj.changeInitialTime( time_now + start_delay );
    cout << HEADER_PRINT GREEN "Start Cartesian Trajectory..." CRESET << endl;
    while( ros::ok() && !_b_action_preempted && !traj.isCompleate(time_now) ){

        ros::spinOnce();

        //check preempt
        if (isPreemptRequestedFcn()){
            _b_action_preempted = true;
            break;
        }

        time_now = ros::Time::now().toSec();

        Vector<3> pos = traj.getPosition(  time_now );
        UnitQuaternion quaternion = traj.getQuaternion(  time_now );
        Vector<3> vel = traj.getLinearVelocity(  time_now );
        Vector<3> w = traj.getAngularVelocity(  time_now );
        
        
        publishCartesian(pos,quaternion,vel,w);

        publishFeedbackFcn(traj.getTimeLeft(time_now),pos,quaternion,vel,w);

        loop_rate.sleep();

    }

    //whait steady state
    if( steady_state_thr!=0 && !_b_action_preempted ){
        cout << HEADER_PRINT YELLOW "Cartesian Trajectory compleate, waiting for steady state..." CRESET << endl;

        double error_steady = INFINITY;

        Vector<3> pos_final = traj.getPosition(  time_now );
        UnitQuaternion quaternion_final = traj.getQuaternion(  time_now );
        Vector<3> vel_final = traj.getLinearVelocity(  time_now );
        Vector<3> w_final = traj.getAngularVelocity(  time_now );

        while(ros::ok() && !_b_action_preempted && fabs(error_steady)>steady_state_thr ){

            ros::spinOnce();

            //check preempt
            if (isPreemptRequestedFcn()){
                _b_action_preempted = true;
                break;
            }

            if( !updateActualStatus(true) ){
                cout << HEADER_PRINT BOLDRED "Failed to update Status" CRESET << endl;
                abortFcn("Failed to update Status");
                releaseResource();
                return;
            }

            error_steady = norm( _actual_clik_error );

            publishCartesian(pos_final,quaternion_final,vel_final,w_final);

            publishFeedbackFcn(traj.getTimeLeft(time_now),pos_final,quaternion_final,vel_final,w_final);

            loop_rate.sleep();

        }

    }

    if(_b_action_preempted){
        cout << HEADER_PRINT BOLDYELLOW "Cartesian Trajectory PREEMPTED!" CRESET << endl;
        preemptedFcn();
    } else{
        cout << HEADER_PRINT BOLDGREEN "Cartesian Trajectory Compleate!" CRESET << endl;
        successFcn();
    }

    releaseResource();

}

void Robot_AS::publishCartesian(const Vector<3>& pos, const UnitQuaternion& quat, const Vector<3>& vel, const Vector<3>& w){

    sun_robot_msgs::PoseTwistStamped out_msg;

    out_msg.pose.position.x = pos[0];
    out_msg.pose.position.y = pos[1];
    out_msg.pose.position.z = pos[2];

    out_msg.pose.orientation.w = quat.getS(); 
    Vector<3> quat_v = quat.getV();
    out_msg.pose.orientation.x = quat_v[0];
    out_msg.pose.orientation.y = quat_v[1];
    out_msg.pose.orientation.z = quat_v[2];
                
    out_msg.twist.linear.x = vel[0];
    out_msg.twist.linear.y = vel[1];
    out_msg.twist.linear.z = vel[2];

    out_msg.twist.angular.x = w[0];
    out_msg.twist.angular.y = w[1];
    out_msg.twist.angular.z = w[2];

    out_msg.header.frame_id = "robot_as_cartesian_trajectory";
    out_msg.header.stamp = ros::Time::now();

    _pub_cartesian.publish(out_msg);

}


/*************************************
    END Action MoveCartesian
***************************************/