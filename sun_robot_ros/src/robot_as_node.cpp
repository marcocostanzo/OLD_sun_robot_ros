#include "sun_robot_ros/Robot_AS.h"

using namespace std;

int main(int argc, char *argv[])
{

    ros::init(argc,argv, "robot_as");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    //params
    int num_joints;
    nh_private.param("num_joints" , num_joints, 0 );
    if(num_joints < 1){
        cout << "ERROR! invalid num_joints: " << num_joints << endl;
        exit(-1);
    }
    double hz;
    nh_private.param("hz" , hz, 1000.0 );
    string move_joint_action_str;
    nh_private.param("action_move_joint" , move_joint_action_str, string("move_joint") );
    string simple_move_joint_action_str;
    nh_private.param("action_simple_move_joint" , simple_move_joint_action_str, string("simple_move_joint") );
    string move_line_segment_action_str;
    nh_private.param("action_line_segment" , move_line_segment_action_str, string("line_segment") );
    string service_clik_status;    
    nh_private.param("service_get_clik_status" , service_clik_status, string("clik/get_status") );
    string service_set_clik_mode;    
    nh_private.param("service_set_clik_mode" , service_set_clik_mode, string("clik/set_mode") );
    string topic_joints_command;    
    nh_private.param("topic_joints_command" , topic_joints_command, string("clik/desired_q") );
    string topic_cartesian_command;  
    nh_private.param("topic_cartesian_command" , topic_cartesian_command, string("clik/desired_pose_twist") );

    Robot_AS robot_as(
        num_joints,
        nh_public,
        hz,
        service_clik_status,
        service_set_clik_mode,
        topic_joints_command,
        topic_cartesian_command,
        move_joint_action_str,
        simple_move_joint_action_str,
        move_line_segment_action_str
        );

    robot_as.start();
    
    ros::spin();

    return 0;
}