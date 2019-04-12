/*

    Robot Force Controller Node

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

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sun_robot_msgs/PoseTwistStamped.h"
#include "sun_robot_msgs/ForceControllerSetMode.h"
#include "sun_robot_msgs/ForceControllerStatus.h"
#include "sun_robot_msgs/ClikStatus.h"
#include "UnitQuaternion.h"
#include "TF_MIMO/TF_MIMO_DIAGONAL.h"

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

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 


using namespace std;
using namespace TooN;

#define DIM_MIMO 3

//Controller Vars
Vector<6> wrench_m;
Vector<6> wrench_d;
Vector<3> pos_in, Delta_pos, pos_out;
UnitQuaternion quat_in, Delta_quat, quat_out;
Vector<3> vel_in;
Vector<3> w_in;
TF_MIMO_DIAGONAL controller_tf(DIM_MIMO);
uint8_t mode = sun_robot_msgs::ForceControllerSetMode::Request::MODE_STOP;

//ROS VARS
ros::Publisher pubPoseTwist_out;
ros::ServiceClient serviceGetClikStatus;

void updatePose_in(bool b_refresh_clik = false){
    sun_robot_msgs::ClikStatus status_msg;

    status_msg.request.refresh = b_refresh_clik;

    bool b_result = serviceGetClikStatus.call(status_msg);
    b_result = b_result && status_msg.response.success;

    if(!b_result){
        cout << HEADER_PRINT BOLDRED "Failed to update initial pose" CRESET << endl;
        exit(-1);
    }

    pos_in[0] = status_msg.response.pose.position.x;
    pos_in[1] = status_msg.response.pose.position.y;
    pos_in[2] = status_msg.response.pose.position.z;

    quat_in = UnitQuaternion(
        status_msg.response.pose.orientation.w, 
        makeVector(
            status_msg.response.pose.orientation.x,
            status_msg.response.pose.orientation.y,
            status_msg.response.pose.orientation.z
        )
    );

    cout << HEADER_PRINT "Initial Position updated: P = " << pos_in << " Q = " << quat_in << endl; 

}

void resetController(){
    cout << HEADER_PRINT "Reset..." << endl;
    controller_tf.reset();
    updatePose_in();
}

void readMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg){

    wrench_m[0] = wrench_msg->wrench.force.x;
    wrench_m[1] = wrench_msg->wrench.force.y;
    wrench_m[2] = wrench_msg->wrench.force.z;
    wrench_m[3] = wrench_msg->wrench.torque.x;
    wrench_m[4] = wrench_msg->wrench.torque.y;
    wrench_m[5] = wrench_msg->wrench.torque.z;

}

void readDesiredWrench(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg){

    wrench_d[0] = wrench_msg->wrench.force.x;
    wrench_d[1] = wrench_msg->wrench.force.y;
    wrench_d[2] = wrench_msg->wrench.force.z;
    wrench_d[3] = wrench_msg->wrench.torque.x;
    wrench_d[4] = wrench_msg->wrench.torque.y;
    wrench_d[5] = wrench_msg->wrench.torque.z;

}

bool serviceSetMode_cb(sun_robot_msgs::ForceControllerSetMode::Request  &req, 
   		 		sun_robot_msgs::ForceControllerSetMode::Response &res)
{

    switch (req.mode)
    {
        case sun_robot_msgs::ForceControllerSetMode::Request::MODE_STOP:{

            break;
        }
        case sun_robot_msgs::ForceControllerSetMode::Request::MODE_FORCE_CONTROL:{

            break;
        }
        default:{
            cout << HEADER_PRINT BOLDRED "Invalid mode in setMode()" CRESET << endl;
            res.success = false;
            return true;
        }
    }

    if(mode!=req.mode){
        cout << HEADER_PRINT "Changing mode to " << req.mode << endl;
        resetController();
        mode = req.mode;
    }
    
    
    wrench_d[0] = req.desired_wrench.force.x;
    wrench_d[1] = req.desired_wrench.force.y;
    wrench_d[2] = req.desired_wrench.force.z;
    wrench_d[3] = req.desired_wrench.torque.x;
    wrench_d[4] = req.desired_wrench.torque.y;
    wrench_d[5] = req.desired_wrench.torque.z;

    res.success = true;
    return true;

}

bool serviceGetStatus_cb(sun_robot_msgs::ForceControllerStatus::Request  &req, 
   		 		sun_robot_msgs::ForceControllerStatus::Response &res)
{

    cout << HEADER_PRINT "Service getStatus" << endl;

    res.mode = mode;

    res.desired_wrench.force.x = wrench_d[0];
    res.desired_wrench.force.y = wrench_d[1];
    res.desired_wrench.force.z = wrench_d[2];
    res.desired_wrench.torque.x = wrench_d[3];
    res.desired_wrench.torque.y = wrench_d[4];
    res.desired_wrench.torque.z = wrench_d[5];

    res.pose_correction.position.x = Delta_pos[0];
    res.pose_correction.position.y = Delta_pos[1];
    res.pose_correction.position.z = Delta_pos[2];
    res.pose_correction.orientation.w = Delta_quat.getS();
    Vector<3> quat_v = Delta_quat.getV();
    res.pose_correction.orientation.x = quat_v[0];
    res.pose_correction.orientation.y = quat_v[1];
    res.pose_correction.orientation.z = quat_v[2];

    res.success = true;
    return true;

}

void readPoseTwist_in(const sun_robot_msgs::PoseTwistStamped::ConstPtr& pose_twist_msg){
    
    pos_in[0] = pose_twist_msg->pose.position.x;
    pos_in[1] = pose_twist_msg->pose.position.y;
    pos_in[2] = pose_twist_msg->pose.position.z;

    quat_in = UnitQuaternion(
                makeVector(
                    pose_twist_msg->pose.orientation.x,
                    pose_twist_msg->pose.orientation.y,
                    pose_twist_msg->pose.orientation.z
                )
                ,
                pose_twist_msg->pose.orientation.w
    );

    vel_in[0] = pose_twist_msg->twist.linear.x;
    vel_in[1] = pose_twist_msg->twist.linear.y;
    vel_in[2] = pose_twist_msg->twist.linear.z;

    w_in[0] = pose_twist_msg->twist.angular.x;
    w_in[1] = pose_twist_msg->twist.angular.y;
    w_in[2] = pose_twist_msg->twist.angular.z;

}

void publishDesiredPoseTwist(){
    
    sun_robot_msgs::PoseTwistStamped out_msg;

    out_msg.pose.position.x = pos_out[0];
    out_msg.pose.position.y = pos_out[1];
    out_msg.pose.position.z = pos_out[2];

    out_msg.pose.orientation.w = quat_out.getS(); 
    Vector<3> quat_v = quat_out.getV();
    out_msg.pose.orientation.x = quat_v[0];
    out_msg.pose.orientation.y = quat_v[1];
    out_msg.pose.orientation.z = quat_v[2];
                
    out_msg.twist.linear.x = vel_in[0];
    out_msg.twist.linear.y = vel_in[1];
    out_msg.twist.linear.z = vel_in[2];

    out_msg.twist.angular.x = w_in[0];
    out_msg.twist.angular.y = w_in[1];
    out_msg.twist.angular.z = w_in[2];

    out_msg.header.frame_id = "force_controller";
    out_msg.header.stamp = ros::Time::now();

    pubPoseTwist_out.publish(out_msg);

}

int main(int argc, char *argv[])
{
    ros::init(argc,argv, "force_controller");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    string topic_measured_wrench_str;
    nh_private.param("topic_measured_wrench" , topic_measured_wrench_str, string("measured_wrench") );
    string topic_desired_wrench_str;
    nh_private.param("topic_desired_wrench" , topic_desired_wrench_str, string("desired_wrench") );
    string topic_pose_twist_in_str;
    nh_private.param("topic_pose_twist_in" , topic_pose_twist_in_str, string("force_control/desired_pose_twist_in") );
    string topic_pose_twist_out_str;
    nh_private.param("topic_pose_twist_out" , topic_pose_twist_out_str, string("clik/desired_pose_twist") );
    string service_get_status_str;
    nh_private.param("service_get_status" , service_get_status_str, string("force_control/get_status") );
    string service_set_mode_str;
    nh_private.param("service_set_mode" , service_set_mode_str, string("force_control/set_mode") );
    string service_clik_status_str;
    nh_private.param("service_get_clik_status" , service_clik_status_str, string("clik/get_status") );

    //control params
    int controlled_index;
    nh_private.param("controlled_index" , controlled_index, 3 );

    double controller_gain;
    nh_private.param("controller_gain" , controller_gain, 1.0 );

    double hz;
    if(nh_private.hasParam("hz")){
        nh_private.param("hz" , hz, 1000.0 );
    } else {
        cout << HEADER_PRINT BOLDRED "param hz not provided!" CRESET << endl;
        exit(-1);
    }

    vector<double> num_coeff_std;
    if(nh_private.hasParam("num_coeff")){
        nh_private.getParam("num_coeff", num_coeff_std);
    } else {
        cout << HEADER_PRINT BOLDRED "param num_coeff not provided!" CRESET << endl;
        exit(-1);
    }
    Vector<> num_coeff = wrapVector(num_coeff_std.data(), num_coeff_std.size());

    vector<double> den_coeff_std;
    if(nh_private.hasParam("den_coeff")){
        nh_private.getParam("den_coeff", den_coeff_std);
    } else {
        cout << HEADER_PRINT BOLDRED "param den_coeff not provided!" CRESET << endl;
        exit(-1);
    }
    Vector<> den_coeff = wrapVector(den_coeff_std.data(), den_coeff_std.size());
    Vector<> a_vect = den_coeff.slice(1,den_coeff.size()-1)/den_coeff[0]; 
    num_coeff = num_coeff/den_coeff[0]; 


    //Subs
    ros::Subscriber subMeasuredWrench = nh_public.subscribe(topic_measured_wrench_str, 1, readMeasuredWrench);
    ros::Subscriber subDesiredWrench = nh_public.subscribe(topic_desired_wrench_str, 1, readDesiredWrench);
    ros::Subscriber subPoseTwist_in = nh_public.subscribe(topic_pose_twist_in_str, 1, readPoseTwist_in);

    //Service Servers
    ros::ServiceServer serviceGetStatus = nh_public.advertiseService( service_get_status_str, serviceGetStatus_cb);
    ros::ServiceServer serviceSetMode = nh_public.advertiseService( service_set_mode_str, serviceSetMode_cb);
    
    //Service clients
    serviceGetClikStatus = nh_public.serviceClient<sun_robot_msgs::ClikStatus>(service_clik_status_str);

    //Pubs
    pubPoseTwist_out = nh_public.advertise<sun_robot_msgs::PoseTwistStamped>(topic_pose_twist_out_str, 1);
    
    //wait for servers
    serviceGetClikStatus.waitForExistence();

    //Build controller
    cout << HEADER_PRINT "Building the controller [controlled_index=" << controlled_index <<"]..." << endl; 
    {
        std::vector<TF_SISO_Ptr> siso_diag_vect;
        for(int i=0; i<DIM_MIMO; i++){
            if(i == controlled_index){
                siso_diag_vect.push_back(
                                TF_SISO_Ptr(
                                    new TF_SISO(num_coeff, a_vect, 1.0/hz)
                                )
                );
            } else{
                //Zero TF
                siso_diag_vect.push_back(
                                TF_SISO_Ptr( 
                                    new TF_SISO(1.0/hz)
                                )
                );
            }
        }

        controller_tf.setSISODiagVect(siso_diag_vect);
    }
    cout << HEADER_PRINT " The controller: " << endl;
    controller_tf.display();
    resetController();
    cout << HEADER_PRINT GREEN "Start!" CRESET << endl;

    ros::Rate loop_rate(hz);
    while (ros::ok())
    {
        ros::spinOnce();

        switch (mode)
        {
            case sun_robot_msgs::ForceControllerSetMode::Request::MODE_STOP:{
                
                Delta_pos = Zeros;

                break;
            }
            case sun_robot_msgs::ForceControllerSetMode::Request::MODE_FORCE_CONTROL:{
                
                Delta_pos = controller_gain*controller_tf.apply( wrench_d.slice<0,3>() - wrench_m.slice<0,3>() );;

                break;
            }
            default:{
                /* Default Code */
                cout << HEADER_PRINT BOLDRED "ERROR! INVALID MODE IN MAIN LOOP" CRESET << endl;
                exit(-1);
            }
        }

        pos_out = pos_in + Delta_pos;
        quat_out = quat_in * Delta_quat; //Here for future implementations...
        
        publishDesiredPoseTwist();        

        loop_rate.sleep();

    }
    

    return 0;
}