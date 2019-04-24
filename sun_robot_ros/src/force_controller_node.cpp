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
#include "sun_robot_msgs/ClikSetMode.h"
#include "UnitQuaternion.h"
#include "TF_MIMO/TF_MIMO_DIAGONAL.h"
#include "TF_SISO/TF_INTEGRATOR.h"

TooN::Vector<> sqrt(const TooN::Vector<>& v_in){
    TooN::Vector<> out = v_in;
    for( int i=0; i<v_in.size();i++ ){
        out[i] = sqrt(v_in[i]);
    }
    return out;
}

TooN::Vector<3> sign(const TooN::Vector<3>& v_in){
    TooN::Vector<3> out = v_in;
    for( int i=0; i<v_in.size();i++ ){
        if(v_in[i] > 0.0)
            out[i] = 1.0;
        else if(v_in[i]<0.0)
            out[i] = -1.0;
        else
            out[i] = 0.0;
    }
    return out;
}

TooN::Vector<> mtimes_ew( const TooN::Vector<>& v1, const TooN::Vector<>& v2 ){
    TooN::Vector<> out = TooN::Zeros(v1.size());
    for( int i=0; i<v1.size(); i++ ){
        out[i] = v1[i]*v2[i];
    }
    return out;
}

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
ros::ServiceClient set_click_mode_sc;
ros::ServiceClient get_click_status_sc;

/**********************/
/* Functions          */
/**********************/

void updateClikStatus(sun_robot_msgs::ClikStatus::Response& res, bool b_refresh_clik = false){
    sun_robot_msgs::ClikStatus::Request status_req;
    status_req.refresh = b_refresh_clik;

    bool b_result = get_click_status_sc.call(status_req,res);
    res.success = b_result && res.success;

    if(!res.success){
        cout << HEADER_PRINT BOLDRED "Failed to update clik status" CRESET << endl;
        return;
    }

    pos_out[0] = res.pose.position.x;
    pos_out[1] = res.pose.position.y;
    pos_out[2] = res.pose.position.z;

    quat_out = UnitQuaternion(
        res.pose.orientation.w, 
        makeVector(
            res.pose.orientation.x,
            res.pose.orientation.y,
            res.pose.orientation.z
        )
    );

    pos_in = pos_out - Delta_pos;
    quat_in = quat_out / Delta_quat;

}

void updateClikStatus(bool b_refresh_clik = false){
    sun_robot_msgs::ClikStatus::Response res;
    updateClikStatus(res, b_refresh_clik);
}

void resetController(bool b_refresh_clik = false){
    cout << HEADER_PRINT "Reset..." << endl;
    controller_tf.reset();
    Delta_pos = Zeros;
    Delta_quat = UnitQuaternion();
    updateClikStatus(b_refresh_clik);
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

void stopForceControl(){
    mode = sun_robot_msgs::ForceControllerStatus::Response::MODE_STOP;
    resetController();
}

bool isClikActive(){

    sun_robot_msgs::ClikStatus msg_status;
    msg_status.request.refresh = false;

    bool b_result = get_click_status_sc.call(msg_status);
    b_result = b_result && msg_status.response.success;

    if(!b_result){
        cout << HEADER_PRINT BOLDRED "Failed to get clik status" CRESET << endl;
        return false;
    }

    return (msg_status.response.mode == sun_robot_msgs::ClikStatus::Response::MODE_CLIK);

}

/**********************/
/* END Functions      */
/**********************/

/**********************/
/* Service Server CB  */
/**********************/

//Clik Interface
bool clik_set_mode_service_cb(
    sun_robot_msgs::ClikSetMode::Request  &req, 
   	sun_robot_msgs::ClikSetMode::Response &res
)
{
    switch (mode){
        case sun_robot_msgs::ForceControllerStatus::Response::MODE_STOP:{
            bool b_tmp = set_click_mode_sc.call(req,res);
            updateClikStatus();
            return b_tmp;
        }
        case sun_robot_msgs::ForceControllerStatus::Response::MODE_FORCE_CONTROL:{
            
            switch (req.mode){
                case sun_robot_msgs::ClikSetMode::Request::MODE_STOP:{
                    cout << HEADER_PRINT YELLOW "Request clik_mode Stop... But Force controller is active..." << CRESET << endl;
                    if( !isClikActive() ){
                        cout << HEADER_PRINT RED "Clik is NOT active... STOPPING THE CONTROLLER..." CRESET << endl;
                        stopForceControl();
                        bool b_tmp = set_click_mode_sc.call(req,res);
                        updateClikStatus();
                        return b_tmp;
                    } else{
                        cout << HEADER_PRINT YELLOW "Also clik is active... do nothing.." << CRESET << endl;
                        res.success = true;
                    }
                    break;
                }
                case sun_robot_msgs::ClikSetMode::Request::MODE_CLIK:{
                    cout << HEADER_PRINT YELLOW "Request clik_mode CLIK... But Force controller is active... I have to check..." CRESET << endl;
                    if( isClikActive() ){
                        cout << HEADER_PRINT YELLOW "Clik is already active... do nothing" CRESET << endl;
                        res.success = true;
                    } else{
                        cout << HEADER_PRINT BOLDRED "Clik is not active... But Force Control is Active... stopping the controller..." CRESET << endl;
                        stopForceControl();
                        res.success = false;
                    }
                    break;
                }
                case sun_robot_msgs::ClikSetMode::Request::MODE_JOINT:{
                    cout << HEADER_PRINT YELLOW "Request clik_mode JOINT...But Force controller is active... Stopping the controller..." CRESET << endl;
                    stopForceControl();
                    bool b_tmp = set_click_mode_sc.call(req,res);
                    updateClikStatus();
                    return b_tmp;
                }
                default:{
                    cout << HEADER_PRINT RED "Error in setMode(): Invalid mode in request!" CRESET << endl;
                    res.success = false;
                }
            }

            break;
        }
        default:{
            /* Default Code */
            cout << HEADER_PRINT BOLDRED "ERROR! INVALID MODE IN clikSetMode" CRESET << endl;
            res.success = false;
        }
    }
    return true;
}

//Clik Interface
bool clik_get_status_service_cb(
    sun_robot_msgs::ClikStatus::Request  &req, 
   	sun_robot_msgs::ClikStatus::Response &res)
{

    switch (mode){
        case sun_robot_msgs::ForceControllerStatus::Response::MODE_STOP:{
            updateClikStatus(res, req.refresh);
            return res.success;
        }
        case sun_robot_msgs::ForceControllerStatus::Response::MODE_FORCE_CONTROL:{

            if( !isClikActive() ){
                cout << HEADER_PRINT BOLDRED "ERROR: getClikStatus... ForceControl is active but CLIK not... stopping the controller!" CRESET << endl;
                stopForceControl();
                res.success = false;
                return false;
            } else {
                
                cout << HEADER_PRINT YELLOW "getClikStatus... Force Control is active!" << CRESET << endl;
                updateClikStatus(res, req.refresh);
                if(!res.success){
                    cout << HEADER_PRINT BOLDRED "ERROR: failed to update clik status..." CRESET << endl;
                    stopForceControl();
                    return true;
                }

                res.pose.position.x = pos_in[0];
                res.pose.position.y = pos_in[1];
                res.pose.position.z = pos_in[2];

                res.pose.orientation.w = quat_in.getS();
                Vector<3> quat_in_v = quat_in.getV();
                res.pose.orientation.x = quat_in_v[0];
                res.pose.orientation.y = quat_in_v[1];
                res.pose.orientation.z = quat_in_v[2];

            }

            break;
        }
        default:{
            /* Default Code */
            cout << HEADER_PRINT BOLDRED "ERROR! INVALID MODE IN clikGetStatus" CRESET << endl;
            res.success = false;
        }
    }

    return true;

}

bool fc_set_mode_service_cb(
    sun_robot_msgs::ForceControllerSetMode::Request  &req, 
   	sun_robot_msgs::ForceControllerSetMode::Response &res
)
{

    switch (req.mode)
    {
        case sun_robot_msgs::ForceControllerSetMode::Request::MODE_STOP:{
            stopForceControl();
            break;
        }
        case sun_robot_msgs::ForceControllerSetMode::Request::MODE_FORCE_CONTROL:{
            if(!isClikActive()){
                cout << HEADER_PRINT BOLDRED "Error in setMode()... cannot set mode force control if CLIK is not active!" CRESET << endl;
                res.success = false;
                return false;
            }
            break;
        }
        default:{
            cout << HEADER_PRINT BOLDRED "Invalid mode in setMode()" CRESET << endl;
            res.success = false;
            return true;
        }
    }

    if(mode!=req.mode){
        cout << HEADER_PRINT "Changing mode to " << (int)req.mode << endl;
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

bool fc_get_status_service_cb(
    sun_robot_msgs::ForceControllerStatus::Request  &req, 
   	sun_robot_msgs::ForceControllerStatus::Response &res
)
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

/**************************/
/* end Service Server CB  */
/**************************/

/**********************/
/* TOPICs CBs         */
/**********************/

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

/**********************/
/* end TOPICs CBs     */
/**********************/

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
    string service_force_control_get_status_str;
    nh_private.param("service_force_control_get_status" , service_force_control_get_status_str, string("force_control/get_status") );
    string service_force_control_set_mode_str;
    nh_private.param("service_force_control_set_mode" , service_force_control_set_mode_str, string("force_control/set_mode") );
    string service_clik_get_status_str;
    nh_private.param("service_clik_get_status" , service_clik_get_status_str, string("clik/get_status") );
    string service_clik_set_mode_str;
    nh_private.param("service_clik_set_mode" , service_clik_set_mode_str, string("clik/set_mode") );
    string service_force_control_get_clik_status_str;
    nh_private.param("service_force_control_get_clik_status" , service_force_control_get_clik_status_str, string("force_control/get_clik_status") );
    string service_force_control_set_clik_mode_str;
    nh_private.param("service_force_control_set_clik_mode" , service_force_control_set_clik_mode_str, string("force_control/set_clik_mode") );

    //control params
    int controlled_index;
    nh_private.param("controlled_index" , controlled_index, 3 );

    bool b_use_integrator;
    nh_private.param("use_integrator" , b_use_integrator, false );

    cout << "INTEGRATOR " << ( b_use_integrator ? "YES" : "NO" ) << endl;

    double integrator_gain;
    nh_private.param("integrator_gain" , integrator_gain, 0.0 );

    double p_gain;
    nh_private.param("p_gain" , p_gain, 0.0 );

    double stiff_2;
    if(nh_private.hasParam("stiff_2")){
        nh_private.param("stiff_2" , stiff_2, 1.0 );
    } else if(b_use_integrator) {
        cout << HEADER_PRINT BOLDRED "param stiff_2 not provided!" CRESET << endl;
        exit(-1);
    }
    

    double controller_gain;
    nh_private.param("controller_gain" , controller_gain, 1.0 );

    if(b_use_integrator)
        controller_gain = 0.0;

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
    Vector<> a_vect = -den_coeff.slice(1,den_coeff.size()-1)/den_coeff[0]; 
    num_coeff = num_coeff/den_coeff[0]; 


    //Subs
    ros::Subscriber subMeasuredWrench = nh_public.subscribe(topic_measured_wrench_str, 1, readMeasuredWrench);
    ros::Subscriber subDesiredWrench = nh_public.subscribe(topic_desired_wrench_str, 1, readDesiredWrench);
    ros::Subscriber subPoseTwist_in = nh_public.subscribe(topic_pose_twist_in_str, 1, readPoseTwist_in);

    //Service Servers
    ros::ServiceServer serviceForceControlGetStatus = nh_public.advertiseService( service_force_control_get_status_str, fc_get_status_service_cb);
    ros::ServiceServer serviceForceControlSetMode = nh_public.advertiseService( service_force_control_set_mode_str, fc_set_mode_service_cb);
    ros::ServiceServer serviceForceControlGetClikStatus = nh_public.advertiseService( service_force_control_get_clik_status_str, clik_get_status_service_cb);
    ros::ServiceServer serviceForceControlSetClikMode = nh_public.advertiseService( service_force_control_set_clik_mode_str, clik_set_mode_service_cb);
    
    //Service clients
    get_click_status_sc = nh_public.serviceClient<sun_robot_msgs::ClikStatus>(service_clik_get_status_str);
    set_click_mode_sc = nh_public.serviceClient<sun_robot_msgs::ClikSetMode>(service_clik_set_mode_str);

    //Pubs
    pubPoseTwist_out = nh_public.advertise<sun_robot_msgs::PoseTwistStamped>(topic_pose_twist_out_str, 1);
    
    //wait for servers
    get_click_status_sc.waitForExistence();
    set_click_mode_sc.waitForExistence();

    //Build controller
    cout << HEADER_PRINT "Building the controller [controlled_index=" << controlled_index <<"]..." << endl; 
    {
        std::vector<TF_SISO_Ptr> siso_diag_vect;
        for(int i=0; i<DIM_MIMO; i++){
            if(i == controlled_index){
                if(b_use_integrator){
                    siso_diag_vect.push_back(
                        TF_INTEGRATOR_Ptr(
                            new TF_INTEGRATOR(
                                1.0/hz,integrator_gain
                            )
                        )
                    );
                }else{
                    siso_diag_vect.push_back(
                                    TF_SISO_Ptr(
                                        new TF_SISO(num_coeff, a_vect, 1.0/hz)
                                    )
                    );
                }
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
                Delta_quat = UnitQuaternion();

                break;
            }
            case sun_robot_msgs::ForceControllerSetMode::Request::MODE_FORCE_CONTROL:{

                if( b_use_integrator ){
                    Vector<3> err = wrench_d.slice<0,3>() - wrench_m.slice<0,3>();
                    Delta_pos = controller_tf.apply( err ) + p_gain * err ;
                    //Delta_pos = Delta_pos/3.2251920750065783e+03;
                    Delta_pos = mtimes_ew(sqrt( abs(Delta_pos)/stiff_2 ) , sign(Delta_pos) );
                }
                else {
                Delta_pos = controller_gain*controller_tf.apply( wrench_d.slice<0,3>() - wrench_m.slice<0,3>() );
                }

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