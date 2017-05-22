#include <stdio.h> 
#include <math.h> 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"

#include "gait_mon.h"

#define INITIAL_DURATION 10 // sec
#define STEP_LEFT_THRESHOLD 0.2
#define STEP_TOUCH_THRESHOLD 0.05

#define LOW_PASS_ACC_THRESHOLD 2.0
#define LOW_PASS_QUAD_THRESHOLD 0.3
#define KNEE_EXT_THRESHOLD 30 // degree

ImuData imu_on_left_foot;
ImuData imu_on_right_foot; 
ImuData imu_on_left_calf;
ImuData imu_on_left_thigh;
ImuData imu_on_right_calf;
ImuData imu_on_right_thigh;

GaitData gait_param;

int testCnt = 0;
ImuData imu_test_data_calf[100];
ImuData imu_test_data_thigh[100];

ros::Publisher left_knee_ext;
ros::Publisher right_knee_ext;

ros::Publisher left_foot_hit;
ros::Publisher right_foot_hit;

ros::Publisher left_foot_acc;
ros::Publisher right_foot_acc;

FILE *fd_left_foot_hit;
FILE *fd_right_foot_hit;
FILE *fd_left_foot_acc;
FILE *fd_right_foot_acc;
FILE *fd_left_knee_ext;
FILE *fd_right_knee_ext;
char tmpResult[2000];

int init_process(ImuData *data, int id)
{
    if(data == NULL) {
        ROS_INFO("wrong : NULL!!!");
        return 0;
    }
    
    if(data->init_done_flag == 1) {
        //ROS_INFO("init_process is done!!!");
        return 1;
    }

    if(data->init_process_flag == 0) {
        ROS_INFO("init_process begins timestamp %f !!!", data->timestamp);
        data->init_very_first_timestamp = data->timestamp;
        data->init_process_flag = 1;
    }

    if(data->init_process_flag && (data->timestamp - data->init_very_first_timestamp)/1000 > INITIAL_DURATION){
        // get initial average values
        data->ax_init = data->ax_init /data->init_data_cnt;
        data->ay_init = data->ay_init /data->init_data_cnt;
        data->az_init = data->az_init /data->init_data_cnt;
        data->qx_init = data->qx_init /data->init_data_cnt;
        data->qy_init = data->qy_init /data->init_data_cnt;
        data->qz_init = data->qz_init /data->init_data_cnt;
        data->qw_init = data->qw_init /data->init_data_cnt;
        data->init_done_flag = 1;
        ROS_INFO("Initial duration is over : init_data_cnt %d", data->init_data_cnt);
        ROS_INFO("Imu init ax: [%f], ay: [%f], az: [%f]", data->ax_init, data->ay_init, data->az_init);
        ROS_INFO("Imu init qw: [%f], ax: [%f], ay: [%f], az: [%f]", data->qw_init, data->qx_init, data->qy_init, data->qz_init);
        return 1;
    }

    // accumulate values
    data->ax_init += data->ax;
    data->ay_init += data->ay;
    data->az_init += data->az;
    data->qx_init += data->qx;
    data->qy_init += data->qy;
    data->qz_init += data->qz;
    data->qw_init += data->qw;

    // store previous values
    data->ax_prev = data->ax;
    data->ay_prev = data->ay;
    data->az_prev = data->az;
    data->qx_prev = data->qx;
    data->qy_prev = data->qy;
    data->qz_prev = data->qz;
    data->qw_prev = data->qw;

    ROS_INFO("Initial duration");
    ROS_INFO("%d Imu ax: [%f], ay: [%f], az: [%f]", id, data->ax, data->ay, data->az);
    ROS_INFO("Imu qw: [%f], ax: [%f], ay: [%f], az: [%f]", data->qw, data->qx, data->qy, data->qz);

    data->init_data_cnt++;
    return 0;
}


int get_footStatus(ImuData *data, int leg_type)
{
    if(data == NULL) {
        ROS_INFO("wrong : NULL!!!");
        return 0;
    }
    
    if(data->init_done_flag == 0) {
        ROS_INFO("init_process is NOT done!!!");
        return 0;
    }

    double sumAcc = 0, acc_mag=0;
    double sumAcc_ref = 0;
    double sumAcc_diff = 0;
    acc_mag = sqrt(data->ax*data->ax + data->ay*data->ay + data->ay*data->ay);
    if (acc_mag < 0)
        acc_mag = -acc_mag;

    std_msgs::Float64 msg;
    msg.data = acc_mag;

    memset(tmpResult, 0x00, sizeof(tmpResult));
    sprintf(tmpResult, "%f,%f\n", data->timestamp, acc_mag);

    if (leg_type == 0 ) { // left leg
        left_foot_acc.publish(msg);
        fwrite(tmpResult, strlen(tmpResult),1,fd_left_foot_acc);
    }
    else {
        right_foot_acc.publish(msg);
        fwrite(tmpResult, strlen(tmpResult),1,fd_right_foot_acc);
    }
        
    //ROS_INFO("Imu ax: [%f], ay: [%f], az: [%f]", data->ax, data->ay, data->az);
    //ROS_INFO("acc_mag %f ", acc_mag);

    if(acc_mag > STEP_LEFT_THRESHOLD) {
        //ROS_INFO("Foot is moving!!!");
        return 1;
    }
    else if (acc_mag <= STEP_TOUCH_THRESHOLD) {
        //ROS_INFO("Foot is not moving!!!");
        return 0;
    }

#if 0
    sumAcc = data->ax; //data->ax + data->ay + data->az;
    sumAcc_ref = data->ax_init; //data->ax_init + data->ay_init + data->az_init;
    sumAcc_diff = sumAcc - sumAcc_ref;
    if (sumAcc_diff < 0)
        sumAcc_diff = -sumAcc_diff;
    //ROS_INFO("Imu init ax: [%f], ay: [%f], az: [%f]", data->ax, data->ay, data->az);
    //ROS_INFO("sumAcc_ref %f sumAcc %f,  diff %f", sumAcc_ref, sumAcc, sumAcc_diff);
    if(sumAcc_diff > STEP_DETECT_THRESHOLD) {
        //ROS_INFO("Foot is moving!!!");
        return 1;
    }
    else {
        //ROS_INFO("Foot is not moving!!!");
        return 0;
    }
#endif
    return -1;
}


int step_detection(ImuData *data, int leg_type)
{
    if(data == NULL) {
        ROS_INFO("wrong : NULL!!!");
        return 0;
    }
    
    if(data->init_done_flag == 0) {
        ROS_INFO("init_process is NOT done!!!");
        return 0;
    }

    int cur_foot_status = get_footStatus(data, leg_type); 
    // 1 : in the air, 0 : on the ground, -1 : not sure

    if(cur_foot_status == 1) {
        // foot is moving
        data->foot_hit_desicion_cnt = 0;
    }
    else {
        // foot is NOT moving
        //data->foot_leave_desicion_cnt = 0;
    }
    
    if(data->prev_foot_status == 1 && cur_foot_status == 0) {
        ROS_INFO("Foot heat the ground!!!");
        data->foot_hit_desicion_cnt++;
        if(data->foot_hit_desicion_cnt > 0) {
            data->prev_foot_status = cur_foot_status;
            data->foot_hit_desicion_cnt = 0;
            return 1;
        }
    }
    else if(data->prev_foot_status == 0 && cur_foot_status == 1) {
        ROS_INFO("Foot left the ground!!!");
        data->foot_leave_desicion_cnt++;
        if(data->foot_leave_desicion_cnt > 1) {
            data->prev_foot_status = cur_foot_status;
            data->foot_leave_desicion_cnt = 0;
        }
    }
    
    return 0;
}


void get_knee_extension(int leg_type)
{
    ImuData imu_data1, imu_data2;
    std_msgs::Int16 msg;

    
    if (leg_type == 0 ) { // left leg
        imu_data1 = imu_on_left_calf;
        imu_data2 = imu_on_left_thigh;
    }
    else {
        imu_data1 = imu_on_right_calf;
        imu_data2 = imu_on_right_thigh;
    }

    // convert to Eigen quaternion structure
    Eigen::Quaterniond quat_on_calf(imu_data1.qw, imu_data1.qx, imu_data1.qy, imu_data1.qz);
    Eigen::Quaterniond quat_on_thigh(imu_data2.qw, imu_data2.qx, imu_data2.qy, imu_data2.qz);

    // get rotation
    Eigen::Matrix3d ori_on_calf = quat_on_calf.toRotationMatrix();
    Eigen::Matrix3d ori_on_thigh = quat_on_thigh.toRotationMatrix();

    // project local quat vector to x-axis in global-space coordiante
    Eigen::Vector3d global_x_axis_on_calf(1,0,0);
    Eigen::Vector3d global_x_axis_on_thigh(1,0,0);
    global_x_axis_on_calf = ori_on_calf*global_x_axis_on_calf;
    global_x_axis_on_thigh = ori_on_thigh*global_x_axis_on_thigh;
    //printf("IMU on calf %f %f %f\n", global_x_axis_on_calf[0], global_x_axis_on_calf[1], global_x_axis_on_calf[2]);
    //printf("IMU on thigh %f %f %f\n", global_x_axis_on_thigh[0], global_x_axis_on_thigh[1], global_x_axis_on_thigh[2]);

    // calculate angle between two vectors
    double dot_product = global_x_axis_on_calf.transpose()*global_x_axis_on_thigh;
    double knee_extention_angle = acos ( dot_product  ) * 180.0 / M_PI;


    if (leg_type == 0 ) { // left leg
        //ROS_INFO("LEFT Knee extension: [%f] degrees", knee_extention_angle);
        msg.data = (int)knee_extention_angle;
        gait_param.left_knee_extention_angle = knee_extention_angle;

        // check sanity
        double data_diff = fabs (gait_param.left_knee_extention_angle_prev - gait_param.left_knee_extention_angle);
        if(gait_param.left_knee_extention_angle_prev >= 0) {
            if (data_diff >= KNEE_EXT_THRESHOLD) {
                ROS_INFO("LEFT Knee extension too big: [%f] degrees", knee_extention_angle);
                ROS_INFO("Imu thigh qw: [%f], qx: [%f], qy: [%f], qz: [%f]",imu_data2.qw, imu_data2.qx, imu_data2.qy, imu_data2.qz);
                ROS_INFO("Imu thigh qw_prev: [%f], qx_prev: [%f], qy_prev: [%f], qz_prev: [%f]",imu_data2.qw_prev, imu_data2.qx_prev, imu_data2.qy_prev, imu_data2.qz_prev);
                ROS_INFO("Imu calf qw: [%f], qx: [%f], qy: [%f], qz: [%f]", imu_data1.qw, imu_data1.qx, imu_data1.qy, imu_data1.qz);
                ROS_INFO("Imu calf qw_prev: [%f], qx_prev: [%f], qy_prev: [%f], qz_prev: [%f]", imu_data1.qw_prev, imu_data1.qx_prev, imu_data1.qy_prev, imu_data1.qz_prev);
            }
        }
        gait_param.left_knee_extention_angle_prev = gait_param.left_knee_extention_angle;
        
        left_knee_ext.publish(msg);

        memset(tmpResult, 0x00, sizeof(tmpResult));
        sprintf(tmpResult, "%f,%d\n", imu_data1.timestamp, msg.data);
        fwrite(tmpResult, strlen(tmpResult),1,fd_left_knee_ext);
    }
    else {
        //ROS_INFO("RIGHT Knee extension: [%f] degrees", knee_extention_angle);
        msg.data = (int)knee_extention_angle;
        gait_param.right_knee_extention_angle = knee_extention_angle;

        // check sanity
        double data_diff = fabs (gait_param.right_knee_extention_angle_prev - gait_param.right_knee_extention_angle);
        if(gait_param.right_knee_extention_angle_prev >= 0) {
            if (data_diff >= KNEE_EXT_THRESHOLD) {
                ROS_INFO("right Knee extension too big: [%f] degrees", knee_extention_angle);
                ROS_INFO("Imu thigh qw: [%f], qx: [%f], qy: [%f], qz: [%f]",imu_data2.qw, imu_data2.qx, imu_data2.qy, imu_data2.qz);
                ROS_INFO("Imu thigh qw_prev: [%f], qx_prev: [%f], qy_prev: [%f], qz_prev: [%f]",imu_data2.qw_prev, imu_data2.qx_prev, imu_data2.qy_prev, imu_data2.qz_prev);
                ROS_INFO("Imu calf qw: [%f], qx: [%f], qy: [%f], qz: [%f]", imu_data1.qw, imu_data1.qx, imu_data1.qy, imu_data1.qz);
                ROS_INFO("Imu calf qw_prev: [%f], qx_prev: [%f], qy_prev: [%f], qz_prev: [%f]", imu_data1.qw_prev, imu_data1.qx_prev, imu_data1.qy_prev, imu_data1.qz_prev);
            }
        }
        gait_param.right_knee_extention_angle_prev = gait_param.right_knee_extention_angle;
        
        right_knee_ext.publish(msg);

        memset(tmpResult, 0x00, sizeof(tmpResult));
        sprintf(tmpResult, "%f,%d\n", imu_data1.timestamp, msg.data);
        fwrite(tmpResult, strlen(tmpResult),1,fd_right_knee_ext);
    }
    
}


double check_acc_noise(double prev_val, double cur_val)
{
    double data_diff = prev_val - cur_val;
    if(data_diff < 0)
        data_diff = -data_diff;
    if(data_diff > LOW_PASS_ACC_THRESHOLD)
        return data_diff;
    if(cur_val == 0)
        return 12345;
    return 0;
}

double check_quad_noise(double prev_val, double cur_val)
{
    double data_diff = prev_val - cur_val;
    if(data_diff < 0)
        data_diff = -data_diff;
    if(data_diff > LOW_PASS_QUAD_THRESHOLD)
        return data_diff;
    if(cur_val == 0)
        return 12345;
    return 0;
}


int filter_imu_data(ImuData *data, int id)
{
    double data_diff = 0.0;

    if(data == NULL) {
        ROS_INFO("wrong : NULL!!!");
        return 0;
    }
    
    if(data->init_process_flag == 0) {
        ROS_INFO("init_process NOT begins !!!");
        return 0;
    }

    data_diff = check_acc_noise(data->ax_prev, data->ax);
    if(data_diff) {
        ROS_INFO("ax Noise id %d : prev %f, cur %f, diff %f", id, data->ax_prev, data->ax, data_diff);
        data->ax = data->ax_prev;
    }
    data_diff = check_acc_noise(data->ay_prev, data->ay);
    if(data_diff) {
        ROS_INFO("ay Noise id %d : prev %f, cur %f, diff %f", id, data->ay_prev, data->ay, data_diff);
        data->ay = data->ay_prev;
    }
    data_diff = check_acc_noise(data->az_prev, data->az);
    if(data_diff) {
        ROS_INFO("az Noise id %d : prev %f, cur %f, diff %f", id, data->az_prev, data->az, data_diff);
        data->az = data->az_prev;
    }
    data_diff = check_quad_noise(data->qw_prev, data->qw);
    if(data_diff) {
        //ROS_INFO("qw Noise id %d : prev %f, cur %f, diff %f", id, data->qw_prev, data->qw, data_diff);
        data->qw = data->qw_prev;
    }
    data_diff = check_quad_noise(data->qx_prev, data->qx);
    if(data_diff) {
        //ROS_INFO("qx Noise id %d : prev %f, cur %f, diff %f", id, data->qx_prev, data->qx, data_diff);
        data->qx = data->qx_prev;
    }
    data_diff = check_quad_noise(data->qy_prev, data->qy);
    if(data_diff) {
        //ROS_INFO("qy Noise id %d : prev %f, cur %f, diff %f", id, data->qy_prev, data->qy, data_diff);
        data->qy = data->qy_prev;
    }
    data_diff = check_quad_noise(data->qz_prev, data->qz);
    if(data_diff) {
        //ROS_INFO("qz Noise id %d : prev %f, cur %f, diff %f", id, data->qz_prev, data->qz, data_diff);
        data->qz = data->qz_prev;
    }

    // store previous values
    data->ax_prev = data->ax;
    data->ay_prev = data->ay;
    data->az_prev = data->az;
    data->qx_prev = data->qx;
    data->qy_prev = data->qy;
    data->qz_prev = data->qz;
    data->qw_prev = data->qw;
}


void imu_1_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_on_left_foot.timestamp = msg->header.stamp.sec*1000 + msg->header.stamp.nsec/1000000; //ms

    imu_on_left_foot.ax = msg->linear_acceleration.x;
    imu_on_left_foot.ay = msg->linear_acceleration.y;
    imu_on_left_foot.az = msg->linear_acceleration.z;
    imu_on_left_foot.qx = msg->orientation.x;
    imu_on_left_foot.qy = msg->orientation.y;
    imu_on_left_foot.qz = msg->orientation.z;
    imu_on_left_foot.qw =msg->orientation.w;

    filter_imu_data(&imu_on_left_foot, 1);

    if(init_process(&imu_on_left_foot, 1) == 0)
        return;

    //ROS_INFO("Imu 1 left foot Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu 1 left foot Acceleration x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);

    std_msgs::Int16 step_msg;
    step_msg.data = 0;
    if(step_detection(&imu_on_left_foot, 0) == 1) {
         ROS_INFO("Imu 1 left foot Touches ground!! ");
         step_msg.data = 100;
    }
    left_foot_hit.publish(step_msg);

    memset(tmpResult, 0x00, sizeof(tmpResult));
    sprintf(tmpResult, "%f,%d\n", imu_on_left_foot.timestamp, step_msg.data);
    fwrite(tmpResult, strlen(tmpResult),1,fd_left_foot_hit);
  
}


void imu_3_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_on_left_calf.timestamp = msg->header.stamp.sec*1000 + msg->header.stamp.nsec/1000000; //ms

    imu_on_left_calf.ax = msg->linear_acceleration.x;
    imu_on_left_calf.ay = msg->linear_acceleration.y;
    imu_on_left_calf.az = msg->linear_acceleration.z;
    imu_on_left_calf.qx = msg->orientation.x;
    imu_on_left_calf.qy = msg->orientation.y;
    imu_on_left_calf.qz = msg->orientation.z;
    imu_on_left_calf.qw =msg->orientation.w;

    filter_imu_data(&imu_on_left_calf, 3);

    if(init_process(&imu_on_left_calf, 3) == 0)
        return;

    //ROS_INFO("Imu 3 left calf Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu 3 left calf Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

    // Calculate knee extension only when both IMUs are ready
    if(imu_on_left_calf.init_done_flag && imu_on_left_thigh.init_done_flag)
        get_knee_extension(0);
}

void imu_5_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_on_left_thigh.timestamp = msg->header.stamp.sec*1000 + msg->header.stamp.nsec/1000000; //ms

    imu_on_left_thigh.ax = msg->linear_acceleration.x;
    imu_on_left_thigh.ay = msg->linear_acceleration.y;
    imu_on_left_thigh.az = msg->linear_acceleration.z;
    imu_on_left_thigh.qx = msg->orientation.x;
    imu_on_left_thigh.qy = msg->orientation.y;
    imu_on_left_thigh.qz = msg->orientation.z;
    imu_on_left_thigh.qw =msg->orientation.w;

    filter_imu_data(&imu_on_left_thigh, 5);

    if(init_process(&imu_on_left_thigh, 5) == 0)
        return;

    //ROS_INFO("Imu 5 left thigh Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu 5 left thigh Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

}

void imu_2_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_on_right_foot.timestamp = msg->header.stamp.sec*1000 + msg->header.stamp.nsec/1000000; //ms

    imu_on_right_foot.ax = msg->linear_acceleration.x;
    imu_on_right_foot.ay = msg->linear_acceleration.y;
    imu_on_right_foot.az = msg->linear_acceleration.z;
    imu_on_right_foot.qx = msg->orientation.x;
    imu_on_right_foot.qy = msg->orientation.y;
    imu_on_right_foot.qz = msg->orientation.z;
    imu_on_right_foot.qw =msg->orientation.w;

    filter_imu_data(&imu_on_right_foot, 2);

    if(init_process(&imu_on_right_foot, 2) == 0)
        return;

    //ROS_INFO("Imu 2 right foot Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu 2 right foot Acceleration x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);

    std_msgs::Int16 step_msg;
    step_msg.data = 0;
    if(step_detection(&imu_on_right_foot, 1) == 1) {
        ROS_INFO("Imu 2 right foot Touches ground!! ");
        step_msg.data = 100;
    }
    right_foot_hit.publish(step_msg);

    memset(tmpResult, 0x00, sizeof(tmpResult));
    sprintf(tmpResult, "%f,%d\n", imu_on_right_foot.timestamp, step_msg.data);
    fwrite(tmpResult, strlen(tmpResult),1,fd_right_foot_hit);
    
}

void imu_4_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_on_right_calf.timestamp = msg->header.stamp.sec*1000 + msg->header.stamp.nsec/1000000; //ms

    imu_on_right_calf.ax = msg->linear_acceleration.x;
    imu_on_right_calf.ay = msg->linear_acceleration.y;
    imu_on_right_calf.az = msg->linear_acceleration.z;
    imu_on_right_calf.qx = msg->orientation.x;
    imu_on_right_calf.qy = msg->orientation.y;
    imu_on_right_calf.qz = msg->orientation.z;
    imu_on_right_calf.qw =msg->orientation.w;

    filter_imu_data(&imu_on_right_calf, 4);

    if(init_process(&imu_on_right_calf, 4) == 0)
        return;

    //ROS_INFO("Imu 4 right calf Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu 4 right calf Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

     // Calculate knee extension only when both IMUs are ready
    if(imu_on_right_calf.init_done_flag && imu_on_right_thigh.init_done_flag)
        get_knee_extension(1);

}

void imu_6_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_on_right_thigh.timestamp = msg->header.stamp.sec*1000 + msg->header.stamp.nsec/1000000; //ms

    imu_on_right_thigh.ax = msg->linear_acceleration.x;
    imu_on_right_thigh.ay = msg->linear_acceleration.y;
    imu_on_right_thigh.az = msg->linear_acceleration.z;
    imu_on_right_thigh.qx = msg->orientation.x;
    imu_on_right_thigh.qy = msg->orientation.y;
    imu_on_right_thigh.qz = msg->orientation.z;
    imu_on_right_thigh.qw =msg->orientation.w;

    filter_imu_data(&imu_on_right_thigh, 6);

    if(init_process(&imu_on_right_thigh, 6) == 0)
        return;

    //ROS_INFO("Imu 6 right thigh Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu 6 right thigh Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

}


void shutdown_SigintHandler(int sig)
{

    ROS_INFO("Shut Down Gait MON");
    
    fclose(fd_left_foot_acc);
    fclose(fd_right_foot_acc);
    fclose(fd_left_foot_hit);
    fclose(fd_right_foot_hit);
    fclose(fd_left_knee_ext);
    fclose(fd_right_knee_ext);
    
    ros::shutdown();
}


int main(int argc, char **argv)
{
    //ros::init(argc, argv, "gait_mon");
    ros::init(argc, argv, "gait_mon", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    // test
    // 0.0417, 0.99, 0.15, 0.07 // 0.0411, 1.01, -0.04, -0.01  // 11.69289299
    // -0.0418, 0.93, 0.39, -0.04 // 0.4294, 0.24, -0.96, 0    // 26.82324647
    // 0.003, 0.98, 0.23, 0.01  // 0.0013, 1, 0.05, -0.09  // 92.96865052
    //imu_test_data_calf[0].qw = 0.0417, imu_test_data_calf[0].qx = 0.99, imu_test_data_calf[0].qy = 0.15, imu_test_data_calf[0].qz = 0.07;
    //imu_test_data_thigh[0].qw =  0.0411, imu_test_data_thigh[0].qx =  1.01, imu_test_data_thigh[0].qy =  -0.04, imu_test_data_thigh[0].qz =  -0.01;

    //imu_test_data_calf[0].qw = -0.0418, imu_test_data_calf[0].qx = 0.93, imu_test_data_calf[0].qy = 0.39, imu_test_data_calf[0].qz = -0.04;
    //imu_test_data_thigh[0].qw =  0.4294, imu_test_data_thigh[0].qx =  0.24, imu_test_data_thigh[0].qy =  -0.96, imu_test_data_thigh[0].qz = 0;

    //imu_test_data_calf[0].qw = 0.003, imu_test_data_calf[0].qx = 0.98, imu_test_data_calf[0].qy = 0.23, imu_test_data_calf[0].qz = 0.01;
    //imu_test_data_thigh[0].qw =  0.0013, imu_test_data_thigh[0].qx =  1, imu_test_data_thigh[0].qy =  0.05, imu_test_data_thigh[0].qz = -0.09;

    // init values
    memset(&imu_on_right_foot, 0, sizeof(ImuData));
    memset(&imu_on_right_calf, 0, sizeof(ImuData));
    memset(&imu_on_right_thigh, 0, sizeof(ImuData));
    memset(&imu_on_left_foot, 0, sizeof(ImuData));
    memset(&imu_on_left_calf, 0, sizeof(ImuData));
    memset(&imu_on_left_thigh, 0, sizeof(ImuData));
    memset(&gait_param, 0, sizeof(GaitData));
    gait_param.left_knee_extention_angle_prev = -1;
    gait_param.right_knee_extention_angle_prev = -1;
    

    // publisher
    left_knee_ext = n.advertise<std_msgs::Int16>("left_knee_ext", 1000);
    right_knee_ext = n.advertise<std_msgs::Int16>("right_knee_ext", 1000);
    left_foot_hit = n.advertise<std_msgs::Int16>("left_foot_hit", 1000);
    right_foot_hit = n.advertise<std_msgs::Int16>("right_foot_hit", 1000);
    left_foot_acc= n.advertise<std_msgs::Float64>("left_foot_acc", 1000);
    right_foot_acc= n.advertise<std_msgs::Float64>("right_foot_acc", 1000);

    // result files
    if ((fd_left_foot_hit= fopen("left_foot_hit.csv", "w")) == NULL) {
        printf("can't open file \"%s\"\n", "left_foot_hit.csv");
        return 0;
    }
    if ((fd_right_foot_hit= fopen("right_foot_hit.csv", "w")) == NULL) {
        printf("can't open file \"%s\"\n", "right_foot_hit.csv");
        return 0;
    }
    if ((fd_left_foot_acc= fopen("left_foot_acc.csv", "w")) == NULL) {
        printf("can't open file \"%s\"\n", "left_foot_acc.csv");
        return 0;
    }
    if ((fd_right_foot_acc= fopen("right_foot_acc.csv", "w")) == NULL) {
        printf("can't open file \"%s\"\n", "right_foot_acc.csv");
        return 0;
    }
    if ((fd_left_knee_ext= fopen("left_knee_ext.csv", "w")) == NULL) {
        printf("can't open file \"%s\"\n", "left_knee_ext.csv");
        return 0;
    }
    if ((fd_right_knee_ext= fopen("right_knee_ext.csv", "w")) == NULL) {
        printf("can't open file \"%s\"\n", "right_knee_ext.csv");
        return 0;
    }
    
    // left foot IMU
    ros::Subscriber sub1 = n.subscribe("/imu_1", 1000, imu_1_Callback);
    // right foot IMIU
    ros::Subscriber sub2 = n.subscribe("/imu_2", 1000, imu_2_Callback);
    
    // left leg calf IMU
    ros::Subscriber sub3 = n.subscribe("/imu_3", 1000, imu_3_Callback);
    // left leg thigh IMU
    ros::Subscriber sub5 = n.subscribe("/imu_5", 1000, imu_5_Callback);

    // right leg calf IMU
    ros::Subscriber sub4 = n.subscribe("/imu_4", 1000, imu_4_Callback);
    // right leg thigh IMU
    ros::Subscriber sub6 = n.subscribe("/imu_6", 1000, imu_6_Callback);


    signal(SIGINT, shutdown_SigintHandler);

    ros::spin();

    return 0;
}
