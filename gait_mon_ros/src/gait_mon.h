#ifndef GAIT_MON_H
#define GAIT_MON_H


struct ImuData
{
    double timestamp;
    double qw,qx,qy,qz;
    double ax,ay,az;
    double qw_prev,qx_prev,qy_prev,qz_prev;
    double ax_prev,ay_prev,az_prev;

    int prev_foot_status;

    double qw_init, qx_init, qy_init, qz_init;
    double ax_init, ay_init, az_init;

    int init_done_flag;
    int init_process_flag;
    int init_very_first_timestamp;
    int init_data_cnt;

    int foot_hit_desicion_cnt;
    int foot_leave_desicion_cnt;

};

struct GaitData
{
    double left_knee_extention_angle;  // degree
    double right_knee_extention_angle;  // degree

    double left_knee_extention_angle_prev;  // degree
    double right_knee_extention_angle_prev;  // degree
};

#endif // GAIT_MON_H