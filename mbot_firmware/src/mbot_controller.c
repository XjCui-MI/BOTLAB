#include "mbot_controller.h"

//PID Controller Systems
rc_filter_t mbot_vel_vx_lpf;
rc_filter_t mbot_vx_pid;
rc_filter_t mbot_vy_pid;

//Motor Velocities PID Controller with calibrated PWM.
rc_filter_t left_wheel_pid;
rc_filter_t right_wheel_pid;
rc_filter_t back_wheel_pid;

//Body Velocity PID Controller; Output the desired body angular velocities for the motors.
rc_filter_t mbot_wz_pid;
rc_filter_t mbot_vel_wz_lpf;

//Initialize the RC filters on the 6 PID controllers with given configuration from ctlr_cfg
//only run once
int mbot_init_ctlr (mbot_ctlr_cfg_t ctlr_cfg) {
    left_wheel_pid = rc_filter_empty();
    right_wheel_pid = rc_filter_empty();
    mbot_vx_pid = rc_filter_empty();
    mbot_wz_pid = rc_filter_empty();

    rc_filter_pid(&left_wheel_pid, ctlr_cfg.left.kp, ctlr_cfg.left.ki, ctlr_cfg.left.kd, ctlr_cfg.left.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_wheel_pid, ctlr_cfg.right.kp, ctlr_cfg.right.ki, ctlr_cfg.right.kd, ctlr_cfg.right.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vx_pid, ctlr_cfg.vx.kp, ctlr_cfg.vx.ki, ctlr_cfg.vx.kd, ctlr_cfg.vx.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_wz_pid, ctlr_cfg.wz.kp, ctlr_cfg.wz.ki, ctlr_cfg.wz.kd, ctlr_cfg.wz.Tf, MAIN_LOOP_PERIOD);

    //ACcceleration-limiting 1st-Order LPF
    mbot_vel_vx_lpf = rc_filter_empty();
    mbot_vel_wz_lpf = rc_filter_empty();
    rc_filter_first_order_lowpass (&mbot_vel_vx_lpf, MAIN_LOOP_PERIOD, 2.0 * MAIN_LOOP_PERIOD);
    rc_filter_first_order_lowpass (&mbot_vel_wz_lpf, MAIN_LOOP_PERIOD, 2.0 * MAIN_LOOP_PERIOD);
  
    #ifdef MBOT_TYPE_OMNI
    back_wheel_pid = rc_filter_empty();
    mbot_vy_pid = rc_filter_empty();
    rc_filter_pid (&mbot_vy_pid, ctlr_cfg.vy.kp, ctlr_cfg.vy.ki, ctlr_cfg.vy.kd, ctl_cfg.vy.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&back_wheel_pid, ctlr_cfg.back.kp, ctlr_cfg.back.ki, ctlr_cfg.back.kd, ctlr_cfg.back.Tf, MAIN_LOOP_PERIOD);
    #endif

    return 0;
}

//Use the Body velocity and Body angular velocity to 
//Output the desired command angular velocities for the left and right wheel motors
int mbot_ctlr(serial_twist2D_t vel_cmd, serial_twist2D_t vel, serial_mbot_motor_vel_t *mbot_motor_vel) {
    // Implement the Angular Velocity Estimator and PID controller for angular velocity.
    float vel_wz_lp = rc_filter_march(&mbot_vel_wz_lpf, vel.wz);
    float vel_cmd_wz = vel_cmd.wz + rc_filter_march(&mbot_wz_pid, vel_cmd.wz - vel_wz_lp);

    //find the commanded left and right motor angular velocities
    float w_L = (vel_cmd.vx - DIFF_BASE_RADIUS * vel_cmd_wz) / DIFF_WHEEL_RADIUS; 
    float w_R = (-vel_cmd.vx - DIFF_BASE_RADIUS * vel_cmd_wz) / DIFF_WHEEL_RADIUS;

    mbot_motor_vel->velocity[MOT_L] = w_L;
    mbot_motor_vel->velocity[MOT_R] = w_R;

    return 0;
}

//Left and Right Wheel Motor Velocities PID Controller; 
//Given desired wheel velocity and actual wheel velocity 
int mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, serial_mbot_motor_pwm_t *mbot_motor_pwm) {
    //find the errors by command - actual motor reading
    float left_error = vel_cmd.velocity[MOT_L] - vel.velocity[MOT_L];
    float right_error = vel_cmd.velocity[MOT_R] - vel.velocity[MOT_R];

    //find the calibrated PWM with the error found and wheele PID Controllerst    
    float left_cmd = rc_filter_march (&left_wheel_pid, left_error);
    float right_cmd = rc_filter_march(&right_wheel_pid, right_error);

    //send the calibrated PWM to the motors
    mbot_motor_pwm->pwm[MOT_L] = left_cmd; 
    mbot_motor_pwm->pwm[MOT_R] = right_cmd;
  
    //Omni-specific code
    #ifdef MBOT_TYPE_OMNI
    float back_error = vel_cmd.velocity[MOT_B] - vel.velocity[MOT_B];
    float back_cmd = rc_filter_march(&back_wheel_pid, back_error);
    mbot_motor_pwm->pwm[MOT_B] = back_cmd;
    #endif

    return 0;
}

