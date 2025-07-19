#include <mbot_lcm_msgs_serial.h>
#include <mbot/defs/mbot_params.h>
#include <math.h>

//Calculate the odometry position and orientation of the robot based on the body velocity data
//mbot_vel: current body velocity of the robot
//dt: time interval over the velocity is applied
//odometry: pointer to the structure where the calculated odometry will be stored
//return int: returns 0 if success

int mbot_calculate_odometry (serial_twist20_t mbot_vel, float dt, serial_pose2D_t * odometry, serial_mbot_imu_t *imu);
