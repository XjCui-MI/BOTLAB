#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>

#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/timestamp_t.hpp>
#include <mbot_lcm_msgs/mbot_message_received_t.hpp>
#include <mbot_lcm_msgs/mbot_slam_reset_t.hpp>
#include <utils/timestamp.h>
#include <utils/geometric/angle_functions.hpp>
#include <utils/geometric/pose_trace.hpp>
#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>

#include "diff_maneuver_controller.h"

/*
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */

//Implement the Pure Pursuit Controller
class PurePursuitManeuverController: public ManeuverControllerBase {
    private: 
        float Kx = 0.8;
        float wz_pid[3] = {0, 0, 2.0};  //Ka, Kb, Kw
        float d_end_crit = 0.02;
        float d_end_midsteps = 0.08;
        float angle_end_crit = 0.2;
    public:
        PurePursuitManeuverController() = default;
        virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, cnst mbot_lcm_msgs::pose2D_t& target) override {
            float vel_sign = 1;
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float alpha = angle_diff(atan2(dy, dx), pose.theta);
            //Defined look-ahead distance
            float L = 0.5
            float xg = L * cos(alpha);
            float yg = L * sin(alpha);
            float r = L * L / (2 * yg);

            float fwd_vel = vel_sign * Kx * L;
            float turn_vel = wz_pid[2] * (1 / r);

            //if alpha is more than 45 degrees, turn in place and then go
            if (alpha > MI_PI/4) {
                fwd_vel = 0;
            }

            return {0, fwd_vel, 0, turn_vel};
        }

        virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose)  override {
                float distance = d_end_midsteps;
                if (is_end_pose)
                    distance = d_end_crit;
                return ((fabs(pose.x - target.x) < distance) && (fabs(pose.y - target.y)  < distance));
        }
}


//Implement the Smart Controller
class SmartManeuverController: public ManueverControllerBase {
    private: 
        float pid[3] = {1.0, 2.5, 0.1}; 
        float d_end_crit = 0.02;
        float d_end_midsteps = 0.08;
        float angle_end_crit = 0.2;
    public:
        SmartManeuverController() = default;
        virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) override
        {
            //Implement the smart controller with the picked proportional gains
            float vel_sign = 1;
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float d_fwd = sqrt(dx * dx + dy * dy);
            float alpha = angle_diff(atan2(dy,dx), pose.theta);
            float beta = wrap_to_pi(target.theta - (alpha + pose.theta));
            float fwd_vel = vel_sign * pid[0] * d_fwd;
            float turn_vel = pid[1] * alpha + pid[2] * beta;

            if (fabs(alpha) > M_PI_4) {
                fwd_vel = 0;
            }
            return {0, fwd_vel, 0, turn_vel};
        }

        virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose) override {
            float distance = d_end_midsteps;
            if (is_end_pose) {
                distance = d_end_crit;
            }
            return ((fabs(pose.x - target.x) < distance) && (fabs(pose.y - target.y) < distance));
        }
};


class StraightManeuverController: public ManeuverControllerBase {
    private: 
        float fwd_pid[3] = {1.0, 0, 0};
        float fwd_sum_error = 0;
        float fwd_last_error = 0;
        float turn_pid[3] = {3.0, 0, 0};
        float turn_sum_error = 0;
        float turn_last_error = 0;
    public:
        StraightManeuverController() = default;
        virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm:pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) override {
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float d_fwd = sqrt(pow(dx, 2) + pow(dy,2));
            float d_theta = angle_diff(atan2(dy, dx), pose.theta);

            //PID controller for fwd

            fwd_sum_error += d_fwd;
            float fwd_der = 0;
            if (fwd_last_error > 0) {
                fwd_der = (d_fwd - fwd_last_error) / 0.05;
            }
            float fwd_vel = fwd_pid[0] * d_fwd + fwd_pid[1] * fwd_sum_error + fwd_pid[2] * fwd_der;

            //PID controller for angular velocity
            turn_sum_error += d_fwd;
            float turn_der = 0;
            if (turn_last_error > 0) {
                turn_der = angle_diff (d_theta, turn_last_error) / 0.05;
            }
            float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
            return {0, fwd_vel, 0, turn, vel};
        }

        virtual bool target_reached (const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose) override {
            return (fabs(pose.x - target.x) < 0.02) && (fabs(pose.y - target.y) < 0.02));
        }
};

            
class TurnManeuverController : public ManeuverControllerBase {
    private:
        float turn_pid[3] = {3.0, 0, 0};
        float turn_sum_error = 0;
        float turn_last_error = 0;
    public:
        TurnManeuverController() = default;   
        virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) override {
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float d_theta = angle_diff(atan2(dy,dx), pose.theta);

            // PID for the angular velocity given the delta theta
            turn_sum_error += d_theta;
            float turn_der = 0.0;
            if (turn_last_error > 0)
                turn_der = (d_theta - turn_last_error) / 0.05;
            float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
            return {0, 0, 0, turn_vel};
        }
        mbot_lcm_msgs::twist2D_t get_command_final_turn(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) {
            float d_theta = angle_diff(target.theta, pose.theta);
            // PID for the angular velocity given the delta theta
            turn_sum_error += d_theta;
            float turn_der = 0;
            if (turn_last_error > 0)
                turn_der = (d_theta - turn_last_error) / 0.05;
            float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
            return {0, 0, 0, turn_vel};
        }
        virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose)  override {
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float target_heading = atan2(dy, dx);
            // Handle the case when the target is on the same x,y but on a different theta
            return (fabs(angle_diff(pose.theta, target_heading)) < 0.05);
        }
        bool target_reached_final_turn(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) {
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float target_heading = atan2(dy, dx);
            // Handle the case when the target is on the same x,y but on a different theta
            return (fabs(angle_diff(target.theta, pose.theta)) < 0.05);
        }
};


class MotionController {
    public: 
        MotionController(lcm::LCM * instance)
        :
            lcmInstance(instance),
            odomToGlobalFrame_{0, 0, 0, 0}
        {
            subscribeToLcm();
	        time_offset = 0;
	        timesync_initialized_ = false;

            //default velocity limits: low speed 0.3, high speed 0.8
            vel_limits_.vx = 0.3; 
            vel_limits_.vy = 0;
            // low speed M_PI * 2.0 / 3.0, high speed M_PI
            vel_limits_.wz = M_PI * 2.0 / 3.0;   
        }
    
        mbot_lcm_msgs::twist2D_t updateCommand(void) 
        {
            mbot_lcm_msgs::twist2D_t cmd {now(), 0.0, 0.0, 0.0};
        
            if(!targets_.empty() && !odomTrace_.empty()) 
            {
                mbot_lcm_msgs::pose2D_t target = targets_.back();
                bool is_last_target = targets_.size() == 1;
                mbot_lcm_msgs::pose2D_t pose = currentPose();

                if (state_ == SMART) 
                {
                    if (smart_controller.target_reached(pose, target, is_last_target))
                    {
                        if (is_last_target)
                            state_ = FINAL_TURN;
                        else if(!assignNextTarget())
                            printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                    }
                    else cmd = smart_controller.get_command(pose, target);
                }

                //Add different states when adding maneuver controls // 
                if (state_ == PURE_PURSUIT) {
                    if (pure_pursuit_controller.target_reached(pose, target, is_last_target)) {
                        if (is_last_target)
                            state_ = FINAL_TURN;
                        else if (!assignNextTarget())
                            printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                    }
                    else cmd = pure_pursuit_controller.get_command(pose, target);
                }

                if (state_ == INITIAL_TURN) {
                    if (turn_controller.target_reached(pose, target, is_last_target)) {
		                    state_ = DRIVE;
                    } 
                    else {
                        cmd = turn_controller.get_command(pose, target);
                    }
                } else if (state_ == DRIVE) {
                    if (straight_controller.target_reached(pose, target, is_last_target)) {
                        state_ = FINAL_TURN;
                    }
                    else {
                        cmd = straight_controller.get_command(pose, target);
                    }      
                } else if (state_ == FINAL_TURN) { 
                    if (turn_controller.target_reached_final_turn(pose, target)) {
		                    if(!assignNextTarget()) {
                            printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                        }
                    } else {
                        cmd = turn_controller.get_command_final_turn(pose, target);
                    }
                }
            }
            return cmd; 
        }

        bool timesync_initialized(){ return timesync_initialized_; }
        void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::timestamp_t* timesync)
        {
	        timesync_initialized_ = true;
	        time_offset = timesync->utime-utime_now();
        }
    
        void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::path2D_t* path)
        {
            targets_ = path->path;
            std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	    std::cout << "received new path at time: " << path->utime << "\n"; 
    	    for(auto pose : targets_)
            {
    		    std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	    }
            std::cout << std::endl;

            // assignNextTarget(); // This eats the first waypoint
            //state_ = SMART;
            state_ = PURE_PURSUIT;

            //confirm that the path was received
            mbot_lcm_msgs::mbot_message_received_t confirm {now(), path->utime, channel};
            lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
        }
    
        void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* odometry)
        {
            mbot_lcm_msgs::pose2D_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
            odomTrace_.addPose(pose);
        }
    
        void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose)
        {
            computeOdometryOffset(*pose);
        }

        void handleMaxVelocity(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::twist2D_t* new_limits)
        {
            vel_limits_.vx = new_limits->vx;
            vel_limits_.vy = new_limits->vy;
            vel_limits_.wz = new_limits->wz;
        }
    
        // getter method to access targets_
        const std::vector<mbot_lcm_msgs::pose2D_t>& getTargets() const
        {
            return targets_;
        }

        const mbot_lcm_msgs::twist2D_t& getVelLimits() const
        {
            return vel_limits_;
        }

    private:
        enum State {
            INITIAL_TURN,
            DRIVE,
            FINAL_TURN, 
            SMART,
            PURE_PURSUIT
        };

        //Transform to convert odometry into the global/map coordinates for navigating in a map
        mbot_lcm_msgs::pose2D_t odomToGlobalFrame_;     
        //Trace of odometry for maintaining the offset estimate
        PoseTrace  odomTrace_;              
        std::vector<mbot_lcm_msgs::pose2D_t> targets_;
        mbot_lcm_msgs::twist2D_t vel_limits_;

        State state_;
        int64_t time_offset;
        bool timesync_initialized_;

        lcm::LCM * lcmInstance;

        StraightManeuverController straight_controller;
        TurnManeuverController turn_controller;
        SmartManeuverController smart_controller;
        PurePursuitManeuverController pure_pursuit_controller;

        int64_t now()
        {
	        return utime_now() + time_offset;
        }
    
        bool assignNextTarget(void) {
            if(!targets_.empty()) { targets_.pop_back(); }
            state_ = PURE_PURSUIT; 
            return !targets_.empty();
        }
    
        void computeOdometryOffset(const mbot_lcm_msgs::pose2D_t& globalPose) {
            mbot_lcm_msgs::pose2D_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
            double deltaTheta = globalPose.theta - odomAtTime.theta;
            double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
            double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
         
            odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
            odomToGlobalFrame_.y = globalPose.y - yOdomRotated; 
            odomToGlobalFrame_.theta = deltaTheta;
        }
    
        mbot_lcm_msgs::pose2D_t currentPose(void)
        {
            assert(!odomTrace_.empty());
            mbot_lcm_msgs::pose2D_t odomPose = odomTrace_.back();
            mbot_lcm_msgs::pose2D_t pose;
            pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) 
                + odomToGlobalFrame_.x;
            pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
                + odomToGlobalFrame_.y;
            pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
            return pose;
        }

        void subscribeToLcm()
        {
            lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
            lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
            lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
            lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
            lcmInstance->subscribe(MBOT_MAX_VEL_CHANNEL, &MotionController::handleMaxVelocity, this);
        }
    };

    bool ctrl_c_pressed;
    void ctrlc(int)
    {
        ctrl_c_pressed = true;
    }

    int main (int argc, char** argv) {
        lcm::LCM lcmInstance(MULTICAST_URL);
        MotionController controller(&lcmInstance);

        ctrl_c_pressed = false;
        signal(SIGINT, ctrlc);
        signal(SIGTERM, ctrlc);
    
        while (true) { 
            lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	    if (controller.timesync_initialized() && !controller.getTargets().empty()){
                mbot_lcm_msgs::twist2D_t cmd = controller.updateCommand();
                // Limit command values
                // Fwd vel
                if (cmd.vx > controller.getVelLimits().vx) cmd.vx = controller.getVelLimits().vx;
                else if (cmd.vx < -controller.getVelLimits().vx) cmd.vx = -controller.getVelLimits().vx;

                // Angular vel
                if (cmd.wz > controller.getVelLimits().wz) cmd.wz = controller.getVelLimits().wz;
                else if (cmd.wz < -controller.getVelLimits().wz) cmd.wz = -controller.getVelLimits().wz;

                lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        	}
        
            if (ctrl_c_pressed) break;
        }

        // Stop the robot when motion controller quits.
        mbot_lcm_msgs::twist2D_t zero;
        zero.vx = 0;
        zero.vy = 0;
        zero.wz = 0;
        lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &zero);
        std::cout << "Robot stopped successfully. Exiting..." << std::endl;

        return 0;
}



        



        
















