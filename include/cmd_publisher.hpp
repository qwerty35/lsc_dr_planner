#ifndef LSC_PLANNER_CMD_PUBLISHER_H
#define LSC_PLANNER_CMD_PUBLISHER_H

#include <ros/ros.h>
#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <dynamic_msgs/TrajectoryArray.h>
#include <dynamic_msgs/FullState.h>
#include <dynamic_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <linear_kalman_filter.hpp>


namespace DynamicPlanning {
    class CmdPublisher {
    public:
        CmdPublisher(const ros::NodeHandle &nh, const Param &param, const Mission &mission, int agent_id);

        void updateTraj(const Trajectory<point3d> &new_traj, const ros::Time &traj_start_time);

        void landingCallback();

        [[nodiscard]] bool isDisturbed() const;

        [[nodiscard]] bool isAgentPoseUpdated() const;

        [[nodiscard]] bool isObsPoseUpdated(int obs_id) const;

        [[nodiscard]] bool landingFinished() const;

        [[nodiscard]] point3d getObservedAgentPosition() const;

        [[nodiscard]] point3d getObservedObsPosition(int obs_id) const;

        [[nodiscard]] nav_msgs::Odometry getObsOdometry(int obs_id) const;

    private:
        Param param;
        Mission mission;

        ros::NodeHandle nh;
        ros::Timer cmd_timer;
        ros::Publisher pub_cmd;
        ros::Publisher pub_cmd_stop;
        ros::Publisher pub_cmd_vis;
        tf::TransformListener tf_listener;

        int agent_id;
        size_t M, n;
        double dt, landing_time;

        point3d observed_agent_position;
        std::vector<LinearKalmanFilter> linear_kalman_filters;
        std::map<int, nav_msgs::Odometry> observed_obs_odoms;
        std::queue<Trajectory<point3d>> traj_queue;
        std::queue<ros::Time> traj_start_time_queue;
        Trajectory<point3d> current_traj;
        ros::Time current_traj_start_time, landing_start_time;
        bool initialized, external_agent_pose_update, external_obs_pose_update, is_disturbed, landing;
        double average_diff, diff_count, max_diff;

        void cmdTimerCallback(const ros::TimerEvent &event);

        void listenTF();

        void loadCurrentTraj();

        bool computeDesiredState(dynamic_msgs::State &desired_state);

        void detectDisturbance(dynamic_msgs::State &desired_state);

        void publishCommand(const dynamic_msgs::State &desired_state);

        void publishLandingCommand(const dynamic_msgs::State &desired_state);

        void failsafe();
    };
}

#endif //LSC_PLANNER_CMD_PUBLISHER_H
