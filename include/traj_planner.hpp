#pragma once
#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <polynomial.hpp>
#include <timer.hpp>
#include <trajectory.hpp>
#include <obstacle_generator.hpp>
#include <kalman_filter.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

// Safe Corridor
#include <collision_constraints.hpp>

// Trajectory Optimizer
#include <traj_optimizer.hpp>

//Grid-based planner
#include <grid_based_planner.hpp>

// Goal Optimizer
#include <goal_optimizer.hpp>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>


namespace DynamicPlanning {
    class TrajPlanner {
    public:
        TrajPlanner(const ros::NodeHandle &nh, const Param &param, const Mission &mission, const Agent &agent);

        traj_t plan(const Agent &agent,
                    const std::shared_ptr<octomap::OcTree> &octree_ptr,
                    const std::shared_ptr<DynamicEDTOctomap> &distmap_ptr,
                    ros::Time sim_current_time,
                    bool is_disturbed);

        void publish();

        // Setter
        void setObstacles(const std::vector<Obstacle> &obstacles);

        // Getter
        [[nodiscard]] int getPlannerSeq() const;

        [[nodiscard]] point3d getCurrentGoalPosition() const;

        [[nodiscard]] PlanningStatistics getPlanningStatistics() const;

        [[nodiscard]] bool getCollisionAlert() const;

    private:
        Param param;
        Mission mission;

        // ROS
        ros::NodeHandle nh;
        ros::Publisher pub_sfc;
        ros::Publisher pub_lsc;
        ros::Publisher pub_feasible_region;
        ros::Publisher pub_initial_traj_vis;
        ros::Publisher pub_obs_pred_traj_vis;
        ros::Publisher pub_grid_path;
        ros::Publisher pub_grid_occupied_points;
        ros::Time sim_current_time;

        // Agent state, report
        Agent agent;
        int planner_seq;
        PlanningStatistics statistics;
        bool initialize_sfc, is_disturbed, is_sol_converged_by_sfc;
        GoalPlannerState goal_planner_state;
        int desired_segment_idx;
        Box sfc_converged;

        // Bernstein Matrix
        Eigen::MatrixXd B, B_inv;

        // Trajectories
        traj_t initial_traj; // [segment_idx][control_pts_idx], initial trajectory
        traj_t prev_traj; // [segment_idx][control_pts_idx], previous trajectory

        // Obstacle
        std::shared_ptr<octomap::OcTree> octree_ptr; // octomap
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr; // Euclidean distance field map
        std::vector<Obstacle> obstacles; // obstacles
        std::vector<traj_t> obs_pred_trajs; // predicted trajectory of obstacles
        std::vector<Trajectory<double>> obs_pred_sizes; // predicted obstacle size
        std::set<int> col_pred_obs_indices; // collision predicted obstacle indices

        // Collision constraints
        CollisionConstraints constraints;

        // Grid based planner
        std::unique_ptr<GridBasedPlanner> grid_based_planner;

        // Trajectory optimizer
        std::unique_ptr<TrajOptimizer> traj_optimizer;

        // Kalman filter
        std::vector<LinearKalmanFilter> linear_kalman_filters;

        // ROS
        void initializeROS();

        // Planner module
        traj_t planImpl();

        // Functions for checking agent state
        void checkPlannerMode(); // Check modes in launch file are valid, and fix them automatically

        [[nodiscard]] bool isDeadlock();

        [[nodiscard]] bool isSolConv() const;

        [[nodiscard]] bool isSolConvNearGoal(int closest_agent_idx) const;

         bool isSolConvBySFC();

        [[nodiscard]] bool isSolValid(const TrajOptResult& result) const;

        // Obstacle prediction
        void obstaclePrediction();

        void obstaclePredictionWithCurrPos(); // Need the position of obstacles

        void obstaclePredictionWithCurrVel(); // Need the position and velocity of obstacles

        void obstaclePredictionWithPrevSol(); // Need trajectory of other agents planned in the previous step.
        // Dynamic obstacle -> current velocity, Agent -> prev sol
        void checkObstacleDisturbance(); // Check obstacle is at the start point of predicted trajectory
        // If not, correct predicted trajectory using obstacle position.
        void obstacleSizePredictionWithConstAcc(); // Predict obstacle size using constant acceleration model

        // Initial trajectory planning
        void initialTrajPlanning();

        void initialTrajPlanningCurrPos();

        void initialTrajPlanningCurrVel();

        void initialTrajPlanningPrevSol();

        void initialTrajPlanningCheck(); // Check agent is at the start point of initial trajectory
        // If not, correct predicted trajectory using obstacle position.

        // Goal planning
        void goalPlanning();

        void goalPlanningWithStaticGoal();

        void goalPlanningWithRightHandRule();

        void goalPlanningWithPriority();

        void goalPlanningWithGridBasedPlanner();

        // Collision constraints
        void constructLSC();

        void constructSFC();

        void generateReciprocalRSFC(); // used in RAL2021 submission

        void generateLSC();

        void generateCLSC();

        void generateBVC();

        void generateSFC();

        static point3d normalVectorBetweenLines(const Line &obs_path, const Line &agent_path, double &closest_dist);

        point3d normalVectorBetweenPolys(int oi, int m,
                                         const traj_t& initial_traj_trans,
                                         const traj_t& obs_pred_traj_trans);

        point3d normalVectorDynamicObs(int oi, int m, double downwash);

        // Trajectory Optimization
        traj_t trajOptimization();

        // Publish
//        void publishCollisionConstraints();

//        void publishInitialTraj();

        void publishSFC();

        void publishLSC();

        void publishFeasibleRegion();

        void publishGridPath();

        void publishObstaclePrediction();

//        void publishStartGoalPoints();

        void publishGridOccupiedPoints();

        // Utility functions
        [[nodiscard]] int findObstacleIdxByObsId(int obs_id) const;

        [[nodiscard]] double distanceToGoalByObsId(int obs_id) const;

        [[nodiscard]] double distanceToGoalByObsIdx(int obs_idx) const;

        double computeCollisionTimeToDistmap(const point3d &start_position,
                                             const point3d &goal_position,
                                             double agent_radius,
                                             double time_horizon);

        [[nodiscard]] double downwashBetween(int obs_idx) const;

        [[nodiscard]] double downwashBetween(int obs_idx1, int obs_idx2) const;

        [[nodiscard]] static point3d coordinateTransform(const point3d& point, double downwash);

        [[nodiscard]] static double computePriority(double dist_to_goal, double goal_threshold);

        void updateCurrentGoal();
    };
}