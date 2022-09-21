#pragma once

#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-5
#define SP_INFINITY         1e+9
#define PI 3.1415

#include <stdexcept>
#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

namespace DynamicPlanning {
    typedef octomap::point3d point3d;
    typedef octomap::point3d vector3d;
    typedef std::vector<point3d> points_t;

    enum class PlannerMode {
        DLSC,
        LSC,
        BVC,
        ORCA,
        RECIPROCALRSFC,
        CIRCLETEST,
    };

    enum class PredictionMode {
        POSITION,
        VELOCITY,
        ORCA,
        PREVIOUSSOLUTION,
    };

    enum class InitialTrajMode {
        POSITION,
        VELOCITY,
        ORCA,
        PREVIOUSSOLUTION,
        SKIP,
    };

    enum class SlackMode {
        NONE,
        CONTINUITY,
        COLLISIONCONSTRAINT,
    };

    enum class GoalMode {
        STATIC,
        ORCA,
        RIGHTHAND,
        PRIORBASED,
        DYNAMICPRIORITY,
        ENTROPY,
        GRIDBASEDPLANNER,
    };

    enum class MAPFMode {
        PIBT,
        ECBS,
    };

    enum PlannerState {
        WAIT,
        GOTO,
        PATROL,
        GOBACK,
        LAND,
    };

    enum PlanningReport {
        Initialized,
        INITTRAJGENERATIONFAILED,
        CONSTRAINTGENERATIONFAILED,
        QPFAILED,
        WAITFORROSMSG,
        SUCCESS,
    };

    enum GoalPlannerState{
        FORWARD,
        RIGHT,
        BACK,
    };

    struct PlanningTime {
        void update(double time){
            current = time;
            if(time < min){
                min = time;
            }
            if(time > max){
                max = time;
            }

            N_sample++;
            average = (average * (N_sample - 1) + time) / N_sample;
        }

        double current = 0;
        double min = SP_INFINITY;
        double max = 0;
        double average = 0;
        int N_sample = 0;
    };

    struct PlanningTimeStatistics {
        void update(PlanningTimeStatistics new_planning_time){
            mapf_time.update(new_planning_time.mapf_time.current);
            initial_traj_planning_time.update(new_planning_time.initial_traj_planning_time.current);
            obstacle_prediction_time.update(new_planning_time.obstacle_prediction_time.current);
            goal_planning_time.update(new_planning_time.goal_planning_time.current);
            lsc_generation_time.update(new_planning_time.lsc_generation_time.current);
            sfc_generation_time.update(new_planning_time.sfc_generation_time.current);
            traj_optimization_time.update(new_planning_time.traj_optimization_time.current);
            total_planning_time.update(new_planning_time.total_planning_time.current);
        }

        PlanningTime mapf_time;
        PlanningTime initial_traj_planning_time;
        PlanningTime obstacle_prediction_time;
        PlanningTime goal_planning_time;
        PlanningTime lsc_generation_time;
        PlanningTime sfc_generation_time;
        PlanningTime traj_optimization_time;
        PlanningTime total_planning_time;
    };

    struct PlanningStatistics {
        int planning_seq = 0;
        PlanningTimeStatistics planning_time;
    };

    enum ObstacleType {
        DYNAMICOBSTACLE,
        AGENT,
    };

    struct State{
        point3d position;
        point3d velocity;
        point3d acceleration;
    };

    struct Agent{
        int id; // id used in planner
        int cid; // crazyflie id
        State current_state;
        point3d start_point;
        point3d desired_goal_point;
        point3d current_goal_point;
        point3d next_waypoint;
        std::vector<double> max_vel; //TODO: in most case, index 0 is only used
        std::vector<double> max_acc; //TODO: in most case, index 0 is only used
        double radius;
        double downwash;
        double nominal_velocity;
        bool collision_alert;
    };

    struct ClosestPoints{
        double dist;
        point3d closest_point1;
        point3d closest_point2;
    };
}