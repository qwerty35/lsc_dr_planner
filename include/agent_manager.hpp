#ifndef LSC_PLANNER_AGENT_MANAGER_H
#define LSC_PLANNER_AGENT_MANAGER_H

#include <trajectory.hpp>
#include <traj_planner.hpp>
#include <map_manager.hpp>
#include <util.hpp>

namespace DynamicPlanning {
    class AgentManager {
    public:
        AgentManager(const ros::NodeHandle &nh, const Param &param, const Mission &mission, int agent_id);

        void doStep(double time_step);

        PlanningReport plan(ros::Time sim_current_time);

        void publish();

        void publishMap();

        void obstacleCallback(const std::vector<Obstacle> &msg_obstacles);

        void mergeMapCallback(const octomap_msgs::Octomap& msg_merge_map);

        bool isInitialStateValid();

        // Setter
        void setCurrentState(const State& msg_current_state);

        void setPlannerState(const PlannerState& new_planner_state);

        void setStartPosition(const point3d& new_start_position);

        void setDesiredGoal(const point3d& new_desired_goal);

        void setGlobalMap();

        void setGlobalMap(const sensor_msgs::PointCloud2& global_map);

        void setNextWaypoint(const point3d& next_waypoint);

        // Getter
        [[nodiscard]] point3d getCurrentPosition() const;

        [[nodiscard]] State getCurrentState() const;

        [[nodiscard]] State getFutureState(double future_time) const;

        [[nodiscard]] PlanningStatistics getPlanningStatistics() const;

        [[nodiscard]] traj_t getTraj() const;

        [[nodiscard]] int getPlannerSeq() const;

        [[nodiscard]] point3d getCurrentGoalPoint() const;

        [[nodiscard]] point3d getDesiredGoalPoint() const;

        [[nodiscard]] Obstacle getAgent() const;

        [[nodiscard]] octomap_msgs::Octomap getOctomapMsg() const;

        [[nodiscard]] point3d getNextWaypoint() const;

        [[nodiscard]] std::shared_ptr<DynamicEDTOctomap> getDistmap() const;

        [[nodiscard]] point3d getStartPoint() const;

//        [[nodiscard]] double getVelContError() const;
//
//        [[nodiscard]] double getAccContError() const;

    private:
        Param param;
        Mission mission;

        // Flags, states
        PlannerState planner_state;
        bool has_current_state, has_obstacles, has_local_map, is_disturbed;

        // Agent
        Agent agent;
        traj_t desired_traj;
        bool collision_alert;

        //Traj Planner
        std::unique_ptr<TrajPlanner> traj_planner;
        std::unique_ptr<MapManager> map_manager;

        void planningStateTransition();
    };
}

#endif //LSC_PLANNER_AGENT_MANAGER_H
