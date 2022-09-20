#ifndef LSC_PLANNER_GRIDBASEDPLANNER_HPP
#define LSC_PLANNER_GRIDBASEDPLANNER_HPP

#include <mapf/pibt.hpp>
#include <mapf/ecbs.hpp>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <mission.hpp>
#include <param.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <utility>
#include <collision_constraints.hpp>

#define GP_OCCUPIED 1
#define GP_EMPTY 0
#define GP_INFINITY 10000

// Wrapper for grid based path planner
namespace DynamicPlanning {
    struct GridNode {
    public:
        GridNode() {
            data = {-1, -1, -1};
        }

        GridNode(int i, int j, int k) {
            data = {i, j, k};
        }


        GridNode operator+(const GridNode &other_node) const;

        GridNode operator-(const GridNode &other_node) const;

        GridNode operator*(int integer) const;

        GridNode operator-() const;

        bool operator==(const GridNode &other_node) const;

        bool operator!=(const GridNode &other_node) const;

        bool operator<(const GridNode &other_node) const;

        [[nodiscard]] int dot(const GridNode &other_agent) const;

        [[nodiscard]] double norm() const;

        [[nodiscard]] int i() const { return data[0]; }

        [[nodiscard]] int j() const { return data[1]; }

        [[nodiscard]] int k() const { return data[2]; }

        int &operator[](unsigned int idx) { return data[idx]; }

        const int &operator[](unsigned int idx) const { return data[idx]; }

        [[nodiscard]] std::array<int, 3> toArray() const { return data; }

    private:
        std::array<int, 3> data{-1, -1, -1};
    };


    struct GridInfo {
        std::array<double, 3> grid_min;
        std::array<double, 3> grid_max;
        std::array<int, 3> dim;
    };

    struct GridMap {
        std::vector<std::vector<std::vector<int>>> grid;

        int getValue(GridNode grid_node) const {
            return grid[grid_node[0]][grid_node[1]][grid_node[2]];
        }

        void setValue(GridNode grid_node, int value) {
            grid[grid_node[0]][grid_node[1]][grid_node[2]] = value;
        }
    };

    typedef std::vector<GridNode> gridpath_t;
    typedef std::vector<GridNode> GridNodes;

    struct GridMission {
        size_t n_agents;
        GridNodes start_points;
        GridNodes current_points;
        GridNodes goal_points;
    };

    struct PlanResult {
        size_t n_agents;
        std::vector<points_t> paths;
    };

    class GridBasedPlanner {
    public:
        GridBasedPlanner(const DynamicPlanning::Param &param, const DynamicPlanning::Mission &mission);

        // single agent path planning
        bool planSAPF(const Agent &agent,
                      const std::shared_ptr<DynamicEDTOctomap> &_distmap_ptr,
                      const std::vector<Obstacle> &obstacles = {},
                      const std::set<int> &grid_obstacles = {});

        // multi agent path planning
        bool planMAPF(const points_t &start_points,
                      const points_t &current_points,
                      const points_t &goal_points,
                      const std::shared_ptr<DynamicEDTOctomap> &_distmap_ptr,
                      double agent_radius, double agent_downwash);

        // Getter
        [[nodiscard]] points_t getPath(size_t i) const;

        [[nodiscard]] points_t getFreePoints() const;

        [[nodiscard]] points_t getOccupiedPoints() const;

        [[nodiscard]] visualization_msgs::MarkerArray occupiedPointsToMsg(std::string frame_id) const;

        [[nodiscard]] visualization_msgs::MarkerArray pathToMarkerMsg(int agent_id,
                                                                      std::string frame_id,
                                                                      std_msgs::ColorRGBA color) const;

        // Goal
        point3d findLOSFreeGoal(const point3d &current_position,
                                const point3d &goal_position,
                                double agent_radius);

    private:
        Mission mission;
        Param param;
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;

        GridInfo grid_info;
        GridMap grid_map;
        GridMission grid_mission;
        PlanResult plan_result;

        void updateGridInfo();

        void updateGridMap(double agent_radius,
                           double agent_downwash,
                           const std::vector<Obstacle> &obstacles = {},
                           const std::set<int> &grid_obstacles = {});

        void updateGridMission(const point3d &start_point,
                               const point3d &goal_point);

        void updateGridMission(const points_t &start_points,
                               const points_t &current_points,
                               const points_t &goal_points);

        bool isValid(const GridNode &grid_node);

        bool isOccupied(const GridMap &map, const GridNode &grid_node);

        bool planImpl(bool is_mapf);

        std::vector<gridpath_t> runMAPF(const GridMap &grid_map, const GridMission &grid_mission);

        [[nodiscard]] points_t gridPathToPath(const gridpath_t &grid_path) const;

        [[nodiscard]]point3d gridNodeToPoint3D(const GridNode &grid_node) const;

        [[nodiscard]] point3d gridNodeToPoint3D(const GridNode &grid_node, int dimension) const;

        [[nodiscard]] std::vector<std::array<int, 3>> gridNodesToArrays(const GridNodes &grid_nodes) const;

        [[nodiscard]] GridNode point3DToGridVector(const point3d &point) const;

        bool castRay(const point3d &current_position, const point3d &goal_position, double agent_radius);
    };
}

#endif //LSC_PLANNER_GRIDBASEDPLANNER_HPP
