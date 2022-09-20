#include <grid_based_planner.hpp>

namespace DynamicPlanning {
    GridNode GridNode::operator+(const GridNode &other_node) const {
        return {i() + other_node.i(), j() + other_node.j(), k() + other_node.k()};
    }

    GridNode GridNode::operator-(const GridNode &other_node) const {
        return {i() - other_node.i(), j() - other_node.j(), k() - other_node.k()};
    }

    GridNode GridNode::operator-() const {
        return {-i(), -j(), -k()};
    }

    GridNode GridNode::operator*(int integer) const {
        return {i() * integer, j() * integer, k() * integer};
    }

    bool GridNode::operator==(const GridNode &other_node) const {
        for (unsigned int i = 0; i < 3; i++) {
            if (operator[](i) != other_node[i])
                return false;
        }
        return true;
    }

    bool GridNode::operator!=(const GridNode &other_node) const {
        for (unsigned int i = 0; i < 3; i++) {
            if (operator[](i) != other_node[i])
                return true;
        }
        return false;
    }

    bool GridNode::operator<(const GridNode &other_node) const {
        for (unsigned int i = 0; i < 3; i++) {
            if (operator[](i) < other_node[i]) {
                return true;
            } else if (operator[](i) > other_node[i]) {
                return false;
            }
        }
        return false;
    }

    int GridNode::dot(const GridNode &other_agent) const {
        return i() * other_agent.i() + j() * other_agent.j() + k() * other_agent.k();
    }

    double GridNode::norm() const {
        return sqrt(i() * i() + j() * j() + k() * k());
    }

    GridBasedPlanner::GridBasedPlanner(const DynamicPlanning::Param &_param,
                                       const DynamicPlanning::Mission &_mission)
            : param(_param), mission(_mission) {
        updateGridInfo();
    }

    bool GridBasedPlanner::planSAPF(const Agent &agent,
                                    const std::shared_ptr<DynamicEDTOctomap> &_distmap_ptr,
                                    const std::vector<Obstacle> &obstacles,
                                    const std::set<int> &grid_obstacles) {
        distmap_ptr = _distmap_ptr;
        updateGridMap(agent.radius, agent.downwash, obstacles, grid_obstacles);
        updateGridMission(agent.current_state.position, agent.desired_goal_point);

        bool success = planImpl(false);
        return success;
    }

    bool GridBasedPlanner::planMAPF(const points_t &start_points,
                                    const points_t &current_points,
                                    const points_t &goal_points,
                                    const std::shared_ptr<DynamicEDTOctomap> &_distmap_ptr,
                                    double agent_radius, double agent_downwash) {
        distmap_ptr = _distmap_ptr;
        updateGridMap(agent_radius, agent_downwash);
        updateGridMission(start_points, current_points, goal_points);

        bool success = planImpl(true);
        return success;
    }

    void GridBasedPlanner::updateGridInfo() {
        double grid_resolution = param.grid_resolution;
        for (int i = 0; i < 3; i++) {
            grid_info.grid_min[i] = -floor((-mission.world_min(i) + SP_EPSILON) / grid_resolution) * grid_resolution;
            grid_info.grid_max[i] = floor((mission.world_max(i) + SP_EPSILON) / grid_resolution) * grid_resolution;
        }
        if (param.world_dimension == 2) {
            grid_info.grid_min[2] = param.world_z_2d;
            grid_info.grid_max[2] = param.world_z_2d;
        }

        for (int i = 0; i < 3; i++) {
            grid_info.dim[i] = (int) round((grid_info.grid_max[i] - grid_info.grid_min[i]) / grid_resolution) + 1;
        }
    }

    void GridBasedPlanner::updateGridMap(double agent_radius,
                                         double agent_downwash,
                                         const std::vector<Obstacle> &obstacles,
                                         const std::set<int> &grid_obstacles) {
        // Initialize gridmap
        grid_map.grid.resize(grid_info.dim[0]);
        for (int i = 0; i < grid_info.dim[0]; i++) {
            grid_map.grid[i].resize(grid_info.dim[1]);
            for (int j = 0; j < grid_info.dim[1]; j++) {
                grid_map.grid[i][j].resize(grid_info.dim[2]);
                for (int k = 0; k < grid_info.dim[2]; k++) {
                    grid_map.grid[i][j][k] = GP_EMPTY;
                }
            }
        }

        // Update distmap to gridmap
        if (distmap_ptr != nullptr) {
            point3d delta(0.5 * param.world_resolution, 0.5 * param.world_resolution, 0.5 * param.world_resolution);
            for (int i = 0; i < grid_info.dim[0]; i++) {
                for (int j = 0; j < grid_info.dim[1]; j++) {
                    for (int k = 0; k < grid_info.dim[2]; k++) {
                        float dist;
                        point3d search_point, closest_point;
                        search_point = gridNodeToPoint3D(GridNode(i, j, k));
                        distmap_ptr->getDistanceAndClosestObstacle(search_point, dist, closest_point);
                        Box closest_cell(closest_point - delta, closest_point + delta);
                        closest_point = closest_cell.closestPoint(search_point);

                        // Due to numerical error of getDistance function, explicitly compute distance to obstacle
                        double dist_to_obs = LInfinityDistance(search_point, closest_point);
                        if (dist_to_obs < agent_radius - SP_EPSILON_FLOAT) {
                            grid_map.grid[i][j][k] = GP_OCCUPIED;
                        }
                    }
                }
            }
        }


        double grid_resolution = param.grid_resolution;
        for (int oi: grid_obstacles) {
            std::vector<point3d> grid_obstacle_positions;
            if (obstacles[oi].type == ObstacleType::AGENT) {
                grid_obstacle_positions.emplace_back(obstacles[oi].position);
            } else {
                double sample_dt = 0.1;
                size_t total_sample = floor(param.obs_uncertainty_horizon / sample_dt) + 1;
                for (size_t i = 0; i < total_sample; i++) {
                    point3d obs_position = obstacles[oi].position + obstacles[oi].velocity * i * sample_dt;
                    grid_obstacle_positions.emplace_back(obs_position);
                }
            }

            for (const auto &obs_position: grid_obstacle_positions) {
                int obs_i, obs_j, obs_k = 0;
                // Update higher priority agent as an obstacle to gridmap
                obs_i = (int) round(
                        (obs_position.x() - grid_info.grid_min[0] + SP_EPSILON) / grid_resolution);
                obs_j = (int) round(
                        (obs_position.y() - grid_info.grid_min[1] + SP_EPSILON) / grid_resolution);
                if (param.world_dimension != 2) {
                    obs_k = (int) round(
                            (obs_position.z() - grid_info.grid_min[2] + SP_EPSILON) / grid_resolution);
                }

                double obstacle_radius, downwash, dist;
                if (obstacles[oi].type == ObstacleType::AGENT) {
                    obstacle_radius = obstacles[oi].radius;
                    downwash = (agent_radius * agent_downwash + obstacles[oi].radius * obstacles[oi].downwash) /
                               (agent_radius + obstacles[oi].radius);
                } else {
                    obstacle_radius = obstacles[oi].radius +
                                      0.5 * obstacles[oi].max_acc * param.obs_uncertainty_horizon *
                                      param.obs_uncertainty_horizon;
                    // obstacle_radius = obstacles[oi].radius;
                    downwash = (agent_radius + obstacles[oi].radius * obstacles[oi].downwash) /
                               (agent_radius + obstacles[oi].radius);
                }

                int size_xy, size_z;
                size_xy = ceil((agent_radius + obstacle_radius) / grid_resolution);
                size_z = ceil((agent_radius * agent_downwash + obstacles[oi].radius * obstacles[oi].downwash) /
                              grid_resolution);

                for (int i = std::max(obs_i - size_xy, 0);
                     i <= std::min(obs_i + size_xy, grid_info.dim[0] - 1); i++) {
                    for (int j = std::max(obs_j - size_xy, 0);
                         j <= std::min(obs_j + size_xy, grid_info.dim[1] - 1); j++) {
                        for (int k = std::max(obs_k - size_z, 0);
                             k <= std::min(obs_k + size_z, grid_info.dim[2] - 1); k++) {
                            if (grid_map.grid[i][j][k] != GP_OCCUPIED) {
                                point3d point = gridNodeToPoint3D(GridNode(i, j, k));
                                dist = ellipsoidalDistance(point, obs_position, downwash);
                                if (dist < agent_radius + obstacle_radius) {
                                    grid_map.grid[i][j][k] = GP_OCCUPIED;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void GridBasedPlanner::updateGridMission(const point3d &start_point,
                                             const point3d &goal_point) {
        grid_mission.n_agents = 1;
        grid_mission.current_points.resize(1);
        grid_mission.goal_points.resize(1);
        grid_mission.current_points[0] = point3DToGridVector(start_point);
        grid_mission.goal_points[0] = point3DToGridVector(goal_point);

        if (param.world_dimension == 2) {
            grid_mission.current_points[0][2] = 0;
            grid_mission.goal_points[0][2] = 0;
        }

        if (grid_map.getValue(grid_mission.current_points[0]) == GP_OCCUPIED) {
//            ROS_WARN_STREAM("[GridBasedPlanner] Start point of agent " << std::to_string(agent_id) << " is occluded");
            double min_dist = SP_INFINITY;
            GridNode closest_point = grid_mission.current_points[0];
            for (int i = -2; i < 3; i++) {
                for (int j = -2; j < 3; j++) {
                    for (int k = 2 - param.world_dimension; k < param.world_dimension - 1; k++) {
                        GridNode candidate_point = grid_mission.current_points[0] + GridNode(i, j, k);
                        if (not isOccupied(grid_map, candidate_point)) {
                            point3d candidate_point3d = gridNodeToPoint3D(candidate_point);
                            double dist = candidate_point3d.distance(start_point);
                            if (dist < min_dist) {
                                min_dist = dist;
                                closest_point = candidate_point;
                            }
                        }
                    }
                }
            }
            grid_mission.current_points[0] = closest_point;

            if (grid_map.getValue(grid_mission.current_points[0]) == GP_OCCUPIED) {
//                ROS_WARN_STREAM("[GridBasedPlanner] Start point of agent is occluded again");
                grid_map.setValue(grid_mission.current_points[0], GP_EMPTY);
            }

        }

        if (isValid(grid_mission.goal_points[0]) and
            grid_map.getValue(grid_mission.goal_points[0]) == GP_OCCUPIED) {
//                ROS_WARN_STREAM("[GridBasedPlanner] Start point of agent " << std::to_string(agent_id) << " is occluded again");
            grid_map.setValue(grid_mission.goal_points[0], GP_EMPTY);
        }
    }

    void GridBasedPlanner::updateGridMission(const points_t &start_points,
                                             const points_t &current_points,
                                             const points_t &goal_points) {
        grid_mission.n_agents = current_points.size();
        grid_mission.start_points.resize(grid_mission.n_agents);
        grid_mission.current_points.resize(grid_mission.n_agents);
        grid_mission.goal_points.resize(grid_mission.n_agents);
        for (size_t i = 0; i < grid_mission.n_agents; i++) {
            grid_mission.start_points[i] = point3DToGridVector(start_points[i]);
            grid_mission.current_points[i] = point3DToGridVector(current_points[i]);
            grid_mission.goal_points[i] = point3DToGridVector(goal_points[i]);

            if (param.world_dimension == 2) {
                grid_mission.current_points[i][2] = 0;
                grid_mission.goal_points[i][2] = 0;
            }

            if (isValid(grid_mission.current_points[i]) and
                grid_map.getValue(grid_mission.current_points[i]) == GP_OCCUPIED) {
                ROS_WARN_STREAM("[GridBasedPlanner] Start point (" << current_points[i] << ") is occluded");
                grid_map.setValue(grid_mission.current_points[i], GP_EMPTY);
            }

            if (isValid(grid_mission.goal_points[i]) and
                grid_map.getValue(grid_mission.goal_points[i]) == GP_OCCUPIED) {
                ROS_WARN_STREAM("[GridBasedPlanner] Goal point (" << goal_points[i] << ") is occluded");
                grid_map.setValue(grid_mission.goal_points[i], GP_EMPTY);
            }
        }
    }

    bool GridBasedPlanner::isValid(const GridNode &grid_node) {
        for (int i = 0; i < 3; i++) {
            if (grid_node[i] < 0 or grid_node[i] >= grid_info.dim[i]) {
                return false;
            }
        }

        return true;
    }

    bool GridBasedPlanner::isOccupied(const GridMap &map, const GridNode &grid_node) {
        for (int i = 0; i < 3; i++) {
            if (grid_node[i] < 0 || grid_node[i] > grid_info.dim[i] - 1) {
                return true;
            }
        }
        return map.getValue(grid_node) == GP_OCCUPIED;
    }

    bool GridBasedPlanner::planImpl(bool is_mapf) {
        std::vector<gridpath_t> grid_paths;
        bool success;
        if (is_mapf) {
            grid_paths = runMAPF(grid_map, grid_mission);
            success = !grid_paths.empty();
            plan_result.n_agents = grid_mission.n_agents;
        }

        if (success) {
            plan_result.paths.resize(plan_result.n_agents);
            for (size_t i = 0; i < plan_result.n_agents; i++) {
                plan_result.paths[i] = gridPathToPath(grid_paths[i]);
            }
        }

        return success;
    }

    std::vector<gridpath_t> GridBasedPlanner::runMAPF(const GridMap &grid_map, const GridMission &grid_mission) {
        MAPF::Problem P = MAPF::Problem(grid_map.grid,
                                        grid_mission.n_agents,
                                        gridNodesToArrays(grid_mission.start_points),
                                        gridNodesToArrays(grid_mission.current_points),
                                        gridNodesToArrays(grid_mission.goal_points));
        std::unique_ptr<MAPF::Solver> solver;
        if (param.mapf_mode == MAPFMode::PIBT) {
            solver = std::make_unique<MAPF::PIBT>(&P);
        } else if (param.mapf_mode == MAPFMode::ECBS) {
            solver = std::make_unique<MAPF::ECBS>(&P);
        } else {
            throw std::invalid_argument("[GridBasedPlanner] Invalid MAPF mode");
        }

        solver->solve();

        MAPF::Plan plan = solver->getSolution();
        std::vector<gridpath_t> grid_paths;
        if (not plan.empty()) {
            grid_paths.resize(grid_mission.n_agents);
            for (size_t t = 0; t < plan.size(); t++) {
                for (size_t i = 0; i < grid_mission.n_agents; i++) {
                    GridNode grid_node(plan.get(t, i)->pos.x,
                                       plan.get(t, i)->pos.y,
                                       0);
                    grid_paths[i].emplace_back(grid_node);
                }
            }
        }

        // Delete repeated path to prevent deadlock
        size_t repeated_interval = 0;
        for(size_t i = 1; i < grid_paths[0].size(); i++) {
            bool repeated = true;
            for (const auto &grid_path: grid_paths) {
                if (grid_path[0] != grid_path[i]) {
                    repeated = false;
                    break;
                }
            }
            if (repeated) {
                repeated_interval = i;
            }
        }
        for(size_t ri = 0; ri < repeated_interval; ri++){
            for(size_t i = 0; i < grid_mission.n_agents; i++){
                grid_paths[i].erase(grid_paths[i].begin());
            }
        }

        return grid_paths;
    }

    points_t GridBasedPlanner::gridPathToPath(const gridpath_t &grid_path) const {
        points_t path;
        for (auto &grid_point: grid_path) {
            path.emplace_back(gridNodeToPoint3D(grid_point));
        }
        return path;
    }

    point3d GridBasedPlanner::gridNodeToPoint3D(const GridNode &grid_vector) const {
        double grid_resolution = param.grid_resolution;
        if(param.world_dimension == 3){
            return point3d(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                           grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                           grid_info.grid_min[2] + grid_vector[2] * grid_resolution);
        }
        else{
            return point3d(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                           grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                           param.world_z_2d);
        }

    }

    point3d GridBasedPlanner::gridNodeToPoint3D(const GridNode &grid_vector, int dimension) const {
        double grid_resolution = param.grid_resolution;
        point3d point;
        if (dimension == 2) {
            point = point3d(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                            grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                            param.world_z_2d);
        } else {
            point = point3d(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                            grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                            grid_info.grid_min[2] + grid_vector[2] * grid_resolution);
        }

        return point;
    }

    std::vector<std::array<int, 3>> GridBasedPlanner::gridNodesToArrays(const GridNodes &grid_nodes) const {
        size_t length = grid_nodes.size();
        std::vector<std::array<int, 3>> arrays;
        arrays.resize(length);

        for (size_t i = 0; i < length; i++) {
            arrays[i] = grid_nodes[i].toArray();
        }

        return arrays;
    }

    GridNode GridBasedPlanner::point3DToGridVector(const point3d &point) const {
        GridNode grid_vector;
        for (int i = 0; i < 3; i++) {
            grid_vector[i] = (int) round((point(i) - grid_info.grid_min[i]) / param.grid_resolution);
            if (grid_vector[i] < 0) {
                grid_vector[i] = 0;
            } else if (grid_vector[i] > grid_info.dim[i] - 1) {
                grid_vector[i] = grid_info.dim[i] - 1;
            }
        }

        return grid_vector;
    }

    points_t GridBasedPlanner::getPath(size_t i) const {
        return plan_result.paths[i];
    }

    points_t GridBasedPlanner::getFreePoints() const {
        //TODO: too naive, save them when generate map?
        points_t free_points;
        for (int i = 0; i < grid_info.dim[0]; i++) {
            for (int j = 0; j < grid_info.dim[1]; j++) {
                for (int k = 0; k < grid_info.dim[2]; k++) {
                    if (grid_map.grid[i][j][k] == GP_EMPTY) {
                        free_points.emplace_back(gridNodeToPoint3D(GridNode(i, j, k)));
                    }
                }
            }
        }

        return free_points;
    }

    points_t GridBasedPlanner::getOccupiedPoints() const {
        points_t occupied_points;
        for (int i = 0; i < grid_info.dim[0]; i++) {
            for (int j = 0; j < grid_info.dim[1]; j++) {
                for (int k = 0; k < grid_info.dim[2]; k++) {
                    if (grid_map.grid[i][j][k] == GP_OCCUPIED) {
                        occupied_points.emplace_back(gridNodeToPoint3D(GridNode(i, j, k)));
                    }
                }
            }
        }

        return occupied_points;
    }

    visualization_msgs::MarkerArray GridBasedPlanner::occupiedPointsToMsg(std::string frame_id) const {
        points_t occupied_points = getOccupiedPoints();

        visualization_msgs::MarkerArray msg_grid_occupied_points;
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        int marker_id = 0;
        for (const auto &point: occupied_points) {
            marker.id = marker_id++;
            marker.pose.position = point3DToPointMsg(point);
            marker.pose.orientation = defaultQuaternion();
            msg_grid_occupied_points.markers.emplace_back(marker);
        }

        return msg_grid_occupied_points;
    }

    visualization_msgs::MarkerArray GridBasedPlanner::pathToMarkerMsg(int agent_id,
                                                                      std::string frame_id,
                                                                      std_msgs::ColorRGBA color) const {


        visualization_msgs::MarkerArray msg_grid_path_vis;
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color = color;
        marker.color.a = 0.2;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        if (plan_result.paths.empty()) {
            return msg_grid_path_vis;
        }

        int marker_id = 0;
        for (const auto &point: plan_result.paths[0]) {
            marker.ns = std::to_string(agent_id);
            marker.id = marker_id++;
            marker.pose.position = point3DToPointMsg(point);
            marker.pose.orientation = defaultQuaternion();
            msg_grid_path_vis.markers.emplace_back(marker);
        }

        return msg_grid_path_vis;
    }

    point3d GridBasedPlanner::findLOSFreeGoal(const point3d &current_position, const point3d &goal_position,
                                              double agent_radius) {
        point3d los_free_goal = current_position;

        if (distmap_ptr != nullptr) {
            points_t path = plan_result.paths[0];
            path.emplace_back(goal_position);

            for (const auto &point: path) {
                bool is_safe = castRay(current_position, point, agent_radius + 0.5 *
                                                                               param.world_resolution); // add 0.5 * param.grid_resolution to avoid numerical error.
//                bool is_safe = castRay(current_position, point, agent_radius);

                if (is_safe) {
                    los_free_goal = point;
                } else {
                    break;
                }
            }

            if (los_free_goal.distance(current_position) < SP_EPSILON_FLOAT and path.size() > 2) {
                los_free_goal = path[1];
            }
        } else {
            los_free_goal = goal_position;
        }

        return los_free_goal;
    }

    bool GridBasedPlanner::castRay(const point3d &current_position,
                                   const point3d &goal_position,
                                   double agent_radius) {
        double safe_dist_curr, safe_dist_goal, dist_to_goal, dist_threshold;
        dist_to_goal = (current_position - goal_position).norm();
        dist_threshold = sqrt(0.25 * dist_to_goal * dist_to_goal + agent_radius * agent_radius);

//        safe_dist_curr = distmap_ptr->getDistance(current_position);
//        safe_dist_goal = distmap_ptr->getDistance(goal_position);

        float dist;
        point3d closest_point;
        distmap_ptr->getDistanceAndClosestObstacle(current_position, dist, closest_point);
        safe_dist_curr = current_position.distance(closest_point);

        distmap_ptr->getDistanceAndClosestObstacle(goal_position, dist, closest_point);
        safe_dist_goal = goal_position.distance(closest_point);

        if (safe_dist_curr < agent_radius + 0.5 * param.world_resolution - SP_EPSILON_FLOAT) {
            return false;
        }
        if (safe_dist_goal < agent_radius + 0.5 * param.world_resolution - SP_EPSILON_FLOAT) {
            return false;
        }
        if (dist_threshold < param.world_max_dist and
            safe_dist_curr > dist_threshold and safe_dist_goal > dist_threshold) {
            return true;
        }

        point3d mid_position = (current_position + goal_position) * 0.5;
        return castRay(current_position, mid_position, agent_radius)
               && castRay(mid_position, goal_position, agent_radius);
    }
}