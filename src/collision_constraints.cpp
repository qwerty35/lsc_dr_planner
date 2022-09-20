#define CONVHULL_3D_ENABLE
#include <collision_constraints.hpp>

namespace DynamicPlanning {
    LSC::LSC(const point3d &_obs_control_point,
             const point3d &_normal_vector,
             double _d)
            : obs_control_point(_obs_control_point), normal_vector(_normal_vector), d(_d) {}

    visualization_msgs::Marker LSC::convertToMarker(double agent_radius, const std::string &world_frame_id) const {
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id;
        msg_marker.type = visualization_msgs::Marker::CUBE;
        msg_marker.action = visualization_msgs::Marker::ADD;

        double box_scale = 40;
        msg_marker.scale.x = box_scale;
        msg_marker.scale.y = box_scale;
        msg_marker.scale.z = box_scale;

        double distance = -(d - agent_radius) + box_scale / 2;
        Eigen::Vector3d V3d_normal_vector(normal_vector.x(), normal_vector.y(), normal_vector.z());
        Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, -V3d_normal_vector);

        msg_marker.pose.position = point3DToPointMsg(obs_control_point - normal_vector * distance);
        msg_marker.pose.orientation = quaternionToQuaternionMsg(q);

        return msg_marker;
    }

    Box::Box(const point3d &_box_min, const point3d &_box_max) {
        box_min = _box_min;
        box_max = _box_max;
    }

    LSCs Box::convertToLSCs(int dim) const {
        point3d normal_vector_min, normal_vector_max;
        point3d zero_point = point3d(0, 0, 0);
        double d_min, d_max;

        LSCs lscs;
        lscs.resize(2 * dim);
        for (int i = 0; i < dim; i++) {
            normal_vector_min = zero_point;
            normal_vector_max = zero_point;
            normal_vector_min(i) = 1;
            normal_vector_max(i) = -1;
            d_min = box_min(i);
            d_max = -box_max(i);

            LSC lsc_min(zero_point, normal_vector_min, d_min);
            LSC lsc_max(zero_point, normal_vector_max, d_max);
            lscs[2 * i] = lsc_min;
            lscs[2 * i + 1] = lsc_max;
        }

        return lscs;
    }

    visualization_msgs::Marker Box::convertToMarker(double agent_radius, const std::string &world_frame_id) const {
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id;
        msg_marker.type = visualization_msgs::Marker::LINE_LIST;
        msg_marker.action = visualization_msgs::Marker::ADD;
        msg_marker.pose.position = defaultPoint();
        msg_marker.pose.orientation = defaultQuaternion();
        msg_marker.scale.x = 0.03;

        point3d inflation_vector(agent_radius, agent_radius, agent_radius);
        Box inflated_box = Box(box_min - inflation_vector, box_max + inflation_vector);
        lines_t edges = inflated_box.getEdges();
        for (const auto &edge: edges) {
            msg_marker.points.emplace_back(point3DToPointMsg(edge.start_point));
            msg_marker.points.emplace_back(point3DToPointMsg(edge.end_point));
        }

        return msg_marker;
    }

    bool Box::isPointInBox(const point3d &point) const {
        return point.x() > box_min.x() - SP_EPSILON_FLOAT &&
               point.y() > box_min.y() - SP_EPSILON_FLOAT &&
               point.z() > box_min.z() - SP_EPSILON_FLOAT &&
               point.x() < box_max.x() + SP_EPSILON_FLOAT &&
               point.y() < box_max.y() + SP_EPSILON_FLOAT &&
               point.z() < box_max.z() + SP_EPSILON_FLOAT;
    }

    bool Box::isPointInBoxStrictly(const point3d &point) const {
        return point.x() > box_min.x() + 0.0001 &&
               point.y() > box_min.y() + 0.0001 &&
               point.z() > box_min.z() + 0.0001 &&
               point.x() < box_max.x() - 0.0001 &&
               point.y() < box_max.y() - 0.0001 &&
               point.z() < box_max.z() - 0.0001;
    }

    bool Box::isLineInBox(const Line &line) const {
        return isPointInBox(line.start_point) && isPointInBox(line.end_point);
    }

    bool Box::isSegmentInBox(const Segment<point3d> &segment) const {
        for (const auto &control_point: segment.control_points) {
            if (not isPointInBox(control_point)) {
                return false;
            }
        }

        return true;
    }

    bool Box::isSegmentInBox(const Segment<point3d> &segment, double margin) const {
        point3d delta = point3d(1, 1, 1) * margin;
        Box sfc_inflated(box_min - delta, box_max + delta);
        for (const auto &control_point: segment.control_points) {
            if (not sfc_inflated.isPointInBox(control_point)) {
                return false;
            }
        }

        return true;
    }

    bool Box::isInOtherBox(const point3d &world_min, const point3d &world_max,
                           double margin) const {
        return box_min.x() > world_min.x() + margin - SP_EPSILON &&
               box_min.y() > world_min.y() + margin - SP_EPSILON &&
               box_min.z() > world_min.z() + margin - SP_EPSILON &&
               box_max.x() < world_max.x() - margin + SP_EPSILON &&
               box_max.y() < world_max.y() - margin + SP_EPSILON &&
               box_max.z() < world_max.z() - margin + SP_EPSILON;
    }

    bool Box::isSuperSetOfConvexHull(const points_t &convex_hull) const {
        float min_value, max_value;
        for (int i = 0; i < 3; i++) {
            std::vector<float> points_i;
            for (auto point: convex_hull) {
                points_i.emplace_back(point(i));
            }
            min_value = *std::min_element(points_i.begin(), points_i.end());
            max_value = *std::max_element(points_i.begin(), points_i.end());
            if (min_value < box_min(i) - SP_EPSILON_FLOAT || max_value > box_max(i) + SP_EPSILON_FLOAT) {
                return false;
            }
        }

        return true;
    }

    bool Box::intersectWith(const Box &other_box) const {
        Box inter_sfc = intersection(other_box);
        for (int i = 0; i < 3; i++) {
            if (inter_sfc.box_min(i) > inter_sfc.box_max(i) - SP_EPSILON_FLOAT) {
                return false;
            }
        }
        return true;
    }

    bool Box::tangentWith(const Box &other_box) const {
        Box inter_sfc = intersection(other_box);
        int count = 0;
        for (int i = 0; i < 3; i++) {
            if (inter_sfc.box_min(i) > inter_sfc.box_max(i) + SP_EPSILON_FLOAT) {
                return false;
            } else if (inter_sfc.box_min(i) > inter_sfc.box_max(i) - SP_EPSILON_FLOAT) {
                count++;
            }
        }

        return count == 1;
    }

    bool Box::include(const Box &other_box) const {
        return isPointInBox(other_box.box_min) and isPointInBox(other_box.box_max);
    }

    Box Box::unify(const Box &other_box) const {
        Box unified_box;
        for (int i = 0; i < 3; i++) {
            unified_box.box_min(i) = std::min(box_min(i), other_box.box_min(i));
            unified_box.box_max(i) = std::max(box_max(i), other_box.box_max(i));
        }
        return unified_box;
    }

    Box Box::intersection(const Box &other_box) const {
        Box inter_box;
        for (int i = 0; i < 3; i++) {
            inter_box.box_min(i) = std::max(box_min(i), other_box.box_min(i));
            inter_box.box_max(i) = std::min(box_max(i), other_box.box_max(i));
        }
        return inter_box;
    }

    point3d Box::closestPoint(const point3d &point) const {
        point3d closest_point = point;
        for (int i = 0; i < 3; i++) {
            if (point(i) < box_min(i)) {
                closest_point(i) = box_min(i);
            } else if (point(i) > box_max(i)) {
                closest_point(i) = box_max(i);
            }
        }

        return closest_point;
    }

    double Box::distanceToPoint(const point3d &point) const {
        if (isPointInBox(point)) {
            return 0;
        }

        point3d closest_point = point;
        for (int i = 0; i < 3; i++) {
            if (point(i) < box_min(i)) {
                closest_point(i) = box_min(i);
            } else if (point(i) > box_max(i)) {
                closest_point(i) = box_max(i);
            }
        }

        return (point - closest_point).norm();
    }

    double Box::distanceToInnerPoint(const point3d &point) const {
        if (not isPointInBox(point)) {
            return -1;
        }

        double dist, min_dist = SP_INFINITY;
        for (int i = 0; i < 3; i++) {
            dist = abs(point(i) - box_min(i));
            if (dist < min_dist) {
                min_dist = dist;
            }

            dist = abs(point(i) - box_max(i));
            if (dist < min_dist) {
                min_dist = dist;
            }
        }

        return min_dist;
    }

    double Box::raycastFromInnerPoint(const point3d &inner_point, const point3d &direction) const {
        point3d surface_direction;
        return raycastFromInnerPoint(inner_point, direction, surface_direction);
    }

    double Box::raycastFromInnerPoint(const point3d &inner_point,
                                      const point3d &direction,
                                      point3d &surface_direction) const {
        if (not isPointInBox(inner_point)) {
            point3d delta = point3d(1, 1, 1) * 0.1;
            Box sfc_inflated(box_min - delta, box_max + delta);
            return sfc_inflated.raycastFromInnerPoint(inner_point, direction, surface_direction);
        }

        double a, b, k, min_dist = SP_INFINITY;
        for (int i = 0; i < 3; i++) {
            point3d n_surface;
            n_surface(i) = 1;
            a = (box_min - inner_point).dot(n_surface);
            b = direction.dot(n_surface);
            if (abs(b) < SP_EPSILON_FLOAT) {
                continue;
            }

            k = a / b;
            if (k > 0 and k < min_dist) {
                min_dist = k;
                surface_direction = n_surface;
            }
        }

        for (int i = 0; i < 3; i++) {
            point3d n_surface;
            n_surface(i) = -1;
            a = (box_max - inner_point).dot(n_surface);
            b = direction.dot(n_surface);
            if (abs(b) < SP_EPSILON_FLOAT) {
                continue;
            }

            k = a / b;
            if (k >= 0 and k < min_dist) {
                min_dist = k;
                surface_direction = n_surface;
            }
        }

        return min_dist;
    }

    points_t Box::getVertices(int dim, double z_2d) const {
        points_t vertices;
        if (dim == 2) {
            point3d point1 = box_min;
            point3d point2 = box_max;
            point1.z() = z_2d;
            point2.z() = z_2d;
            vertices.emplace_back(point1);
            vertices.emplace_back(point2);
            point1.x() = box_max.x();
            point2.x() = box_min.x();
            vertices.emplace_back(point1);
            vertices.emplace_back(point2);
        } else {
            vertices.emplace_back(box_min);
            vertices.emplace_back(box_max);
            for (int i = 0; i < dim; i++) {
                point3d point1 = box_min;
                point3d point2 = box_max;
                point1(i) = box_max(i);
                point2(i) = box_min(i);
                vertices.emplace_back(point1);
                vertices.emplace_back(point2);
            }
        }

        return vertices;
    }

    lines_t Box::getEdges() const {
        lines_t edges;

        // find edges
        point3d vertex1, vertex2, vertex3;

        vertex1 = box_min;
        for (int i = 0; i < 3; i++) {
            vertex2 = box_min;
            vertex2(i) = box_max(i);
            edges.emplace_back(Line(vertex1, vertex2));

            for (int j = 0; j < 3; j++) {
                if (i == j) continue;
                vertex3 = vertex2;
                vertex3(j) = box_max(j);
                edges.emplace_back(Line(vertex2, vertex3));
            }
        }

        vertex2 = box_max;
        for (int i = 0; i < 3; i++) {
            vertex1 = box_max;
            vertex1(i) = box_min(i);
            edges.emplace_back(Line(vertex1, vertex2));
        }

        return edges;
    }

    bool Box::operator==(Box &other_sfc) const {
        return box_min.distance(other_sfc.box_min) < SP_EPSILON_FLOAT and
               box_max.distance(other_sfc.box_max) < SP_EPSILON_FLOAT;
    }

    CollisionConstraints::CollisionConstraints(const Param &param_, const Mission &mission_)
            : param(param_), mission(mission_) {}

    void CollisionConstraints::initializeSFC(const point3d &agent_position, double agent_radius) {
        sfcs.resize(param.M);

        Box initial_sfc, expanded_sfc;
        for (size_t k = 0; k < 3; k++) {
            initial_sfc.box_min(k) = floor(agent_position(k) / param.world_resolution) * param.world_resolution;
            initial_sfc.box_max(k) = ceil(agent_position(k) / param.world_resolution) * param.world_resolution;
        }

        bool success = expandSFC(initial_sfc, agent_radius, expanded_sfc);
        if (not success) {
            throw std::invalid_argument("[CollisionConstraints] Invalid initial SFC");
        }

        for (int m = 0; m < param.M; m++) {
            sfcs[m] = expanded_sfc;
        }
    }

    void CollisionConstraints::initializeLSC(size_t N_obs) {
        lscs.clear();
        lscs.resize(N_obs);
        for (size_t oi = 0; oi < N_obs; oi++) {
            lscs[oi].resize(param.M);
            for (int m = 0; m < param.M; m++) {
                lscs[oi][m].resize(param.n + 1);
            }
        }
    }

    void CollisionConstraints::constructSFCFromPoint(const point3d &point,
                                                     const point3d &goal_point,
                                                     double agent_radius) {
        // Update sfc for segments m < M-1 from previous sfc
        for (int m = 0; m < param.M - 1; m++) {
            sfcs[m] = sfcs[m + 1];
        }

        Box sfc_update;
        bool success = expandSFCFromPoint(point, goal_point, sfcs[param.M - 1], agent_radius, sfc_update);
        if (not success) {
            ROS_WARN("[CollisionConstraints] Cannot find proper SFC, use previous one");
            sfc_update = sfcs[param.M - 1]; // Reuse previous one
        }

        sfcs[param.M - 1] = sfc_update;
    }

    void CollisionConstraints::constructSFCFromConvexHull(const points_t &convex_hull,
                                                          const point3d &next_waypoint,
                                                          double agent_radius) {
        // Update sfc for segments m < M-1 from previous sfc
        for (int m = 0; m < param.M - 1; m++) {
            sfcs[m] = sfcs[m + 1];
        }

        Box sfc_update;
        points_t convex_hull_greedy = convex_hull;
        convex_hull_greedy.emplace_back(next_waypoint);
        bool success = expandSFCFromConvexHull(convex_hull_greedy, agent_radius, sfc_update);
        if (not success) {
            success = expandSFCFromConvexHull(convex_hull, sfcs[param.M - 1],
                                              agent_radius, sfc_update);
            if (not success) {
                ROS_WARN("[CollisionConstraints] Cannot find proper SFC, use previous one");
                sfc_update = sfcs[param.M - 1]; // Reuse previous one
            }
        }

        sfcs[param.M - 1] = sfc_update;
    }

    void CollisionConstraints::constructCommunicationRange(const point3d &next_waypoint) {
        if(param.communication_range > 0){
            point3d delta(0.5 * param.communication_range,
                          0.5 * param.communication_range,
                          0.5 * param.communication_range);
            communication_range.box_min = next_waypoint - delta;
            communication_range.box_max = next_waypoint + delta;
        }
    }

    std::vector<SFCs> CollisionConstraints::findSFCsCandidates(const std::vector<SFCs> &valid_sfcs) {
        Box temp;
        std::vector<SFCs> sfc_candidates = findSFCsCandidates(valid_sfcs, temp, 0);
        for (auto &sfc_cand: sfc_candidates) {
            std::reverse(sfc_cand.begin(), sfc_cand.end());
        }

        return sfc_candidates;
    }

    std::vector<SFCs> CollisionConstraints::findSFCsCandidates(const std::vector<SFCs> &valid_sfcs,
                                                               const Box &sfc_prev,
                                                               int m) {
        std::vector<SFCs> sfcs_candidates;
        if (m == param.M) {
            SFCs sfcs_cand;
            sfcs_candidates.emplace_back(sfcs_cand);
            return sfcs_candidates;
        }

        for (const auto &sfc_curr: valid_sfcs[m]) {
            if (m == 0 or sfc_curr.intersectWith(sfc_prev)) {
                std::vector<SFCs> sfcs_candidates_child = findSFCsCandidates(valid_sfcs, sfc_curr, m + 1);
                for (const auto &sfcs_cand_child: sfcs_candidates_child) {
                    SFCs sfcs_cand_curr = sfcs_cand_child;
                    sfcs_cand_curr.emplace_back(sfc_curr);
                    sfcs_candidates.emplace_back(sfcs_cand_curr);
                }
            }
        }

        return sfcs_candidates;
    }

    void CollisionConstraints::clearSlackObstacles() {
        dynamic_obstacle_indices.clear();
    }

    void CollisionConstraints::addSlackObstacle(int obs_idx) {
        dynamic_obstacle_indices.emplace(obs_idx);
    }

    LSC CollisionConstraints::getLSC(int oi, int m, int i) const {
        return lscs[oi][m][i];
    }

    Box CollisionConstraints::getSFC(int m) const {
        return sfcs[m];
    }

    size_t CollisionConstraints::getObsSize() const {
        return lscs.size();
    }

    std::set<int> CollisionConstraints::getDynamicObstacles() const {
        return dynamic_obstacle_indices;
    }

    bool CollisionConstraints::isDynamicObstacle(int oi) const {
        return dynamic_obstacle_indices.find(oi) != dynamic_obstacle_indices.end();
    }

    bool CollisionConstraints::slackObstaclesEmpty() const {
        return dynamic_obstacle_indices.empty();
    }

    void CollisionConstraints::setDistmap(std::shared_ptr<DynamicEDTOctomap> distmap_ptr_) {
        distmap_ptr = distmap_ptr_;
    }

    void CollisionConstraints::setOctomap(std::shared_ptr<octomap::OcTree> octree_ptr_) {
        octree_ptr = octree_ptr_;
    }

    void CollisionConstraints::setLSC(int oi, int m,
                                      const points_t &obs_control_points,
                                      const vector3d &normal_vector,
                                      const std::vector<double> &ds) {
        for (int i = 0; i < param.n + 1; i++) {
            lscs[oi][m][i] = LSC(obs_control_points[i], normal_vector, ds[i]);
        }
    }

    void CollisionConstraints::setLSC(int oi, int m,
                                      const points_t &obs_control_points,
                                      const vector3d &normal_vector,
                                      double d) {
        for (int i = 0; i < param.n + 1; i++) {
            lscs[oi][m][i] = LSC(obs_control_points[i], normal_vector, d);
        }
    }

    void CollisionConstraints::setLSC(int oi, int m,
                                      const point3d &obs_point,
                                      const vector3d &normal_vector,
                                      double d) {
        for (int i = 0; i < param.n + 1; i++) {
            lscs[oi][m][i] = LSC(obs_point, normal_vector, d);
        }
    }

    void CollisionConstraints::setSFC(int m, const Box &sfc) {
        sfcs[m] = sfc;
    }

//    visualization_msgs::MarkerArray CollisionConstraints::convertToMarkerArrayMsg(
//            const std::vector<Obstacle> &obstacles,
//            const std::vector<std_msgs::ColorRGBA> &colors,
//            int agent_id, double agent_radius) const {
//        visualization_msgs::MarkerArray msg1 = convertLSCsToMarkerArrayMsg(obstacles, colors, agent_radius);
//        visualization_msgs::MarkerArray msg2 = convertSFCsToMarkerArrayMsg(colors[agent_id], agent_radius);
//        msg1.markers.insert(msg1.markers.end(), msg2.markers.begin(), msg2.markers.end());
//        return msg1;
//    }

    visualization_msgs::MarkerArray CollisionConstraints::convertLSCsToMarkerArrayMsg(
            const std::vector<Obstacle> &obstacles,
            const std::vector<std_msgs::ColorRGBA> &colors,
            double agent_radius) const {
        visualization_msgs::MarkerArray msg_marker_array;

        if (lscs.empty()) {
            return msg_marker_array;
        }

        size_t N_obs = obstacles.size();
        msg_marker_array.markers.clear();
        for (size_t oi = 0; oi < N_obs; oi++) {
            std_msgs::ColorRGBA marker_color;
            if (obstacles[oi].type == AGENT) {
                marker_color = colors[obstacles[oi].id];
                marker_color.a = 0.1;
            } else {
                marker_color.r = 0.0;
                marker_color.g = 0.0;
                marker_color.b = 0.0;
                marker_color.a = 0.1;
            }

            for (size_t m = 0; m < lscs[oi].size(); m++) {
//                visualization_msgs::Marker msg_marker = lscs[oi][m][0].convertToMarker(agent_radius);
                visualization_msgs::Marker msg_marker = lscs[oi][m][0].convertToMarker(0, param.world_frame_id);
                if (obstacles[oi].type == AGENT) {
                    msg_marker.ns = "agent" + std::to_string(m);
                } else if (obstacles[oi].type == DYNAMICOBSTACLE) {
                    msg_marker.ns = "dynamic_obstacle" + std::to_string(m);
                }
                msg_marker.id = m * N_obs + oi;
                msg_marker.color = marker_color;

                msg_marker_array.markers.emplace_back(msg_marker);
            }
        }

        return msg_marker_array;
    }

    visualization_msgs::MarkerArray CollisionConstraints::convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA &color,
                                                                                      double agent_radius) const {
        visualization_msgs::MarkerArray msg_marker_array;

        if (sfcs.empty()) {
            return msg_marker_array;
        }

        msg_marker_array.markers.clear();
        for (size_t m = 0; m < sfcs.size(); m++) {
//            visualization_msgs::Marker msg_marker = sfcs[m].box.convertToMarker(agent_radius);
            visualization_msgs::Marker msg_marker = sfcs[m].convertToMarker(0, param.world_frame_id);
            msg_marker.id = m;
            msg_marker.ns = "SFC" + std::to_string(m);
            msg_marker.color = color;
            msg_marker.color.a = 1.0;
            msg_marker_array.markers.emplace_back(msg_marker);
        }

        return msg_marker_array;
    }

    visualization_msgs::MarkerArray CollisionConstraints::feasibleRegionToMarkerArrayMsg(
            int agent_id,
            const std_msgs::ColorRGBA &color) const {

        visualization_msgs::MarkerArray msg_marker_array;
        if (lscs.empty() and sfcs.empty()) {
            return msg_marker_array;
        }

        for (int m = 0; m < param.M + 1; m++) {
            points_t vertices;
            if(m < param.M){
                vertices = findFeasibleVertices(m, 0);
            } else {
                vertices = findFeasibleVertices(param.M - 1, param.n);
            }

            visualization_msgs::Marker msg_marker = convexHullToMarkerMsg(vertices, color, param.world_frame_id);

            msg_marker.id = agent_id;
            msg_marker.ns = std::to_string(m);
            msg_marker.color = color;
            msg_marker.color.a = 0.4;
            msg_marker_array.markers.emplace_back(msg_marker);

//            int count = 0;
//            for(const auto& vertex: vertices){
//                visualization_msgs::Marker msg_marker;
//                msg_marker.ns = std::to_string(m);
//                msg_marker.id = count;
//                msg_marker.color = color;
//                msg_marker.color.a = 1.0;
//
//                msg_marker.header.frame_id = param.world_frame_id;
//                msg_marker.type = visualization_msgs::Marker::SPHERE;
//                msg_marker.action = visualization_msgs::Marker::ADD;
//                msg_marker.pose.position = point3DToPointMsg(vertex);
//                msg_marker.pose.orientation = defaultQuaternion();
//                msg_marker.scale.x = 0.2;
//                msg_marker.scale.y = 0.2;
//                msg_marker.scale.z = 0.2;
//
//                msg_marker_array.markers.emplace_back(msg_marker);
//                count++;
//            }
        }

        return msg_marker_array;
    }

    bool CollisionConstraints::expandSFCFromPoint(const point3d &point, const point3d &goal_point,
                                                  const Box &prev_sfc, double agent_radius,
                                                  Box &expanded_sfc) {
        // Initialize initial_box
        Box initial_sfc;
        for (size_t k = 0; k < 3; k++) {
            initial_sfc.box_min(k) = floor(point(k) / param.world_resolution) * param.world_resolution;
            initial_sfc.box_max(k) = ceil(point(k) / param.world_resolution) * param.world_resolution;
        }
        if (not prev_sfc.include(initial_sfc)) {
            initial_sfc = prev_sfc.intersection(initial_sfc);
            for (size_t k = 0; k < 3; k++) {
                initial_sfc.box_min(k) = ceil((initial_sfc.box_min(k) - SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
                initial_sfc.box_max(k) = floor((initial_sfc.box_max(k) + SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
            }
        }

        bool success = expandSFC(initial_sfc, goal_point, agent_radius, expanded_sfc);
        if (not success or not expanded_sfc.isPointInBox(point)) {
            ROS_ERROR("????");
//            success = expandSFC(initial_sfc, expanded_sfc, agent_radius);
        }

        return success;
    }

    bool CollisionConstraints::expandSFCFromConvexHull(const points_t &convex_hull,
                                                       double agent_radius,
                                                       Box &expanded_sfc) {
        if (convex_hull.empty()) {
            return false;
        }

        // Initialize initial_box
        Box initial_sfc;
        initial_sfc.box_min = convex_hull[0];
        initial_sfc.box_max = convex_hull[0];
        for (const auto &point: convex_hull) {
            for (int k = 0; k < 3; k++) {
                if (point(k) < initial_sfc.box_min(k)) {
                    initial_sfc.box_min(k) = point(k);
                }
                if (point(k) > initial_sfc.box_max(k)) {
                    initial_sfc.box_max(k) = point(k);
                }
            }
        }

        // Align initial SFC to grid
        for (int k = 0; k < 3; k++) {
            initial_sfc.box_min(k) = round(initial_sfc.box_min(k) / param.world_resolution) * param.world_resolution;
            initial_sfc.box_max(k) = round(initial_sfc.box_max(k) / param.world_resolution) * param.world_resolution;
        }

        bool success = expandSFC(initial_sfc, agent_radius, expanded_sfc);
        if (success and not expanded_sfc.isSuperSetOfConvexHull(convex_hull)) {
            success = false;
        }

        return success;
    }

    bool CollisionConstraints::expandSFCFromConvexHull(const points_t &convex_hull,
                                                       const Box &prev_sfc,
                                                       double agent_radius,
                                                       Box &expanded_sfc) {
        if (convex_hull.empty()) {
            return false;
        }

        // Initialize initial_box
        Box initial_sfc;
        initial_sfc.box_min = convex_hull[0];
        initial_sfc.box_max = convex_hull[0];
        for (const auto &point: convex_hull) {
            for (int k = 0; k < 3; k++) {
                if (point(k) < initial_sfc.box_min(k)) {
                    initial_sfc.box_min(k) = point(k);
                }
                if (point(k) > initial_sfc.box_max(k)) {
                    initial_sfc.box_max(k) = point(k);
                }
            }
        }

        // Align initial SFC to grid
        for (int k = 0; k < 3; k++) {
            initial_sfc.box_min(k) = floor(initial_sfc.box_min(k) / param.world_resolution) * param.world_resolution;
            initial_sfc.box_max(k) = ceil(initial_sfc.box_max(k) / param.world_resolution) * param.world_resolution;
        }
        if (not prev_sfc.include(initial_sfc)) {
            initial_sfc = prev_sfc.intersection(initial_sfc);
            for (size_t k = 0; k < 3; k++) {
                initial_sfc.box_min(k) = ceil((initial_sfc.box_min(k) - SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
                initial_sfc.box_max(k) = floor((initial_sfc.box_max(k) + SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
            }
        }

        bool success = expandSFC(initial_sfc, agent_radius, expanded_sfc);
        if (not success or not expanded_sfc.isSuperSetOfConvexHull(convex_hull)) {
            ROS_ERROR("????");
        }

        return success;
    }

    bool CollisionConstraints::isObstacleInSFC(const Box &sfc, double margin) {
        point3d delta(0.5 * param.world_resolution, 0.5 * param.world_resolution, 0.5 * param.world_resolution);
        std::array<int, 3> sfc_size = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            sfc_size[i] =
                    (int) floor((sfc.box_max(i) - sfc.box_min(i) + SP_EPSILON_FLOAT) / param.world_resolution) + 1;
        }

        std::array<size_t, 3> iter = {0, 0, 0};
        for (iter[0] = 0; iter[0] < sfc_size[0]; iter[0]++) {
            for (iter[1] = 0; iter[1] < sfc_size[1]; iter[1]++) {
                for (iter[2] = 0; iter[2] < sfc_size[2]; iter[2]++) {
                    float dist;
                    point3d search_point, closest_point;
                    for (int i = 0; i < 3; i++) {
                        search_point(i) = sfc.box_min(i) + iter[i] * param.world_resolution;
                    }

                    distmap_ptr->getDistanceAndClosestObstacle(search_point, dist, closest_point);
                    Box closest_cell(closest_point - delta, closest_point + delta);
                    closest_point = closest_cell.closestPoint(search_point);
                    double dist_to_obs = LInfinityDistance(closest_point, search_point);
                    if (dist_to_obs < margin + SP_EPSILON_FLOAT) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    bool CollisionConstraints::isSFCInBoundary(const Box &sfc, double margin) {
        return sfc.box_min.x() > mission.world_min.x() + margin - SP_EPSILON_FLOAT &&
               sfc.box_min.y() > mission.world_min.y() + margin - SP_EPSILON_FLOAT &&
               sfc.box_min.z() > mission.world_min.z() + margin - SP_EPSILON_FLOAT &&
               sfc.box_max.x() < mission.world_max.x() - margin + SP_EPSILON_FLOAT &&
               sfc.box_max.y() < mission.world_max.y() - margin + SP_EPSILON_FLOAT &&
               sfc.box_max.z() < mission.world_max.z() - margin + SP_EPSILON_FLOAT;
    }

    bool CollisionConstraints::expandSFC(const Box &initial_sfc, double margin, Box &expanded_sfc) {
        if (isObstacleInSFC(initial_sfc, margin)) {
            return false;
        }

        Box sfc, sfc_cand, sfc_update;
        std::vector<int> axis_cand = {0, 1, 2, 3, 4, 5}; // -x, -y, -z, +x, +y, +z

        int i = -1;
        int axis;
        sfc = initial_sfc;
        while (!axis_cand.empty()) {
            // initialize boxes
            sfc_cand = sfc;
            sfc_update = sfc;

            //check collision update_box only! box_current + box_update = box_cand
            while (isSFCInBoundary(sfc_update, 0) && !isObstacleInSFC(sfc_update, margin)) {
                i++;
                if (i >= axis_cand.size()) {
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                sfc = sfc_cand;
                sfc_update = sfc_cand;

                //expand box_cand and get updated part of box (box_update)
                if (axis < 3) {
                    sfc_update.box_max(axis) = sfc_cand.box_min(axis);
                    sfc_cand.box_min(axis) = sfc_cand.box_min(axis) - param.world_resolution;
                    sfc_update.box_min(axis) = sfc_cand.box_min(axis);
                } else {
                    sfc_update.box_min(axis - 3) = sfc_cand.box_max(axis - 3);
                    sfc_cand.box_max(axis - 3) = sfc_cand.box_max(axis - 3) + param.world_resolution;
                    sfc_update.box_max(axis - 3) = sfc_cand.box_max(axis - 3);
                }
            }
            // if obstacle is in box then do not expand box to the current axis direction
            axis_cand.erase(axis_cand.begin() + i);
            if (i > 0) {
                i--;
            } else {
                i = axis_cand.size() - 1;
            }
        }

        //SFC margin compensation
        double delta = margin - ((int) (margin / param.world_resolution) * param.world_resolution);
        for (int k = 0; k < 3; k++) {
            if (sfc.box_min(k) > mission.world_min(k) + SP_EPSILON_FLOAT) {
                sfc.box_min(k) = sfc.box_min(k) - delta;
            }
            if (sfc.box_max(k) < mission.world_max(k) - SP_EPSILON_FLOAT) {
                sfc.box_max(k) = sfc.box_max(k) + delta;
            }
        }

        expanded_sfc = sfc;
        return true;
    }

    bool CollisionConstraints::expandSFC(const Box &initial_sfc, const point3d &goal_point, double margin,
                                         Box &expanded_sfc) {
        if (isObstacleInSFC(initial_sfc, margin)) {
            return false;
        }

        Box sfc, sfc_cand, sfc_update;
        std::vector<int> axis_cand = setAxisCand(initial_sfc, goal_point);  // -x, -y, -z, +x, +y, +z
        // {0, 1, 2, 3, 4, 5};

        int i = -1;
        int axis;
        sfc = initial_sfc;
        while (!axis_cand.empty()) {
            // initialize boxes
            sfc_cand = sfc;
            sfc_update = sfc;

            //check collision update_box only! box_current + box_update = box_cand
            while (isSFCInBoundary(sfc_update, 0) && !isObstacleInSFC(sfc_update, margin)) {
                i++;
                if (i >= axis_cand.size()) {
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                sfc = sfc_cand;
                sfc_update = sfc_cand;

                //expand box_cand and get updated part of box (box_update)
                if (axis < 3) {
                    sfc_update.box_max(axis) = sfc_cand.box_min(axis);
                    sfc_cand.box_min(axis) = sfc_cand.box_min(axis) - param.world_resolution;
                    sfc_update.box_min(axis) = sfc_cand.box_min(axis);
                } else {
                    sfc_update.box_min(axis - 3) = sfc_cand.box_max(axis - 3);
                    sfc_cand.box_max(axis - 3) = sfc_cand.box_max(axis - 3) + param.world_resolution;
                    sfc_update.box_max(axis - 3) = sfc_cand.box_max(axis - 3);
                }
            }
            // if obstacle is in box then do not expand box to the current axis direction
            axis_cand.erase(axis_cand.begin() + i);
            if (i > 0) {
                i--;
            } else {
                i = axis_cand.size() - 1;
            }
        }

        //SFC margin compensation
        double delta = margin - ((int) (margin / param.world_resolution) * param.world_resolution);
        for (int k = 0; k < 3; k++) {
            if (sfc.box_min(k) > mission.world_min(k) + SP_EPSILON_FLOAT) {
                sfc.box_min(k) = sfc.box_min(k) - delta;
            }
            if (sfc.box_max(k) < mission.world_max(k) - SP_EPSILON_FLOAT) {
                sfc.box_max(k) = sfc.box_max(k) + delta;
            }
        }

        expanded_sfc = sfc;
        return true;
    }

    points_t CollisionConstraints::findFeasibleVertices(int m, int control_point_idx) const {
        // Use brute force method to find vertices of feasible region
        LSCs lc;

        // LSC
        for (size_t oi = 0; oi < lscs.size(); oi++) {
            if (isDynamicObstacle(oi)) {
                continue;
            }
            lc.emplace_back(lscs[oi][m][control_point_idx]);
        }

        // SFC
        if (param.world_use_octomap) {
            LSCs lsc_sfc = sfcs[m].convertToLSCs(3);
            for (const auto &lsc: lsc_sfc) {
                lc.emplace_back(lsc);
            }
        } else {
            Box sfc_world_boundary(mission.world_min, mission.world_max);
            LSCs lscs_world_boundary = sfc_world_boundary.convertToLSCs(3);
            for (const auto &lsc: lscs_world_boundary) {
                lc.emplace_back(lsc);
            }
        }

        // Communication range
        if(param.communication_range > 0){
            LSCs lscs_communication_range = communication_range.convertToLSCs(3);
            for (const auto &lsc: lscs_communication_range) {
                lc.emplace_back(lsc);
            }
        }

        points_t vertices;
        size_t N = lc.size();
        for (size_t i = 0; i < N; i++) {
            for (size_t j = i + 1; j < N; j++) {
                for (size_t k = j + 1; k < N; k++) {
                    Eigen::Matrix3f A;
                    A << lc[i].normal_vector(0), lc[i].normal_vector(1), lc[i].normal_vector(2),
                         lc[j].normal_vector(0), lc[j].normal_vector(1), lc[j].normal_vector(2),
                         lc[k].normal_vector(0), lc[k].normal_vector(1), lc[k].normal_vector(2);
                    Eigen::Vector3f b;
                    b << lc[i].obs_control_point.dot(lc[i].normal_vector) + lc[i].d,
                         lc[j].obs_control_point.dot(lc[j].normal_vector) + lc[j].d,
                         lc[k].obs_control_point.dot(lc[k].normal_vector) + lc[k].d;

                    Eigen::Vector3f x = A.householderQr().solve(b);
                    double relative_error = (A * x - b).norm() / b.norm();
                    if (relative_error < SP_EPSILON_FLOAT) {
                        point3d point(x(0), x(1), x(2));
                        bool feasible_point = true;
                        for (size_t ci = 0; ci < N; ci++) {
                            if ((point - lc[ci].obs_control_point).dot(lc[ci].normal_vector) - lc[ci].d <
                                -SP_EPSILON_FLOAT) {
                                feasible_point = false;
                                continue;
                            }
                        }

                        if (feasible_point) {
                            vertices.emplace_back(point);
                        }
                    }
                }
            }
        }

        return vertices;
    }

//    visualization_msgs::Marker CollisionConstraints::convexHullToMarkerMsg(const points_t& convex_hull,
//                                                                           const std::string& world_frame_id) {
//        visualization_msgs::Marker msg_marker;
//        msg_marker.header.frame_id = world_frame_id;
//        msg_marker.type = visualization_msgs::Marker::LINE_LIST;
//        msg_marker.action = visualization_msgs::Marker::ADD;
//        msg_marker.pose.position = defaultPoint();
//        msg_marker.pose.orientation = defaultQuaternion();
//        msg_marker.scale.x = 0.03;
//
//        if(convex_hull.empty()){
//            return msg_marker;
//        }
//
//        size_t nVertices = convex_hull.size();
//        ch_vertex* vertices;
//        vertices = (ch_vertex*)malloc(nVertices*sizeof(ch_vertex));
//        for (int i = 0; i < nVertices; i++) {
//            vertices[i].x = convex_hull[i].x();
//            vertices[i].y = convex_hull[i].y();
//            vertices[i].z = convex_hull[i].z();
//        }
//
//        int* faceIndices = NULL;
//        int nFaces;
//        convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);
//        /* Where 'faceIndices' is a flat 2D matrix [nFaces x 3] */
//
//        for(int i = 0; i < nFaces; i++){
//            auto vertex1 = vertices[faceIndices[i * 3]];
//            auto vertex2 = vertices[faceIndices[i * 3 + 1]];
//            auto vertex3 = vertices[faceIndices[i * 3 + 2]];
//
//            point3d point1(vertex1.x, vertex1.y, vertex1.z);
//            point3d point2(vertex2.x, vertex2.y, vertex2.z);
//            point3d point3(vertex3.x, vertex3.y, vertex3.z);
//
//            geometry_msgs::Point pointmsg1 = point3DToPointMsg(point1);
//            geometry_msgs::Point pointmsg2 = point3DToPointMsg(point2);
//            geometry_msgs::Point pointmsg3 = point3DToPointMsg(point3);
//
//            msg_marker.points.emplace_back(pointmsg1);
//            msg_marker.points.emplace_back(pointmsg2);
//
//            msg_marker.points.emplace_back(pointmsg2);
//            msg_marker.points.emplace_back(pointmsg3);
//
//            msg_marker.points.emplace_back(pointmsg3);
//            msg_marker.points.emplace_back(pointmsg1);
//        }
//
//        free(vertices);
//        free(faceIndices);
//        return msg_marker;
//    }

    visualization_msgs::Marker CollisionConstraints::convexHullToMarkerMsg(const points_t &convex_hull,
                                                                           const std_msgs::ColorRGBA &color,
                                                                           const std::string &world_frame_id) {
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id;
        msg_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        msg_marker.action = visualization_msgs::Marker::ADD;
        msg_marker.pose.position = defaultPoint();
        msg_marker.pose.orientation = defaultQuaternion();
        msg_marker.scale.x = 1;
        msg_marker.scale.y = 1;
        msg_marker.scale.z = 1;
        std_msgs::ColorRGBA mesh_color = color;
        mesh_color.a = 0.2;


        if (convex_hull.empty()) {
            return msg_marker;
        }

        size_t nVertices = convex_hull.size();
        ch_vertex *vertices;
        vertices = (ch_vertex *) malloc(nVertices * sizeof(ch_vertex));
        for (size_t i = 0; i < nVertices; i++) {
            vertices[i].x = convex_hull[i].x();
            vertices[i].y = convex_hull[i].y();
            vertices[i].z = convex_hull[i].z();
        }

        int *faceIndices = NULL;
        int nFaces;
        convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);
        /* Where 'faceIndices' is a flat 2D matrix [nFaces x 3] */

        for (int i = 0; i < nFaces; i++) {
            auto vertex1 = vertices[faceIndices[i * 3]];
            auto vertex2 = vertices[faceIndices[i * 3 + 1]];
            auto vertex3 = vertices[faceIndices[i * 3 + 2]];

            point3d point1(vertex1.x, vertex1.y, vertex1.z);
            point3d point2(vertex2.x, vertex2.y, vertex2.z);
            point3d point3(vertex3.x, vertex3.y, vertex3.z);

            geometry_msgs::Point pointmsg1 = point3DToPointMsg(point1);
            geometry_msgs::Point pointmsg2 = point3DToPointMsg(point2);
            geometry_msgs::Point pointmsg3 = point3DToPointMsg(point3);

            msg_marker.points.emplace_back(pointmsg1);
            msg_marker.points.emplace_back(pointmsg2);
            msg_marker.points.emplace_back(pointmsg3);
            msg_marker.colors.emplace_back(mesh_color);
        }

        free(vertices);
        free(faceIndices);
        return msg_marker;
    }

    std::vector<int> CollisionConstraints::setAxisCand(const Box &box, const octomap::point3d &goal_point) {
        std::vector<int> axis_cand;
        axis_cand.resize(6);

        octomap::point3d mid_point = (box.box_min + box.box_max) * 0.5;
        octomap::point3d delta = goal_point - mid_point;
        std::vector<int> offsets;
        offsets.emplace_back(delta.x() > 0 ? 3 : 0);
        offsets.emplace_back(delta.y() > 0 ? 3 : 0);
        offsets.emplace_back(delta.z() > 0 ? 3 : 0);
        std::vector<double> values;
        values.emplace_back(abs(delta.x()));
        values.emplace_back(abs(delta.y()));
        values.emplace_back(abs(delta.z()));

        std::vector<int> element_order; // 0 is largest, 2 is smallest
        double max_value = -1;
        double min_value = SP_INFINITY;
        for (int i = 0; i < 3; i++) {
            if (values[i] > max_value) {
                element_order.insert(element_order.begin(), i);
                max_value = values[i];
            } else if (values[i] < min_value) {
                element_order.emplace_back(i);
                min_value = values[i];
            } else {
                element_order.insert(element_order.begin() + 1, i);
            }
        }

        for (int i = 0; i < 3; i++) {
            axis_cand[i] = element_order[i] + offsets[element_order[i]];
            axis_cand[5 - i] = element_order[i] + (3 - offsets[element_order[i]]);
        }

        return axis_cand;
    }
}