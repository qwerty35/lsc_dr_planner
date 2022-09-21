#ifndef LSC_PLANNER_COLLISION_CONSTRAINTS_HPP
#define LSC_PLANNER_COLLISION_CONSTRAINTS_HPP

#include <vector>
#include <octomap/octomap_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <utility>
#include <trajectory.hpp>
#include <convhull_3d/convhull_3d.h>

namespace DynamicPlanning {
    // Linear Safe Corridor
    // LSC = {c \in R^3 | (c - c_obs).dot(normal_vector) - d > 0}
    class LSC {
    public:
        LSC() = default;

        LSC(const point3d &obs_control_point,
            const point3d &normal_vector,
            double d);

        [[nodiscard]] visualization_msgs::Marker convertToMarker(double agent_radius,
                                                                 const std::string &world_frame_id) const;

        point3d obs_control_point;
        point3d normal_vector;
        double d = 0;
    };

    typedef std::vector<LSC> LSCs;

    // Axis aligned bounding box
    // Box = {c \in R^3 | box_min < |c| < box_max}
    class Box {
    public:
        point3d box_min;
        point3d box_max;

        Box() = default;

        Box(const point3d &box_min, const point3d &box_max);

        [[nodiscard]] LSCs convertToLSCs(int dim) const;

        [[nodiscard]] visualization_msgs::Marker convertToMarker(double agent_radius,
                                                                 const std::string &world_frame_id) const;

        [[nodiscard]] bool isPointInBox(const point3d &point) const;

        [[nodiscard]] bool isPointInBoxStrictly(const point3d &point) const;

        [[nodiscard]] bool isLineInBox(const Line &line) const;

        [[nodiscard]] bool isSegmentInBox(const Segment<point3d> &segment) const;

        [[nodiscard]] bool isSegmentInBox(const Segment<point3d> &segment, double margin) const;

        [[nodiscard]] bool isInOtherBox(const point3d &world_min, const point3d &world_max, double margin) const;

        [[nodiscard]] bool isSuperSetOfConvexHull(const points_t &convex_hull) const;

        [[nodiscard]] bool intersectWith(const Box &other_box) const;

        [[nodiscard]] bool tangentWith(const Box &other_box) const;

        [[nodiscard]] bool include(const Box &other_box) const;

        [[nodiscard]] Box unify(const Box &other_box) const;

        [[nodiscard]] Box intersection(const Box &other_box) const;

        [[nodiscard]] point3d closestPoint(const point3d &point) const;

        [[nodiscard]] double distanceToPoint(const point3d &point) const;

        [[nodiscard]] double distanceToInnerPoint(const point3d &point) const;

        [[nodiscard]] double raycastFromInnerPoint(const point3d &point, const point3d &direction) const;

        [[nodiscard]] double raycastFromInnerPoint(const point3d &point, const point3d &direction,
                                                   point3d &surface_direction) const;

        [[nodiscard]] points_t getVertices(int dim, double z_2d) const;

        [[nodiscard]] lines_t getEdges() const;

        bool operator==(Box &other_sfc) const;
    };

    typedef std::vector<std::vector<std::vector<LSC>>> RSFCs; // [obs_idx][segment_idx][control_point_idx]
    typedef std::vector<Box> SFCs; // [segment_idx]

    class CollisionConstraints {
    public:
        CollisionConstraints(const Param &param, const Mission &mission);

        void initializeSFC(const point3d &agent_position, double agent_radius);

        void initializeLSC(size_t N_obs);

        void constructSFCFromPoint(const point3d &point, const point3d &goal_point, double agent_radius);

        void constructSFCFromConvexHull(const points_t &convex_hull,
                                        const point3d &next_waypoint,
                                        double agent_radius);

        void constructCommunicationRange(const point3d &next_waypoint);

        std::vector<SFCs> findSFCsCandidates(const std::vector<SFCs> &sfcs_valid);

        std::vector<SFCs> findSFCsCandidates(const std::vector<SFCs> &sfcs_valid,
                                             const Box &sfc_prev,
                                             int m);

        // Getter
        [[nodiscard]] LSC getLSC(int oi, int m, int i) const;

        [[nodiscard]] Box getSFC(int m) const;

        [[nodiscard]] size_t getObsSize() const;

        [[nodiscard]] std::set<int> getDynamicObstacles() const;

        [[nodiscard]] bool isDynamicObstacle(int oi) const;

        [[nodiscard]] bool slackObstaclesEmpty() const;

        // Setter
        void setDistmap(std::shared_ptr<DynamicEDTOctomap> distmap_ptr);

        void setOctomap(std::shared_ptr<octomap::OcTree> octree_ptr);

        void setLSC(int oi, int m,
                    const points_t &obs_control_points,
                    const point3d &normal_vector,
                    const std::vector<double> &ds);

        void setLSC(int oi, int m,
                    const points_t &obs_control_points,
                    const point3d &normal_vector,
                    double d);

        void setLSC(int oi, int m,
                    const point3d &obs_point,
                    const point3d &normal_vector,
                    double d);

        void setSFC(int m, const Box &sfc);

        // Converter
        [[nodiscard]] visualization_msgs::MarkerArray convertLSCsToMarkerArrayMsg(
                const std::vector<Obstacle> &obstacles,
                const std::vector<std_msgs::ColorRGBA> &colors,
                double agent_radius) const;

        [[nodiscard]] visualization_msgs::MarkerArray convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA &color,
                                                                                  double agent_radius) const;

        [[nodiscard]] visualization_msgs::MarkerArray
        feasibleRegionToMarkerArrayMsg(int agent_id, const std_msgs::ColorRGBA &color) const;

    private:
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;
        std::shared_ptr<octomap::OcTree> octree_ptr;
        Mission mission;
        Param param;

        RSFCs lscs; // Safe corridor to avoid agents and dynamic obstacles
        SFCs sfcs; // Safe corridor to avoid static obstacles
        Box communication_range;
        std::set<int> dynamic_obstacle_indices; // Set of indices of dynamic obstacles

        bool expandSFCFromPoint(const point3d &point, const point3d &goal_point,
                                const Box &prev_sfc, double agent_radius, Box &expanded_sfc);

        bool expandSFCFromConvexHull(const points_t &convex_hull, double agent_radius, Box &expanded_sfc);

        bool expandSFCFromConvexHull(const points_t &convex_hull, const Box &prev_sfc,
                                     double agent_radius, Box &expanded_sfc);

        bool isObstacleInSFC(const Box &initial_sfc, double margin);

        bool isSFCInBoundary(const Box &sfc, double margin);

        bool expandSFC(const Box &initial_sfc, double margin, Box &expanded_sfc);

        bool expandSFC(const Box &initial_sfc, const point3d &goal_point, double margin, Box &expanded_sfc);

        [[nodiscard]] points_t findFeasibleVertices(int m, int control_point_idx) const;

        static visualization_msgs::Marker convexHullToMarkerMsg(const points_t &convex_hull,
                                                                const std_msgs::ColorRGBA &color,
                                                                const std::string &world_frame_id);

        static std::vector<int> setAxisCand(const Box &box, const octomap::point3d &goal_point);
    };
}


#endif //LSC_PLANNER_COLLISION_CONSTRAINTS_HPP
