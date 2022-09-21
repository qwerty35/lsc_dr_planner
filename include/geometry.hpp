#pragma once
#include <sp_const.hpp>
#include <openGJK/openGJK.hpp>

namespace DynamicPlanning{
    struct Line {
        Line(const point3d &_start_point, const point3d &_end_point)
                : start_point(_start_point), end_point(_end_point) {};

        [[nodiscard]] vector3d direction() const {
            if ((end_point - start_point).norm() < SP_EPSILON_FLOAT) {
                return {0, 0, 0};
            } else {
                return (end_point - start_point).normalized();
            }
        }

        [[nodiscard]] double length() const {
            return start_point.distance(end_point);
        }

        Line operator-(const Line &other) const {
            return {start_point - other.start_point, end_point - other.end_point};
        }

        point3d start_point, end_point;
    };

    typedef std::vector<Line> lines_t;

    static ClosestPoints closestPointsBetweenPointAndLine(const point3d& point,
                                                          const point3d& line_point,
                                                          const point3d& line_direction){
        point3d a, c, line_closest_point;
        a = line_point - point;
        c = a - line_direction * a.dot(line_direction);
        line_closest_point = point + c;

        ClosestPoints closest_points;
        closest_points.dist = c.norm();
        closest_points.closest_point1 = point;
        closest_points.closest_point2 = line_closest_point;

        return closest_points;
    }

    static ClosestPoints closestPointsBetweenPointAndRay(const point3d& point,
                                                         const point3d& ray_start,
                                                         const point3d& ray_direction){
        ClosestPoints closest_points;

        point3d delta_to_start, ray_closest_point;
        delta_to_start = point - ray_start;
        if(delta_to_start.dot(ray_direction) < 0){
            closest_points.dist = delta_to_start.norm();
            closest_points.closest_point1 = point;
            closest_points.closest_point2 = ray_closest_point;
        }
        else{
            closest_points = closestPointsBetweenPointAndLine(point, ray_start, ray_direction);
        }

        return closest_points;
    }

    //closest_point: closest point in line
    static ClosestPoints closestPointsBetweenPointAndLineSegment(const point3d& point,
                                                                 const Line& line){
        point3d a, b, c, n_line, rel_closest_point;
        double dist, dist_min;
        a = line.start_point - point;
        b = line.end_point - point;

        if (a == b) {
            dist_min = a.norm();
            rel_closest_point = a;
        } else {
            dist_min = a.norm();
            rel_closest_point = a;

            dist = b.norm();
            if (dist_min > dist) {
                dist_min = dist;
                rel_closest_point = b;
            }

            n_line = (b - a).normalized();
            c = a - n_line * a.dot(n_line);
            dist = c.norm();
            if ((c - a).dot(c - b) < 0 && dist_min > dist) {
                dist_min = dist;
                rel_closest_point = c;
            }
        }

        ClosestPoints closest_points;
        closest_points.dist = dist_min;
        closest_points.closest_point1 = point;
        closest_points.closest_point2 = rel_closest_point + point;

        return closest_points;
    }

    // find min((line1_start + alpha(line1_goal-line1_start) - (line2_start + alpha(line2_goal-line2_start)))
    static ClosestPoints closestPointsBetweenLinePaths(const Line& line1,
                                                       const Line& line2)
    {
        point3d origin;
        Line rel_path = line2 - line1;
        origin = point3d(0, 0, 0);

        ClosestPoints rel_closest_points, closest_points;
        double alpha, line_length;
        rel_closest_points = closestPointsBetweenPointAndLineSegment(origin, rel_path);
        line_length = rel_path.length();
        if(line_length > 0){
            alpha = (rel_closest_points.closest_point2 - rel_path.start_point).norm() / line_length;
        }
        else{
            alpha = 0;
        }

        closest_points.dist = rel_closest_points.dist;
        closest_points.closest_point1 = line1.start_point + (line1.end_point - line1.start_point) * alpha;
        closest_points.closest_point2 = line2.start_point + (line2.end_point - line2.start_point) * alpha;
        return closest_points;
    }

    static ClosestPoints closestPointsBetweenLines(const Line& line1, const Line& line2) {
        if (line1.start_point == line1.end_point) {
            throw std::invalid_argument(
                    "[Geometry] Invalid input for distance between lines, line1 start and goal is same");
        }
        if (line2.start_point == line2.end_point) {
            throw std::invalid_argument(
                    "[Geometry] Invalid input for distance between lines, line2 start and goal is same");
        }

        ClosestPoints closest_points;

        // get closest point of line segment(a->b) from origin
        point3d n1, n2, n3, delta;
        n1 = (line1.end_point - line1.start_point).normalized();
        n2 = (line2.end_point - line2.start_point).normalized();
        if (n1.distance(n2) < SP_EPSILON_FLOAT or n1.distance(-n2) < SP_EPSILON_FLOAT) {
            delta = line2.start_point - line1.start_point;
            delta = delta - n1 * (delta.dot(n1));
            closest_points.dist = delta.norm();
            closest_points.closest_point1 = line1.start_point;
            closest_points.closest_point2 = line1.start_point + delta;
        } else {
            delta = line2.start_point - line1.start_point;
            n3 = (n2.cross(n1)).normalized();

            // get alpha1,2,3 by solving line1_start + alpha1 * n1 + alpha3 * n3 = line2_start + alpha2 * n2
            Eigen::Matrix3f A;
//            A <<  n1(0),  n1(1),  n1(2),
//                 -n2(0), -n2(1), -n2(2),
//                  n3(0),  n3(1),  n3(2);
            A << n1(0), -n2(0), n3(0),
                    n1(1), -n2(1), n3(1),
                    n1(2), -n2(2), n3(2);
            Eigen::Vector3f b, alphas;
            b << delta(0), delta(1), delta(2);
            alphas = A.inverse() * b;

            closest_points.dist = abs(alphas(2));
            closest_points.closest_point1 = line1.start_point + n1 * alphas(0);
            closest_points.closest_point2 = line2.start_point + n2 * alphas(1);
        }
        return closest_points;
    }

    static ClosestPoints closestPointsBetweenLineSegments(const Line& line1,
                                                          const Line& line2) {
        ClosestPoints closest_points;
        if (line1.start_point.distance(line1.end_point) < SP_EPSILON_FLOAT) {
            closest_points = closestPointsBetweenPointAndLineSegment(line1.start_point, line2);
        } else if (line2.start_point.distance(line2.end_point) < SP_EPSILON_FLOAT) {
            closest_points = closestPointsBetweenPointAndLineSegment(line2.start_point, line1);
            std::swap(closest_points.closest_point1, closest_points.closest_point2);
        } else {
            point3d v1, v2, n1, n2;
            double l1, l2;
            v1 = line1.end_point - line1.start_point;
            v2 = line2.end_point - line2.start_point;
            l1 = v1.norm();
            l2 = v2.norm();
            n1 = v1 * (1 / l1);
            n2 = v2 * (1 / l2);

            if ((n1.cross(n2)).norm() < SP_EPSILON_FLOAT) { // If two line segments are parallel
                double bound_min = (line2.start_point - line1.start_point).dot(n1);
                double bound_max = (line2.end_point - line1.start_point).dot(n1);
                point3d p2_min = line2.start_point;
                point3d p2_max = line2.end_point;
                if(bound_max < bound_min){
                    std::swap(bound_min, bound_max);
                    std::swap(p2_min, p2_max);
                }

                point3d delta = line2.start_point - line1.start_point;
                delta = delta - n1 * (delta.dot(n1));
                if(l1 < bound_min){
                    closest_points.closest_point1 = line1.end_point;
                    closest_points.closest_point2 = p2_min;
                } else if(bound_max < 0) {
                    closest_points.closest_point1 = line1.start_point;
                    closest_points.closest_point2 = p2_max;
                } else if(bound_min < 0) {
                    closest_points.closest_point1 = line1.start_point;
                    closest_points.closest_point2 = line1.start_point + delta;
                } else {
                    closest_points.closest_point1 = p2_min - delta;
                    closest_points.closest_point2 = p2_min;
                }

                closest_points.dist = closest_points.closest_point1.distance(closest_points.closest_point2);
            } else {
                closest_points = closestPointsBetweenLines(line1, line2);

                //alpha check
                double alpha1, alpha2;
                alpha1 = (closest_points.closest_point1 - line1.start_point).dot(n1) / l1;
                alpha2 = (closest_points.closest_point2 - line2.start_point).dot(n2) / l2;

                if(alpha1 < 0) {
                    closest_points.closest_point1 = line1.start_point;
                } else if(alpha1 > 1) {
                    closest_points.closest_point1 = line1.end_point;
                }

                if(alpha2 < 0) {
                    closest_points.closest_point2 = line2.start_point;
                } else if(alpha2 > 1) {
                    closest_points.closest_point2 = line2.end_point;
                }

                if(alpha1 < 0 or alpha1 > 1){
                    double dot = n2.dot(closest_points.closest_point1 - line2.start_point);
                    if(dot < 0){
                        dot = 0;
                    } else if(dot > l2){
                        dot = l2;
                    }
                    closest_points.closest_point2 = line2.start_point + n2 * dot;
                }

                if(alpha2 < 0 or alpha2 > 1){
                    double dot = n1.dot(closest_points.closest_point2 - line1.start_point);
                    if(dot < 0){
                        dot = 0;
                    } else if(dot > l1){
                        dot = l1;
                    }
                    closest_points.closest_point1 = line1.start_point + n1 * dot;
                }

                closest_points.dist = closest_points.closest_point1.distance(closest_points.closest_point2);
            }
        }

        return closest_points;
    }

    static ClosestPoints closestPointsBetweenPointAndConvexHull(const point3d& point,
                                                                const points_t& convex_hull){
        /* Squared distance computed by openGJK.                                 */
        double dd;
        /* Structure of simplex used by openGJK.                                 */
        struct simplex  s;
        double v[3];
        /* Number of vertices defining body 1 and body 2, respectively.          */
        int nvrtx1, nvrtx2;
        /* Structures of body 1 and body 2, respectively.                        */
        struct bd bd1;
        struct bd bd2;

        bd1.numpoints = static_cast<int>(convex_hull.size());
        bd1.coord = point3DsToArray(convex_hull);

        points_t point_vector;
        point_vector.emplace_back(point);
        bd2.numpoints = 1;
        bd2.coord = point3DsToArray(point_vector);

        // Run GJK algorithm
        dd = gjk (bd1, bd2, &s, v);

        ClosestPoints closest_points;
        closest_points.closest_point1 = point;
        closest_points.closest_point2 = point + point3d(v[0], v[1], v[2]);
        closest_points.dist = dd;

        return closest_points;
    }

    static double computeCollisionTime(const Line &obs_path, const Line &agent_path,
                                       double collision_radius, double time_horizon) {
        // just solve 2nd poly?
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenLinePaths(obs_path, agent_path);

        double dist_in_sphere1, dist_in_sphere2, collision_time = SP_INFINITY;
        if (closest_points.dist <= collision_radius) {
            point3d a, b, delta;
            a = agent_path.start_point - obs_path.start_point;
            b = agent_path.end_point - obs_path.end_point;
            delta = closest_points.closest_point2 - closest_points.closest_point1;

            if (a.norm() <= collision_radius) {
                collision_time = 0;
            } else if (delta == b) {
                point3d n_line, c;
                double dist_to_b, dist_to_c;
                dist_to_b = b.norm();
                n_line = (b - a).normalized();
                c = a - n_line * a.dot(n_line);
                dist_to_c = c.norm();
                dist_in_sphere1 = sqrt(collision_radius * collision_radius - dist_to_c * dist_to_c);
                dist_in_sphere2 = sqrt(dist_to_b * dist_to_b - dist_to_c * dist_to_c);
                collision_time = (1 - (dist_in_sphere1 - dist_in_sphere2) / (b - a).norm()) * time_horizon;
            } else {
                double dist_to_b = b.norm();
                dist_in_sphere1 = sqrt(collision_radius * collision_radius - closest_points.dist * closest_points.dist);
                dist_in_sphere2 = sqrt(dist_to_b * dist_to_b - closest_points.dist * closest_points.dist);
                collision_time = (1 - (dist_in_sphere1 + dist_in_sphere2) / (b - a).norm()) * time_horizon;
            }
        }

        return collision_time;
    }
}