#ifndef DYNAMIC_PLANNER_TRAJECTORY_H
#define DYNAMIC_PLANNER_TRAJECTORY_H

#include <sp_const.hpp>
#include <polynomial.hpp>
#include <util.hpp>

namespace DynamicPlanning {
    template<typename T>
    class Segment {
    public:
        std::vector<T> control_points;
        double segment_time;

        // Find subsegment that t \in [t_normalized_0, t_normalized_f] * segment_time
        Segment<T> subSegment(double t_normalized_0, double t_normalized_f,
                              const Eigen::MatrixXd& B, const Eigen::MatrixXd& B_inv);

        [[nodiscard]] T startPoint() const;

        [[nodiscard]] T lastPoint() const;

        T operator[](int idx) const { return control_points[idx]; }

        T &operator[](int idx) { return control_points[idx]; }
    };

    // M-segment Bernstein polynomial trajectory
    template<typename T>
    class Trajectory {
    public:
        Trajectory();

        Trajectory(size_t M, size_t n, double dt);

        void planConstVelTraj(T current_state, T velocity);

        [[nodiscard]] int size() const;

        [[nodiscard]] T getPointAt(double time) const;

        [[nodiscard]] State getStateAt(double time) const;

        [[nodiscard]] T startPoint() const;

        [[nodiscard]] T lastPoint() const;

        [[nodiscard]] Trajectory<T> derivative() const;

        Trajectory<T> coordinateTransform(double downwash);

        [[nodiscard]] bool empty() const;

        void clear();

        [[nodiscard]] visualization_msgs::Marker toMarkerMsg(int agent_id,
                                               const std::string &frame_id,
                                               std_msgs::ColorRGBA color) const; // Only for point3d

        Segment<T> operator[](int idx) const;

        Segment<T> &operator[](int idx);

    private:
        size_t M; // The number of segments
        size_t n; // degree of the polynomial
        std::vector<Segment<T>> segments;
    };

    typedef Trajectory<point3d> traj_t;

    template class Segment<point3d>;
    template class Segment<double>;
    template class Trajectory<point3d>;
    template class Trajectory<double>;
}

#endif //DYNAMIC_PLANNER_TRAJECTORY_H
