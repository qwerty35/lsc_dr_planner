//
// Created by jungwon on 22. 1. 5..
//

#include "trajectory.hpp"

namespace DynamicPlanning {
    template<typename T>
    Segment<T> Segment<T>::subSegment(double t_normalized_0, double t_normalized_f, const Eigen::MatrixXd& B,
                                      const Eigen::MatrixXd& B_inv){
        ROS_ERROR("Wrong usage");
    }

    template<>
    Segment<point3d> Segment<point3d>::subSegment(double t_normalized_0, double t_normalized_f,
                                                  const Eigen::MatrixXd& B,
                                                  const Eigen::MatrixXd& B_inv){
        // t -> at + b
        double a, b;
        b = t_normalized_0;
        a = t_normalized_f - t_normalized_0;

        int n = (int)control_points.size() - 1;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n + 1, n + 1);
        for(int i = 0; i < n + 1; i++){
            for(int j = 0; j < i + 1; j++){
                A(i, j) = nChoosek(i, j) * pow(a, j) * pow(b, i - j);
            }
        }

        Eigen::MatrixXd c_tr = Eigen::MatrixXd::Zero(3, n + 1);
        for(int k = 0; k < 3; k++){
            for(int i = 0; i < n + 1; i++){
                c_tr(k, i) = control_points[i](k);
            }
        }
        c_tr = c_tr * B * A * B_inv;

        Segment<point3d> sub_segment;
        sub_segment.segment_time = segment_time * a;
        sub_segment.control_points.resize(n + 1);
        for(int i = 0; i < n + 1; i++){
            sub_segment.control_points[i] = point3d(c_tr(0, i),
                                                    c_tr(1, i),
                                                    c_tr(2, i));
        }

        return sub_segment;
    }

    template<typename T>
    T Segment<T>::startPoint() const {
        return control_points[0];
    }

    template<typename T>
    T Segment<T>::lastPoint() const {
        return control_points.back();
    }

    template<typename T>
    Trajectory<T>::Trajectory() {
        M = 0;
        n = 0;
    }

    template<typename T>
    Trajectory<T>::Trajectory(size_t _M, size_t _n, double dt) {
        M = _M;
        n = _n;
        segments.resize(M);
        for (int m = 0; m < M; m++) {
            segments[m].control_points.resize(n + 1);
            segments[m].segment_time = dt;
        }
    }

    template<typename T>
    void Trajectory<T>::planConstVelTraj(T current_state, T velocity){
        if(n == 0 or segments.empty()){
            throw std::invalid_argument("[Trajectory] n = 0");
        }

        double time = 0;
        for (int m = 0; m < M; m++) {
            for (int i = 0; i < n + 1; i++) {
                segments[m][i] = current_state + velocity * time;
                time +=  segments[m].segment_time / n;
            }
        }
    }

    template<typename T>
    bool Trajectory<T>::empty() const {
        return segments.empty();
    }

    template<typename T>
    void Trajectory<T>::clear() {
        M = 0;
        n = 0;
        segments.clear();
    }

    template<typename T>
    int Trajectory<T>::size() const {
        return segments.size();
    }

    template<typename T>
    T Trajectory<T>::getPointAt(double time) const {
        T point;
        if (time < 0) {
            ROS_ERROR("[Trajectory] trajectory getPoint time < 0");
            return point;
//            throw std::invalid_argument("[Trajectory] trajectory getPoint time < 0");
        }

        int m = -1;
        double t_normalized, segment_end_time = 0;
        for (int idx = 0; idx < M; idx++) {
            segment_end_time += segments[idx].segment_time;
            if (time < segment_end_time) {
                m = idx;
                t_normalized = 1 - (segment_end_time - time) / segments[idx].segment_time;
                break;
            }
        }

        if (m == -1) {
            if(time < segment_end_time + SP_EPSILON_FLOAT){
                m = M - 1;
                t_normalized = 1.0;
            }
            else{
                ROS_ERROR("[Trajectory] trajectory getPoint time is out of bound");
                return point;
//                throw std::invalid_argument("[Trajectory] trajectory getPoint time is out of bound");
            }
        }

        for (int i = 0; i < n + 1; i++) {
            double b_i_n = getBernsteinBasis(n, i, t_normalized);
            point = point + segments[m].control_points[i] * b_i_n;
        }

        return point;
    }

    template<typename T>
    State Trajectory<T>::getStateAt(double time) const {
        ROS_ERROR("Wrong usage");
    }

    template<>
    State Trajectory<point3d>::getStateAt(double time) const {
        State state;
        state.position = getPointAt(time);

        traj_t dtraj = derivative();
        state.velocity = dtraj.getPointAt(time);

        traj_t ddtraj = dtraj.derivative();
        state.acceleration = ddtraj.getPointAt(time);

        traj_t dddtraj = ddtraj.derivative();
        point3d jerk = dddtraj.getPointAt(time);

        return state;
    }

    template<typename T>
    T Trajectory<T>::startPoint() const {
        return segments[0][0];
    }

    template<typename T>
    T Trajectory<T>::lastPoint() const {
        return segments[M - 1][n];
    }

    template<typename T>
    Trajectory<T> Trajectory<T>::derivative() const {
        Trajectory<T> dtraj;
        dtraj.M = M;
        dtraj.n = n - 1;
        dtraj.segments.resize(M);
        for (int m = 0; m < M; m++) {
            dtraj.segments[m].segment_time = segments[m].segment_time;
            dtraj.segments[m].control_points.resize(n + 1);
            for (int i = 0; i < n; i++) {
                dtraj.segments[m].control_points[i] =
                        (segments[m].control_points[i + 1] - segments[m].control_points[i]) *
                        (n / segments[m].segment_time);
            }
        }

        return dtraj;
    }

    template<typename T>
    Trajectory<T> Trajectory<T>::coordinateTransform(double downwash) {
        ROS_ERROR("Wrong usage");
    }

    template<>
    Trajectory<point3d> Trajectory<point3d>::coordinateTransform(double downwash) {
        traj_t traj_trans;
        traj_trans.M = M;
        traj_trans.n = n;
        traj_trans.segments = segments;
        for(int m = 0; m < M; m++){
            for(int i = 0; i < n + 1; i++){
                traj_trans.segments[m].control_points[i].z() /= (float)downwash;
            }
        }

        return traj_trans;
    }

    template<typename T>
    visualization_msgs::Marker Trajectory<T>::toMarkerMsg(int agent_id,
                                                          const std::string &frame_id,
                                                          std_msgs::ColorRGBA color) const {
        ROS_ERROR("Wrong usage");
        visualization_msgs::Marker marker;
        return marker;
    }

    template<>
    visualization_msgs::Marker Trajectory<point3d>::toMarkerMsg(int agent_id,
                                                                const std::string &frame_id,
                                                                std_msgs::ColorRGBA color) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = std::to_string(agent_id);
        marker.id = agent_id;
        marker.scale.x = 0.05;
        marker.color = color;
        marker.pose.orientation = defaultQuaternion();

        double horizon = 0, dt = 0.05, time = 0;
        for(const auto& segment : segments){
            horizon += segment.segment_time;
        }
        while(time < horizon){
            point3d point = getPointAt(time);
            marker.points.emplace_back(point3DToPointMsg(point));
            time += dt;
        }
        point3d point = getPointAt(horizon);
        marker.points.emplace_back(point3DToPointMsg(point));

        return marker;
    }

    template<typename T>
    Segment<T> Trajectory<T>::operator[] (int idx) const {
        return segments[idx];
    }

    template<typename T>
    Segment<T>& Trajectory<T>::operator[] (int idx){
        return segments[idx];
    }

//    template class Segment<point3d>;
//    template class Segment<double>;
//    template class Trajectory<point3d>;
//    template class Trajectory<double>;
}