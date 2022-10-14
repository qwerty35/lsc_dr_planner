#ifndef LSC_PLANNER_UTIL_HPP
#define LSC_PLANNER_UTIL_HPP

#include <sp_const.hpp>
#include <polynomial.hpp>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Vector3.h>

namespace DynamicPlanning {
    static geometry_msgs::Point defaultPoint(){
        geometry_msgs::Point msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        return msg;
    }

    static geometry_msgs::Vector3 defaultVector(){
        geometry_msgs::Vector3 msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        return msg;
    }


    static geometry_msgs::Quaternion defaultQuaternion(){
        geometry_msgs::Quaternion msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        msg.w = 1;
        return msg;
    }

    static point3d pointMsgToPoint3d(const geometry_msgs::Point& msg){
        return point3d(msg.x, msg.y, msg.z);
    }

    static point3d vector3MsgToPoint3d(const geometry_msgs::Vector3& msg){
        return point3d(msg.x, msg.y, msg.z);
    }

    static point3d vector3MsgToPoint3d(const tf::Vector3& vector){
        return point3d(vector.x(), vector.y(), vector.z());
    }

    static geometry_msgs::Point vector3MsgToPointMsg(const tf::Vector3& vector){
        geometry_msgs::Point point;
        point.x = vector.x();
        point.y = vector.y();
        point.z = vector.z();
        return point;
    }

    static geometry_msgs::Vector3 point3DToVector3Msg(const point3d& point){
        geometry_msgs::Vector3 vector;
        vector.x = point.x();
        vector.y = point.y();
        vector.z = point.z();
        return vector;
    }

    static geometry_msgs::Point point3DToPointMsg(const point3d& point){
        geometry_msgs::Point msg;
        msg.x = point.x();
        msg.y = point.y();
        msg.z = point.z();
        return msg;
    }

    static geometry_msgs::Twist point3DToTwistMsg(const point3d& point){
        geometry_msgs::Twist msg;
        msg.linear.x = point.x();
        msg.linear.y = point.y();
        msg.linear.z = point.z();
        return msg;
    }

    static geometry_msgs::Quaternion point3DToQuaternionMsg(const point3d& point){
        geometry_msgs::Quaternion msg;
        msg.x = point.x();
        msg.y = point.y();
        msg.z = point.z();
        msg.w = sqrt(1 - point.norm_sq());
        return msg;
    }

    static point3d quaternionMsgToPoint3D(const geometry_msgs::Quaternion& msg){
        return point3d(msg.x, msg.y, msg.z);
    }

    static geometry_msgs::Quaternion quaternionToQuaternionMsg(const Eigen::Quaterniond& q){
        geometry_msgs::Quaternion msg;
        msg.x = q.x();
        msg.y = q.y();
        msg.z = q.z();
        msg.w = q.w();
        return msg;
    }

    static std::vector<std::array<double, 3>> point3DsToArray(const points_t& points){
        /* Allocate memory. */
        std::vector<std::array<double, 3>> arrays;

        /* Read and store vertices' coordinates. */
        for (const auto& point : points)
        {
            std::array<double,3> array = {{point.x(), point.y(), point.z()}};
            arrays.emplace_back(array);
        }

        return arrays;
    }

    static point3d arrayToPoint3D(double* array){
        return point3d(array[0], array[1], array[2]);
    }

    static double LInfinityDistance(const point3d &p1, const point3d &p2){
        double dist = 0;
        for(int k = 0; k < 3; k++){
            double dist_cand = abs(p1(k) - p2(k));
            if(dist < dist_cand){
                dist = dist_cand;
            }
        }
        return dist;
    }

    template <typename T>
    bool hasElement(std::vector<T> vec, T element){
        return std::find(vec.begin(), vec.end(), element) != vec.end();
    }

    template <typename T>
    bool hasElement(std::set<T> set, T element){
        return set.find(element) != set.end();
    }

    template <typename T>
    int getIndex(std::vector<T> vec, T element){
        int index = -1;
        auto it = std::find(vec.begin(), vec.end(), element);
        if(it != vec.end()){
            index = it - vec.begin();
        }

        return index;
    }

    static double ellipsoidalDistance(const point3d& p1, const point3d& p2, double downwash){
        point3d delta = p1 - p2;
        delta.z() = delta.z() / downwash;
        return delta.norm();
    }

    static std::vector<std_msgs::ColorRGBA> getHSVColorMap(int size){
        std::vector<std_msgs::ColorRGBA> color;
        double h, f;
        int i;

        color.resize(size);
        for (int idx = 0; idx < size; idx++) {
            h = idx * 6 / (double) size;
            i = (int) h;
            f = h - i;

            switch (i) {
                case 0:
                    color[idx].r = 1;
                    color[idx].g = f;
                    color[idx].b = 0;
                    break;
                case 1:
                    color[idx].r = 1 - f;
                    color[idx].g = 1;
                    color[idx].b = 0;
                    break;
                case 2:
                    color[idx].r = 0;
                    color[idx].g = 1;
                    color[idx].b = f;
                    break;
                case 3:
                    color[idx].r = 0;
                    color[idx].g = 1 - f;
                    color[idx].b = 1;
                    break;
                case 4:
                    color[idx].r = f;
                    color[idx].g = 0;
                    color[idx].b = 1;
                    break;
                case 5:
                    color[idx].r = 1;
                    color[idx].g = 0;
                    color[idx].b = 1 - f;
                    break;
                default:
                    break;
            }
        }

        return color;
    }

    static visualization_msgs::MarkerArray msgDeleteAll(){
        visualization_msgs::MarkerArray msg_delete_all;
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        msg_delete_all.markers.emplace_back(marker);
        return msg_delete_all;
    }
}

#endif //LSC_PLANNER_UTIL_HPP
