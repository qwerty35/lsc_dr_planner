#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char* argv[]){
    ROS_INFO("Multi Sync Simulator");
    ros::init (argc, argv, "simple_publisher_node");
    ros::NodeHandle nh( "~" );

    ros::Publisher pub_collision_model = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_collision_model", 1);
    tf::TransformListener listener;

    ros::Rate rate(100.0);
    while (ros::ok()){
        double radius = 0.35;

        visualization_msgs::MarkerArray msg_obs_collision_model;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        for (int oi = 0; oi < 2; oi++) {
            tf::StampedTransform transform;
            try{
                listener.lookupTransform("/world", "/obs" + std::to_string(oi), ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.5).sleep();
            }

            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.color.a = 0.5;

            marker.scale.x = 2 * radius;
            marker.scale.y = 2 * radius;
            marker.scale.z = 2 * radius * 4;

            marker.id = oi;
            marker.pose.position.x = transform.getOrigin().x();
            marker.pose.position.y = transform.getOrigin().y();
            marker.pose.position.z = transform.getOrigin().z();

            msg_obs_collision_model.markers.emplace_back(marker);
        }

        pub_collision_model.publish(msg_obs_collision_model);
        rate.sleep();
    }
    return 0;
}