#pragma once

#include <fstream>
#include <iostream>
#include <math.h>
#include <sp_const.hpp>
#include <mission.hpp>
#include <obstacle.hpp>
#include <random>


//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

namespace DynamicPlanning {
    class ObstacleGenerator {
    public:
        ObstacleGenerator(const ros::NodeHandle &_nh, const Mission& _mission)
            : nh(_nh), mission(_mission) {
            pub_obstacle_collision_model = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_collision_model",
                                                                                         1);
            start_time = ros::Time::now();
            obstacles.resize(mission.on);
        }

        void update(double t, double observer_stddev){
            std::vector<point3d> empty_vector;
            empty_vector.resize(mission.on);
            updateObstacles(t, observer_stddev, empty_vector);

            //update obstacle msg and add measurement error
            updateObstacles(t, observer_stddev);
        }

        void update(double t, double observer_stddev, const std::vector<point3d>& chasing_goal_points){
            updateObstacles(t, observer_stddev, chasing_goal_points);

            //update obstacle msg and add measurement error
            updateObstacles(t, observer_stddev);
        }

        void update(double t, double observer_stddev, const std::vector<Obstacle>& _obstacles){
            obstacles = _obstacles;

            //update obstacle msg and add measurement error
            updateObstacles(t, observer_stddev);
        }

        void publish(const std::string& frame_id) {
            publishObstacles(frame_id);
        }

        [[nodiscard]] int getNumObs() const {
            return mission.on;
        }

        // get obstacle states which contain measurement error
        Obstacle getObstacle(int oi){
            return obstacles[oi];
        }

        void resetStartTime(ros::Time _start_time){
            start_time = _start_time;
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_obstacle_collision_model;

        Mission mission;
        ros::Time start_time;
        std::vector<Obstacle> obstacles;

        void updateObstacles(double t, double observer_stddev, const std::vector<point3d>& chasing_points){
            // if obstacle is chasing then update target and other obstacles information
            for (size_t oi = 0; oi < mission.on; oi++) {
                if(mission.obstacles[oi]->getType() == "chasing"){
                    std::shared_ptr<ChasingObstacle> chasing_obstacle_ptr = std::static_pointer_cast<ChasingObstacle>(mission.obstacles[oi]);
                    chasing_obstacle_ptr->setGoalPoint(chasing_points[oi]);
                    std::vector<Obstacle> obstacles_prev_step = obstacles;
                    obstacles_prev_step.erase(obstacles_prev_step.begin() + oi);
                    chasing_obstacle_ptr->setObstacles(obstacles_prev_step);
                }
            }

            //update obstacles
            for(size_t oi = 0; oi < mission.on; oi++){
                obstacles[oi] = mission.obstacles[oi]->getObstacle(t);
                obstacles[oi].id = oi;
            }
        }

        void updateObstacles(double t, double observer_stddev){
            obstacles.resize(mission.on);

            std::random_device rd;
            std::mt19937 generator(rd());
            std::normal_distribution<double> distribution(0, observer_stddev);
            for (size_t oi = 0; oi < mission.on; oi++) {
                obstacles[oi].start_time = start_time;
                obstacles[oi] = obstacles[oi];
                obstacles[oi].observed_position.x() += distribution(generator);
                obstacles[oi].observed_position.y() += distribution(generator);
                obstacles[oi].observed_position.z() += distribution(generator);
            }
        }

        void publishObstacles(const std::string& frame_id) {
            visualization_msgs::MarkerArray msg_obstacle_collision_model;
            msg_obstacle_collision_model.markers.clear();

            for (size_t oi = 0; oi < mission.on; oi++) {
                if(mission.obstacles[oi]->getType() == "real"){
                    continue;
                }

                // obstacle collision model
                visualization_msgs::Marker mk_obs_true_position;
                mk_obs_true_position.header.frame_id = frame_id;
                mk_obs_true_position.ns = "true";
                mk_obs_true_position.id = oi;

                mk_obs_true_position.action = visualization_msgs::Marker::ADD;
//                mk_obs_true_position.lifetime = ros::Duration(1.0);
                mk_obs_true_position.pose.orientation.x = 0;
                mk_obs_true_position.pose.orientation.y = 0;
                mk_obs_true_position.pose.orientation.z = 0;
                mk_obs_true_position.pose.orientation.w = 1.0;
                mk_obs_true_position.pose.position = point3DToPointMsg(obstacles[oi].position);

                mk_obs_true_position.type = visualization_msgs::Marker::SPHERE;
                mk_obs_true_position.scale.x = 2 * obstacles[oi].radius;
                mk_obs_true_position.scale.y = 2 * obstacles[oi].radius;
                mk_obs_true_position.scale.z = 2 * obstacles[oi].radius * obstacles[oi].downwash;

                mk_obs_true_position.color.a = 0.5;
                mk_obs_true_position.color.r = 0;
                mk_obs_true_position.color.g = 0;
                mk_obs_true_position.color.b = 0;
                msg_obstacle_collision_model.markers.emplace_back(mk_obs_true_position);

                visualization_msgs::Marker mk_obs_observed_position = mk_obs_true_position;
                mk_obs_observed_position.ns = "observed";
                mk_obs_observed_position.id = oi;
                mk_obs_observed_position.pose.position = point3DToPointMsg(obstacles[oi].position);
                msg_obstacle_collision_model.markers.emplace_back(mk_obs_observed_position);
            }

            pub_obstacle_collision_model.publish(msg_obstacle_collision_model);
        }
    };
}