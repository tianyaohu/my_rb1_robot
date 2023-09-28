
#include "my_rb1_ros/Rotate.h"
#include "ros/spinner.h"
#include "ros/subscriber.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>

class RotateRb1Service {

protected:
  ros::NodeHandle nh_;
  ros::ServiceServer ss_;

  // init msg needed
  geometry_msgs::Twist msg_move_;
  nav_msgs::Odometry msg_odom_;

  // init publisher
  ros::Rate *rate_;
  ros::Publisher pub_move_;
  ros::Subscriber sub_odom_;

  // init other vars
  std::string service_name_;
  int rate_hz_;
  float angular_speed;
  std::string success;

  // orginal, rotation, and new quaternion
  double angle_cur;
  const double tolerance;

public:
  RotateRb1Service() : angular_speed(0.3), tolerance(0.15) {

    // init service, publisher, subsciber
    ss_ = nh_.advertiseService("/rotate_robot", &RotateRb1Service::serviceCB,
                               this);
    pub_move_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    sub_odom_ = nh_.subscribe("/odom", 1000, &RotateRb1Service::odomCB, this);

    rate_hz_ = 1;
    angular_speed = 0.3;

    // start rate, publisher, and subscriber
    rate_ = new ros::Rate(rate_hz_);
    ROS_INFO("Starting publisher for rotate");
  }

  // destructor
  ~RotateRb1Service(void) {}

  bool serviceCB(my_rb1_ros::Rotate::Request &req,
                 my_rb1_ros::Rotate::Response &res) {
    ROS_INFO("Requested degrees=%d", req.degrees);

    double angle_delta(degree2rad(req.degrees));
    ROS_INFO("Processed angle_delta in rad =%f", angle_delta);
    ROS_INFO("Processed current angle in rad is =%f", angle_cur);
    // update angular vel; shortest turn
    // angular_speed = (angle_delta > M_PI) ? angular_speed : -angular_speed;
    // Honest turn;
    angular_speed = (angle_delta > 0) ? -angular_speed : angular_speed;

    // update target
    double angle_target(normalize(angle_cur + angle_delta));
    ROS_INFO("Angle Target =%f", angle_target);

    this->rotate2target(angle_target);

    res.result =
        "Finished Rotating " + std::to_string(req.degrees) + " degree.";
    ROS_INFO("Finished service move_bb8_in_circle_custom");
    // reseting angular speed
    angular_speed = 0.3;
    return true;
  }

  double normalize(double rad) {
    rad = remainder(rad, (M_PI * 2));
    if (abs(rad) > M_PI) {
      rad = -(M_PI * 2 - rad);
    //   optional line: seeking the shortest turn 
    }
    return rad;
  }

  double degree2rad(int degrees) {
    return degrees / 180.0 * M_PI; // convert to rads
  }

  void rotate2target(double angle_target) {
    // set vel_msg
    msg_move_.linear.x = 0;
    msg_move_.angular.z = angular_speed;
    ROS_INFO("Begin to rotating robot...");
    while (nh_.ok()) {
      ROS_INFO("angle_target %f angle_cur) %f", angle_target, angle_cur);
      //   ROS_INFO("Updating current Yaw angle in degree %f", angle_cur);
      rate_->sleep();
      pub_move_.publish(msg_move_);
      ros::spinOnce();
      if (abs(angle_target - angle_cur) < tolerance) {
        ROS_INFO("angle_target %f angle_cur) %f", angle_target, angle_cur);
        break;
      }
    }
    // stop rotate
    this->stop();
  }

  void stop() {
    ROS_INFO("STOPING the robot.");
    msg_move_.linear.x = 0;
    msg_move_.angular.z = 0;
    pub_move_.publish(msg_move_);
  }

  void odomCB(const nav_msgs::OdometryConstPtr &msg) {
    angle_cur = tf::getYaw(msg->pose.pose.orientation);
    // ROS_INFO("Updating current Yaw angle in degree %f", angle_cur);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rb1_rotate_server");

  RotateRb1Service rotate_rb1_ss;

  ros::spin();

  return 0;
}