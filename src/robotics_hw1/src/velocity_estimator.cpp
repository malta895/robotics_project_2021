#include "boost/bind/bind.hpp"
#include "boost/math/constants/constants.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/time_synchronizer.h"

#include "robotics_hw1/MotorSpeed.h"

#include "geometry_msgs/TwistStamped.h"
#include <iomanip>
#include <sstream>
#include <string>

// I verified that in the bags the four messages have exactly synchronized
// timestamps, thus I can use the ExactTime policy
typedef message_filters::sync_policies::ExactTime<
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry>
    ExactTimePolicy;

typedef message_filters::Synchronizer<ExactTimePolicy> ExactTimeSynchronizer;

const double pi = boost::math::constants::pi<double>();

class VelocityEstimator {
private:
  ros::NodeHandle node_handle;

  ros::Publisher publisher;

  boost::shared_ptr<ExactTimeSynchronizer> time_syncronizer_ptr;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_right;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_rigt;
  message_filters::Subscriber<nav_msgs::Odometry> subscriber_scout_odom;

  const double initial_pose_x;
  const double initial_pose_y;
  const double initial_pose_theta;

  const double real_baseline;
  const double wheel_radius;
  const double gear_ratio;//TODO calibrate and use this instead of hardcoded 40

  double get_double_parameter(const std::string &parameter_key);

public:
  VelocityEstimator();

  void sync_and_publish_velocity_message(
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
      const nav_msgs::OdometryConstPtr &scout_odom);
};

void VelocityEstimator::sync_and_publish_velocity_message(
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
    const nav_msgs::OdometryConstPtr &scout_odom) {

  ROS_INFO("Seq %d, rpm are %f", motor_speed_front_left->header.seq,
           motor_speed_front_left->rpm);

  if (motor_speed_front_left->rpm != motor_speed_rear_left->rpm) {
    ROS_WARN("Front left rpm value %f is different from rear left rpm value %f",
             motor_speed_front_left->rpm, motor_speed_rear_left->rpm);
  }

  if (motor_speed_front_right->rpm != motor_speed_rear_right->rpm) {
    ROS_WARN(
        "Front right rpm value %f is different from rear right rpm value %f",
        motor_speed_front_right->rpm, motor_speed_rear_right->rpm);
  }

  geometry_msgs::TwistStamped velocity_message;

  // take the Header from one of the messages, no matter which one, they are all
  // the same because of ExactTimePolicy
  velocity_message.header = motor_speed_front_left->header;

  // Speed message structure:
  // Header header
  // float64 rpm
  //
  // formula for apparent baseline:
  // \chi = frac{y_l - y_r}{B}

  // \omega = 2\pi\math{rpm}

  // TODO calculate the speeds below
  //\omega = 2pi * rpm/60

  double omega_l = (2 * pi) * (-motor_speed_front_left->rpm / 60);
  omega_l /= 40;// TODO calibrator (package?) that from odometry calculates the ratio and the baseline.
  double omega_r = (2 * pi) * (motor_speed_front_right->rpm / 60);
  omega_r /= 40;
  double odom_omega_z = scout_odom->twist.twist.angular.z;
  double apparent_baseline =
      (-(omega_l * wheel_radius) + (omega_r * wheel_radius)) / odom_omega_z;

  velocity_message.twist.linear.x =
      ((omega_l * wheel_radius) + (omega_r * wheel_radius)) / 2;
  velocity_message.twist.linear.y =
      apparent_baseline; // FIXME just for debugging, remove this
  velocity_message.twist.linear.z =
      apparent_baseline /
      real_baseline; //\chi FIXME just for debugging, remove this

  velocity_message.twist.angular.x = omega_l;
  velocity_message.twist.angular.y = omega_r;
  // velocity_message.twist.angular.z = 0;

  publisher.publish(velocity_message);
}

double
VelocityEstimator::get_double_parameter(const std::string &parameter_key) {
  // this is just an helper function used to retrieve the double params from ROS
  // environment. I need this because in C++ I can initialize const in classes
  // only by calling a function in the member initializer list of the
  // constructor. I am using such const members for consinstency on params that
  // are not supposed to be modified

  // we take the parameter from the node_handle
  double parameter_value;

  if (!node_handle.getParam(parameter_key, parameter_value)) {
    // NOTE maybe I should throw an exception and exit with an error instead
    ROS_ERROR("Parameter %s not provided! Defaulting to 0",
              parameter_key.c_str());
    return 0;
  }
  return parameter_value;
}

// the constructor take the parameters and set them to the const
// variables in the initializer list by calling the helper function defined
// above
VelocityEstimator::VelocityEstimator()
    : initial_pose_x(get_double_parameter("initial_pose_x")),
      initial_pose_y(get_double_parameter("initial_pose_y")),
      initial_pose_theta(get_double_parameter("initial_pose_theta")),
      wheel_radius(get_double_parameter("wheel_radius")),
      real_baseline(get_double_parameter("initial_pose_theta"))

{

  // subscribe to the motor speed topics TODO remove hard coding and put
  // parameters/argvs with topics names

  subscriber_front_right.subscribe(node_handle, "/motor_speed_fr", 100);
  subscriber_front_left.subscribe(node_handle, "/motor_speed_fl", 100);
  subscriber_rear_rigt.subscribe(node_handle, "/motor_speed_rr", 100);
  subscriber_rear_left.subscribe(node_handle, "/motor_speed_rl", 100);
  subscriber_scout_odom.subscribe(node_handle, "/scout_odom", 100);

  // publish the topic
  publisher = node_handle.advertise<geometry_msgs::TwistStamped>(
      "/robot_twisted_stamped", 10);

  // initialize the time syncronizer
  time_syncronizer_ptr.reset(new ExactTimeSynchronizer(
      ExactTimePolicy(100), subscriber_front_left, subscriber_front_right,
      subscriber_rear_left, subscriber_rear_rigt, subscriber_scout_odom));

  // register the callback. One placeholder for each function argument
  time_syncronizer_ptr->registerCallback(
      boost::bind(&VelocityEstimator::sync_and_publish_velocity_message, this,
                  _1, _2, _3, _4, _5));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "velocity_estimator");

  // we need our args
  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  // TODO see if we need to receive some args. Otherwise, remove this code
  // we should receive:
  // program name
  // object id (e.g. front or obs)
  // if (args.size() != 2) {
  //   ROS_ERROR("tf_publisher: Usage: %s object_id", argv[0]);
  //   ROS_ERROR("tf_publisher: Instead you provided %lu argument(s): ",
  //             args.size());
  //   for (std::string arg : args)
  //     ROS_ERROR("%s", arg.c_str());
  //   return 1;
  // }

  VelocityEstimator odometry_calculator;

  ros::spin();

  return 0;
}
