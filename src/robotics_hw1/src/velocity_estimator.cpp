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
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed>
    ExactTimePolicy;

typedef message_filters::Synchronizer<ExactTimePolicy> ExactTimeSynchronizer;

// let's take the double value of pi from boost library
const double pi = boost::math::constants::pi<double>();

class VelocityEstimator {
private:
  const ros::NodeHandle node_handle;

  ros::Publisher publisher;

  boost::shared_ptr<ExactTimeSynchronizer> time_syncronizer_ptr;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_right;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_rigt;

  const double initial_pose_x;
  const double initial_pose_y;
  const double initial_pose_theta;

  const double real_baseline;
  const double apparent_baseline;
  const double wheel_radius;
  const double gear_ratio;

public:
  VelocityEstimator(ros::NodeHandle &node_handle, const double &initial_pose_x,
                    const double &initial_pose_y,
                    const double &initial_pose_theta,
                    const double &wheel_radius, const double &real_baseline,
                    const double &gear_ratio, const double &apparent_baseline);

  void sync_and_publish_velocity_message(
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right);
};

void VelocityEstimator::sync_and_publish_velocity_message(
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right) {

  ROS_INFO("Seq %d, rpm are %f", motor_speed_front_left->header.seq,
           motor_speed_front_left->rpm);

  if (motor_speed_front_left->rpm != motor_speed_rear_left->rpm) {
    ROS_INFO("Front left rpm value %f is different from rear left rpm value %f",
             motor_speed_front_left->rpm, motor_speed_rear_left->rpm);
  }

  if (motor_speed_front_right->rpm != motor_speed_rear_right->rpm) {
    ROS_INFO(
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

  // Since often on same timestamps the rpm of the front and back wheels on the
  // same side differ, maybe because of measurement errors or due the fact that
  // it is not an ideal robot, we take the mean of the two measurements

  // the left rotation is registered as backwards, so we invert it
  double rpm_left =
      -(motor_speed_front_left->rpm + motor_speed_rear_left->rpm) / 2;
  double rpm_right =
      (motor_speed_front_right->rpm + motor_speed_rear_right->rpm) / 2;

  double omega_l = (2 * pi) * ((rpm_left * gear_ratio) / 60);
  double omega_r = (2 * pi) * ((rpm_right * gear_ratio) / 60);

  // from the rotational speeds and the wheel radius we can easily estimate the
  // linear speed
  velocity_message.twist.linear.x =
      ((omega_l * wheel_radius) + (omega_r * wheel_radius)) / 2;

  // we use the formula with the apparent baseline to get omega_z, i.e. the
  // angular speed of the robot when it turns
  velocity_message.twist.angular.z =
      (-(omega_l * wheel_radius) + (omega_r * wheel_radius)) /
      apparent_baseline;

  publisher.publish(velocity_message);
}

VelocityEstimator::VelocityEstimator(
    ros::NodeHandle &node_handle, const double &initial_pose_x,
    const double &initial_pose_y, const double &initial_pose_theta,
    const double &wheel_radius, const double &real_baseline,
    const double &gear_ratio, const double &apparent_baseline)
    : node_handle(node_handle), initial_pose_x(initial_pose_x),
      initial_pose_y(initial_pose_y), initial_pose_theta(initial_pose_theta),
      wheel_radius(wheel_radius), real_baseline(real_baseline),
      gear_ratio(gear_ratio), apparent_baseline(apparent_baseline) {

  // subscribe to the motor speed topics TODO remove hard coding and put
  // parameters/argvs with topics names

  subscriber_front_right.subscribe(node_handle, "/motor_speed_fr", 100);
  subscriber_front_left.subscribe(node_handle, "/motor_speed_fl", 100);
  subscriber_rear_rigt.subscribe(node_handle, "/motor_speed_rr", 100);
  subscriber_rear_left.subscribe(node_handle, "/motor_speed_rl", 100);

  // publish the topic
  publisher = node_handle.advertise<geometry_msgs::TwistStamped>(
      "/robot_twisted_stamped", 10);

  // initialize the time syncronizer
  time_syncronizer_ptr.reset(new ExactTimeSynchronizer(
      ExactTimePolicy(100), subscriber_front_left, subscriber_front_right,
      subscriber_rear_left, subscriber_rear_rigt));

  // register the callback. One placeholder for each function argument
  time_syncronizer_ptr->registerCallback(
      boost::bind(&VelocityEstimator::sync_and_publish_velocity_message, this,
                  _1, _2, _3, _4));
}

double get_double_parameter(const std::string &parameter_key,
                            const ros::NodeHandle &node_handle) {
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

int main(int argc, char **argv) {

  ros::init(argc, argv, "velocity_estimator");

  // we need our args
  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  // we should receive:
  // program name
  // a string which is either scout_odom or gt_pose, to decide which estimated baseline and gear ratio use
  if (args.size() != 2) {
    ROS_ERROR("tf_publisher: Usage: %s pose_or_odom", argv[0]);
    ROS_ERROR("tf_publisher: Instead you provided %lu argument(s): ",
              args.size());
    for (std::string arg : args)
      ROS_ERROR("%s", arg.c_str());
    return 1;
  }

  std::string pose_or_odom = args[1];

  ros::NodeHandle node_handle;
  VelocityEstimator odometry_calculator(
      node_handle, get_double_parameter("initial_pose_x", node_handle),
      get_double_parameter("initial_pose_y", node_handle),
      get_double_parameter("initial_pose_theta", node_handle),
      get_double_parameter("wheel_radius", node_handle),
      get_double_parameter("real_baseline", node_handle),
      get_double_parameter("gear_ratio_from_" + pose_or_odom, node_handle),
      get_double_parameter("apparent_baseline_from_" + pose_or_odom, node_handle));

  ros::spin();

  return 0;
}
