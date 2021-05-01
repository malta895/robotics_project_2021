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

#include "robotics_hw1/CalibratedParamsGtPose.h"
#include "robotics_hw1/CalibratedParamsOdom.h"
#include "robotics_hw1/MotorSpeed.h"

#include "geometry_msgs/TwistStamped.h"
#include <iomanip>
#include <sstream>
#include <string>

typedef message_filters::sync_policies::ExactTime<
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry>
    ExactTimeOdometryPolicy;

typedef message_filters::Synchronizer<ExactTimeOdometryPolicy>
    ExactTimeOdometrySynchronizer;

typedef message_filters::sync_policies::ExactTime<
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, geometry_msgs::Pose>
    ExactTimeGtPosePolicy;

typedef message_filters::Synchronizer<ExactTimeGtPosePolicy>
    ExactTimeGtPoseSynchronizer;

const double pi = boost::math::constants::pi<double>();

class ParameterCalibrator {
private:
  ros::NodeHandle node_handle;

  ros::Publisher calibrated_from_odom_publisher;
  ros::Publisher calibrated_from_gt_pose_publisher;

  boost::shared_ptr<ExactTimeOdometrySynchronizer> time_syncronizer_ptr;

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

  // these variables are needed to calculate the mean of the gear ratio and the
  // apparent base line
  long int count_apparent_baseline = 0;
  long int count_gear_ratio = 0;
  double gear_ratio = 0;
  double apparent_baseline = 0;

  double get_double_parameter(const std::string &parameter_key);

public:
  ParameterCalibrator();

  void calculate_mean_callback(
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
      const nav_msgs::OdometryConstPtr &scout_odom);
};

void ParameterCalibrator::calculate_mean_callback(
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
    const nav_msgs::OdometryConstPtr &scout_odom) {

  double omega_l = (2 * pi) * (-motor_speed_front_left->rpm / 60);
  double omega_r = (2 * pi) * (motor_speed_front_right->rpm / 60);
  double odom_omega_z = scout_odom->twist.twist.angular.z;
  double v_x = scout_odom->twist.twist.linear.x;

  double current_gear_ratio =
      std::abs((2 * v_x) / ((omega_l + omega_r) * wheel_radius));

  // I know from the specifications that the gear ratio is within this range
  if (current_gear_ratio > .025000l && current_gear_ratio < .028571l) {
    gear_ratio = ((gear_ratio * count_gear_ratio) + current_gear_ratio) /
                 ++count_gear_ratio;
  }

  double current_apparent_baseline = std::abs(
      ((-omega_l + omega_r) * 2 * v_x) / (odom_omega_z * (omega_l + omega_r)));

  // I can guess that values outside this range are a result of too "dirty"
  // measuremnents
  if (current_apparent_baseline > 0.800000 &&
      current_apparent_baseline < 1.200000) {
    apparent_baseline = ((apparent_baseline * count_apparent_baseline) +
                         current_apparent_baseline) /
                        ++count_apparent_baseline;
  }

  //publish the calibrated values as a message

  robotics_hw1::CalibratedParamsOdom params_message;

  params_message.header = scout_odom->header;
  params_message.apparent_baseline = apparent_baseline;
  params_message.gear_ratio = gear_ratio;

  calibrated_from_odom_publisher.publish(params_message);
}

double
ParameterCalibrator::get_double_parameter(const std::string &parameter_key) {

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

ParameterCalibrator::ParameterCalibrator()
    : initial_pose_x(get_double_parameter("initial_pose_x")),
      initial_pose_y(get_double_parameter("initial_pose_y")),
      initial_pose_theta(get_double_parameter("initial_pose_theta")),
      wheel_radius(get_double_parameter("wheel_radius")),
      real_baseline(get_double_parameter("real_baseline")) {

  // subscribe to the motor speed topics TODO remove hard coding and put
  // parameters/argvs with topics names

  subscriber_front_right.subscribe(node_handle, "/motor_speed_fr", 100);
  subscriber_front_left.subscribe(node_handle, "/motor_speed_fl", 100);
  subscriber_rear_rigt.subscribe(node_handle, "/motor_speed_rr", 100);
  subscriber_rear_left.subscribe(node_handle, "/motor_speed_rl", 100);
  subscriber_scout_odom.subscribe(node_handle, "/scout_odom", 100);

  // publish the topics
  calibrated_from_odom_publisher =
      node_handle.advertise<robotics_hw1::CalibratedParamsOdom>(
          "/apparent_baseline_from_odom", 10);
  calibrated_from_gt_pose_publisher =
      node_handle.advertise<robotics_hw1::CalibratedParamsGtPose>(
          "/apparent_baseline_from_gt_pose", 10);

  // initialize the time syncronizer
  time_syncronizer_ptr.reset(new ExactTimeOdometrySynchronizer(
      ExactTimeOdometryPolicy(100), subscriber_front_left,
      subscriber_front_right, subscriber_rear_left, subscriber_rear_rigt,
      subscriber_scout_odom));

  // register the callback. One placeholder for each function argument
  time_syncronizer_ptr->registerCallback(boost::bind(
      &ParameterCalibrator::calculate_mean_callback, this, _1, _2, _3, _4, _5));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "parameters_calibrator");

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

  ParameterCalibrator parameter_calibrator;

  ros::spin();

  return 0;
}
