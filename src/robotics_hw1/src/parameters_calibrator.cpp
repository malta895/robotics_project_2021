#include "boost/bind/bind.hpp"
#include "boost/math/constants/constants.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"

#include "message_filters/sync_policies/approximate_time.h"

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
    OdometryTimePolicy;

typedef message_filters::Synchronizer<OdometryTimePolicy>
    ExactTimeOdometrySynchronizer;

typedef message_filters::sync_policies::ApproximateTime<
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
    geometry_msgs::PoseStamped>
    GtPoseTimePolicy;

typedef message_filters::Synchronizer<GtPoseTimePolicy> GtPoseSynchronizer;

const double pi = boost::math::constants::pi<double>();

class ParameterCalibrator {
private:
  ros::NodeHandle node_handle;

  ros::Publisher calibrated_from_odom_publisher;
  ros::Publisher calibrated_from_gt_pose_publisher;

  boost::shared_ptr<ExactTimeOdometrySynchronizer> odom_time_syncronizer_ptr;
  boost::shared_ptr<GtPoseSynchronizer> gt_pose_time_syncronizer_ptr;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_right;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_rigt;
  message_filters::Subscriber<nav_msgs::Odometry> subscriber_scout_odom;
  message_filters::Subscriber<geometry_msgs::PoseStamped> subscriber_gt_pose;

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

  // this are needed for calculations on \gt_pose
  long int count_pose = 0;
  long int count_pose_gear_ratio = 0;
  long int count_pose_apparent_baseline = 0;
  geometry_msgs::PoseStamped prev_pose;
  double pose_gear_ratio = 0;
  double pose_apparent_baseline = 0;

  double get_double_parameter(const std::string &parameter_key);
  double calculate_apparent_baseline(const double &omega_l,
                                     const double omega_r, const double &v_x,
                                     const double &omega_z);

  double calculate_gear_ratio(const double &v_x, const double &omega_l,
                              const double &omega_r);

public:
  ParameterCalibrator();

  void calibrate_with_odom_callback(
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
      const nav_msgs::OdometryConstPtr &scout_odom);

  void calibrate_with_gt_pose_callback(
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
      const geometry_msgs::PoseStampedConstPtr &gt_pose);
};

double ParameterCalibrator::calculate_apparent_baseline(const double &omega_l,
                                                        const double omega_r,
                                                        const double &v_x,
                                                        const double &omega_z) {
  if (std::abs(v_x) > 0.1 && std::abs(omega_z) > 0.1)
    return std::abs(((-omega_l + omega_r) * 2 * v_x) /
                    (omega_z * (omega_l + omega_r)));
  else
    return 0;
}

double ParameterCalibrator::calculate_gear_ratio(const double &v_x,
                                                 const double &omega_l,
                                                 const double &omega_r) {
  if (std::abs(v_x) > 0.1)
    return std::abs((2 * v_x) / ((omega_l + omega_r) * wheel_radius));
  else
    return 0;
}

void ParameterCalibrator::calibrate_with_odom_callback(
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
    const nav_msgs::OdometryConstPtr &scout_odom) {

  double omega_l = (2 * pi) * (-motor_speed_front_left->rpm / 60);
  double omega_r = (2 * pi) * (motor_speed_front_right->rpm / 60);
  double odom_omega_z = scout_odom->twist.twist.angular.z;
  double v_x = scout_odom->twist.twist.linear.x;

  double current_gear_ratio = calculate_gear_ratio(v_x, omega_l, omega_r);

  // I know from the specifications that the gear ratio is within this range
  if (current_gear_ratio > .025000l && current_gear_ratio < .028571l) {
    gear_ratio = ((gear_ratio * count_gear_ratio) + current_gear_ratio);
    gear_ratio /= ++count_gear_ratio;
  }

  double current_apparent_baseline =
      calculate_apparent_baseline(omega_l, omega_r, v_x, odom_omega_z);

  // I can guess that values outside this range are a result of too "dirty"
  // measuremnents
  // if (current_apparent_baseline > 0.900000 &&
  //     current_apparent_baseline < 1.100000) {
  if (current_apparent_baseline > real_baseline) {
    apparent_baseline = ((apparent_baseline * count_apparent_baseline) +
                         current_apparent_baseline);
    apparent_baseline /= ++count_apparent_baseline;
  }

  // publish the calibrated values as a message

  robotics_hw1::CalibratedParamsOdom params_message;

  params_message.header = scout_odom->header;
  params_message.apparent_baseline = apparent_baseline;
  params_message.gear_ratio = gear_ratio;
  params_message.last_apparent_baseline = current_apparent_baseline;
  params_message.last_gear_ratio = current_gear_ratio;

  calibrated_from_odom_publisher.publish(params_message);
}

void ParameterCalibrator::calibrate_with_gt_pose_callback(
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
    const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right,
    const geometry_msgs::PoseStampedConstPtr &gt_pose) {
  // TODO we have to take at least 2 positions. we then calculate the speed only
  // when the position changes to avoid bad reads

  double v_x = 0;
  double omega_z = 0;

  if (count_pose == 0) {
    prev_pose = *gt_pose;
    ++count_pose;
  } else {
    double distance = std::sqrt(
        std::pow(gt_pose->pose.position.x - prev_pose.pose.position.x, 2) +
        std::pow(gt_pose->pose.position.y - prev_pose.pose.position.y, 2) +
        std::pow(gt_pose->pose.position.z - prev_pose.pose.position.z, 2));

    double delta_z = std::asin(gt_pose->pose.orientation.z) * 2 -
      std::asin(prev_pose.pose.orientation.z) * 2;


    ros::Duration delta_time = gt_pose->header.stamp - prev_pose.header.stamp;

    if (distance != 0) {
      omega_z = delta_z / delta_time.toSec();
      v_x = distance / delta_time.toSec();
    }

    ++count_pose;
    prev_pose = *gt_pose;

    double omega_l = (2 * pi) * (-motor_speed_front_left->rpm / 60);
    double omega_r = (2 * pi) * (motor_speed_front_right->rpm / 60);

    if (omega_r < 0)
      v_x = -v_x;

    double current_apparent_baseline =
        calculate_apparent_baseline(omega_l, omega_r, v_x, omega_z);
    double current_gear_ratio = calculate_gear_ratio(v_x, omega_l, omega_r);

    robotics_hw1::CalibratedParamsGtPose params_message;
    if (current_gear_ratio > .025000l && current_gear_ratio < .028571l) {
      pose_gear_ratio =
          ((pose_gear_ratio * count_pose_gear_ratio) + current_gear_ratio);
      pose_gear_ratio /= ++count_pose_gear_ratio;
    }

    // if (current_apparent_baseline > 0.900000l &&
    //     current_apparent_baseline < 1.1000 00l){
    if (current_apparent_baseline > 1.0 && //I know it is bigger than 1, lower values are probably "dirty"
        current_apparent_baseline < 1.034) {// as a upper bound I put the odometry value
      pose_apparent_baseline =
          ((pose_apparent_baseline * count_pose_apparent_baseline) +
           current_apparent_baseline);
      pose_apparent_baseline /= ++count_pose_apparent_baseline;
    }

    params_message.header = gt_pose->header;
    params_message.apparent_baseline = pose_apparent_baseline;
    params_message.gear_ratio = pose_gear_ratio;
    if (std::abs(current_apparent_baseline) > 0.010000l)
      params_message.last_apparent_baseline = current_apparent_baseline;
    if (std::abs(current_gear_ratio) > 0.010000l)
      params_message.last_gear_ratio = current_gear_ratio;

    calibrated_from_gt_pose_publisher.publish(params_message);
  }
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
  subscriber_gt_pose.subscribe(node_handle, "/gt_pose", 100);

  // publish the topics
  calibrated_from_odom_publisher =
      node_handle.advertise<robotics_hw1::CalibratedParamsOdom>(
          "/apparent_baseline_from_odom", 10);
  calibrated_from_gt_pose_publisher =
      node_handle.advertise<robotics_hw1::CalibratedParamsGtPose>(
          "/apparent_baseline_from_gt_pose", 10);

  // initialize the time syncronizer
  odom_time_syncronizer_ptr.reset(new ExactTimeOdometrySynchronizer(
      OdometryTimePolicy(100), subscriber_front_left, subscriber_front_right,
      subscriber_rear_left, subscriber_rear_rigt, subscriber_scout_odom));

  // register the callback. One placeholder for each function argument
  odom_time_syncronizer_ptr->registerCallback(
      boost::bind(&ParameterCalibrator::calibrate_with_odom_callback, this, _1,
                  _2, _3, _4, _5));

  // initialize the time syncronizer
  gt_pose_time_syncronizer_ptr.reset(new GtPoseSynchronizer(
      GtPoseTimePolicy(100), subscriber_front_left, subscriber_front_right,
      subscriber_rear_left, subscriber_rear_rigt, subscriber_gt_pose));

  // register the callback. One placeholder for each function argument
  gt_pose_time_syncronizer_ptr->registerCallback(
      boost::bind(&ParameterCalibrator::calibrate_with_gt_pose_callback, this,
                  _1, _2, _3, _4, _5));
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
