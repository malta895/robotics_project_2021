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

#include "tf/transform_broadcaster.h"

#include "dynamic_reconfigure/server.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/time_synchronizer.h"

#include "robotics_hw1/IntegratedOdometry.h"
#include "robotics_hw1/MotorSpeed.h"
#include "robotics_hw1/VelocityEstimatorConfig.h"

#include "geometry_msgs/TwistStamped.h"
#include <iomanip>
#include <sstream>
#include <string>

// I verified that in the bags the four messages have exactly synchronized
// timestamps, so I can use the ExactTime policy
typedef message_filters::sync_policies::ExactTime<
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed>
    ExactTimePolicy;

typedef message_filters::Synchronizer<ExactTimePolicy> ExactTimeSynchronizer;

// let's take the double value of pi from boost library
const double pi = boost::math::constants::pi<double>();

enum OdometryIntegrationMethod { euler = 0, rungeKutta };
const char *odometry_integration_method_name[] = {"Euler", "Runge-Kutta"};

class VelocityEstimator {
private:
  const ros::NodeHandle node_handle;

  ros::Publisher twist_stamped_publisher;
  ros::Publisher odometry_publisher;
  ros::Publisher odometry_custom_message_publisher;

  tf2_ros::TransformBroadcaster transform_broadcaster;

  boost::shared_ptr<ExactTimeSynchronizer> time_syncronizer_ptr;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_right;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_rigt;

  boost::shared_ptr<
      dynamic_reconfigure::Server<robotics_hw1::VelocityEstimatorConfig>>
      dynamic_reconfigure_server_ptr;

  const geometry_msgs::Pose initial_pose;
  geometry_msgs::PoseStamped last_pose_stamped;
  bool is_first_measurement = true;

  const double real_baseline;
  const double apparent_baseline;
  const double wheel_radius;
  const double gear_ratio;

  // by default set euler as odometry integration method
  OdometryIntegrationMethod odometry_integration_method = euler;

  nav_msgs::Odometry calculateOdometry(const double &linear_speed,
                                       const double &angular_speed,
                                       const std_msgs::Header &header);

public:
  VelocityEstimator(ros::NodeHandle &node_handle, const double &initial_pose_x,
                    const double &initial_pose_y,
                    const double &initial_pose_theta,
                    const double &wheel_radius, const double &real_baseline,
                    const double &gear_ratio, const double &apparent_baseline,
                    std::string &pose_or_odom);

  void motorsSyncCallback(
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_front_right,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_left,
      const robotics_hw1::MotorSpeedConstPtr &motor_speed_rear_right);

  void dynamicReconfigureCallback(robotics_hw1::VelocityEstimatorConfig &config,
                                  uint32_t level);
};

void VelocityEstimator::dynamicReconfigureCallback(
    robotics_hw1::VelocityEstimatorConfig &config, uint32_t level) {

  odometry_integration_method =
      OdometryIntegrationMethod(config.odometry_integration_method);

  ROS_INFO(
      "Integration method changed to %s",
      odometry_integration_method_name[config.odometry_integration_method]);
}

nav_msgs::Odometry
VelocityEstimator::calculateOdometry(const double &linear_speed,
                                     const double &angular_speed,
                                     const std_msgs::Header &current_header) {
  nav_msgs::Odometry calculated_odometry;
  calculated_odometry.header = current_header;
  calculated_odometry.header.frame_id = "odom";
  calculated_odometry.child_frame_id = "base_link";

  const double delta_time =
      (current_header.stamp - last_pose_stamped.header.stamp).toSec();

  // Let's get theta angle from the orientation quaternion
  tf::Quaternion quaternion(last_pose_stamped.pose.orientation.x,
                            last_pose_stamped.pose.orientation.y,
                            last_pose_stamped.pose.orientation.z,
                            last_pose_stamped.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3 quaternion_matrix(quaternion);
  quaternion_matrix.getRPY(roll, pitch, yaw);

  double &theta = yaw;

  switch (odometry_integration_method) {
  case euler:

    // x_{k+1}
    calculated_odometry.pose.pose.position.x =
        last_pose_stamped.pose.position.x +
        linear_speed * delta_time * std::cos(theta);

    // y_{k+1}
    calculated_odometry.pose.pose.position.y =
        last_pose_stamped.pose.position.y +
        linear_speed * delta_time * std::sin(theta);

    // theta_{k+1}
    calculated_odometry.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(theta + angular_speed * delta_time);

    break;
  case rungeKutta:

    // x_{k+1}
    calculated_odometry.pose.pose.position.x =
        last_pose_stamped.pose.position.x +
        linear_speed * delta_time *
            std::cos(theta + (angular_speed * delta_time / 2));

    // y_{k+1}
    calculated_odometry.pose.pose.position.y =
        last_pose_stamped.pose.position.y +
        linear_speed * delta_time *
            std::sin(theta + (angular_speed * delta_time / 2));

    // theta_{k+1}
    calculated_odometry.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(theta + angular_speed * delta_time);

    break;
  }

  // the speed is assumed to be equal to the previous one
  calculated_odometry.twist.twist.linear.x = linear_speed;
  calculated_odometry.twist.twist.angular.z = angular_speed;
  return calculated_odometry;
}

void VelocityEstimator::motorsSyncCallback(
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

  if (is_first_measurement) {
    last_pose_stamped.header = velocity_message.header;
    last_pose_stamped.pose = initial_pose;
    is_first_measurement = false;
  }

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
  double linear_speed =
      ((omega_l * wheel_radius) + (omega_r * wheel_radius)) / 2;
  velocity_message.twist.linear.x = linear_speed;

  // we use the formula with the apparent baseline to get omega_z, i.e. the
  // angular speed of the robot when it turns
  double angular_speed =
      (-(omega_l * wheel_radius) + (omega_r * wheel_radius)) /
      apparent_baseline;
  velocity_message.twist.angular.z = angular_speed;

  twist_stamped_publisher.publish(velocity_message);

  const nav_msgs::Odometry odometry_message = calculateOdometry(
      linear_speed, angular_speed, motor_speed_front_left->header);

  odometry_publisher.publish(odometry_message);

  robotics_hw1::IntegratedOdometry integrated_odometry_custom_message;
  integrated_odometry_custom_message.odom = odometry_message;
  integrated_odometry_custom_message.method.data =
      odometry_integration_method_name[odometry_integration_method];

  odometry_custom_message_publisher.publish(integrated_odometry_custom_message);

  last_pose_stamped.pose = odometry_message.pose.pose;
  last_pose_stamped.header = odometry_message.header;
}

geometry_msgs::Pose pose_from_x_y_theta(const double &x, const double &y,
                                        const double &theta) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  return pose;
}

VelocityEstimator::VelocityEstimator(
    ros::NodeHandle &node_handle, const double &initial_pose_x,
    const double &initial_pose_y, const double &initial_pose_theta,
    const double &wheel_radius, const double &real_baseline,
    const double &gear_ratio, const double &apparent_baseline,
    std::string &pose_or_odom)
    : node_handle(node_handle),
      initial_pose(pose_from_x_y_theta(initial_pose_x, initial_pose_y,
                                       initial_pose_theta)),
      wheel_radius(wheel_radius), real_baseline(real_baseline),
      gear_ratio(gear_ratio), apparent_baseline(apparent_baseline) {

  // subscribe to the motor speed topics TODO remove hard coding and put
  // parameters/argvs with topics names

  subscriber_front_right.subscribe(node_handle, "/motor_speed_fr", 100);
  subscriber_front_left.subscribe(node_handle, "/motor_speed_fl", 100);
  subscriber_rear_rigt.subscribe(node_handle, "/motor_speed_rr", 100);
  subscriber_rear_left.subscribe(node_handle, "/motor_speed_rl", 100);

  // publish the topics
  twist_stamped_publisher = node_handle.advertise<geometry_msgs::TwistStamped>(
      "/robot_twisted_stamped", 10);

  // TODO probably this is useless, the last one is enough
  odometry_publisher = node_handle.advertise<nav_msgs::Odometry>(
      "/scout_integrated_odom/from_" + pose_or_odom, 10);

  odometry_custom_message_publisher =
      node_handle.advertise<robotics_hw1::IntegratedOdometry>(
          "/scout_integrated_odom_custom", 10);

  // initialize the time syncronizer
  time_syncronizer_ptr.reset(new ExactTimeSynchronizer(
      ExactTimePolicy(100), subscriber_front_left, subscriber_front_right,
      subscriber_rear_left, subscriber_rear_rigt));

  // register the callback. One placeholder for each function argument
  time_syncronizer_ptr->registerCallback(boost::bind(
      &VelocityEstimator::motorsSyncCallback, this, _1, _2, _3, _4));

  // set up the dynamic reconfigure server
  dynamic_reconfigure_server_ptr.reset(
      new dynamic_reconfigure::Server<robotics_hw1::VelocityEstimatorConfig>);
  dynamic_reconfigure_server_ptr->setCallback(boost::bind(
      &VelocityEstimator::dynamicReconfigureCallback, this, _1, _2));
}

double getDoubleParameter(const std::string &parameter_key,
                          const ros::NodeHandle &node_handle) {
  // this is just an helper function used to retrieve the double params from ROS
  // environment. I need this because in C++ I can initialize const in classes
  // only by calling a function in the member initializer list of the
  // constructor. I am using such const members for consinstency with the fact
  // that params are not supposed to be modified

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
  // a string which is either scout_odom or gt_pose, to take the according param
  // of the estimated apparent baseline and gear ratio
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
      node_handle, getDoubleParameter("initial_pose_x", node_handle),
      getDoubleParameter("initial_pose_y", node_handle),
      getDoubleParameter("initial_pose_theta", node_handle),
      getDoubleParameter("wheel_radius", node_handle),
      getDoubleParameter("real_baseline", node_handle),
      getDoubleParameter("gear_ratio_from_" + pose_or_odom, node_handle),
      getDoubleParameter("apparent_baseline_from_" + pose_or_odom, node_handle),
      pose_or_odom);

  ros::spin();

  return 0;
}
