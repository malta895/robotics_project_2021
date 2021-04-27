#include "boost/bind/bind.hpp"
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

// Speed message structure:
// Header header
// float64 rpm

// I verified that in the bags the four messages have exactly synchronized
// timestamps, thus I can use the ExactTime policy
typedef message_filters::sync_policies::ExactTime<
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed>
    ExactTimePolicy;

typedef message_filters::Synchronizer<ExactTimePolicy> ExactTimeSynchronizer;

class VelocityEstimator {
private:
  ros::NodeHandle node_handle;

  ros::Publisher publisher;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_right;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_rigt;

  boost::shared_ptr<ExactTimeSynchronizer> time_syncronizer_ptr;

  const double initial_pose_x;
  const double initial_pose_y;
  const double initial_pose_theta;

  double get_double_parameter(const std::string &parameter_key);

public:
  VelocityEstimator();
  void sync_and_publish_velocity_message(
      const robotics_hw1::MotorSpeedConstPtr &message_front_left,
      const robotics_hw1::MotorSpeedConstPtr &message_front_right,
      const robotics_hw1::MotorSpeedConstPtr &message_rear_left,
      const robotics_hw1::MotorSpeedConstPtr &message_rear_right);
};

void VelocityEstimator::sync_and_publish_velocity_message(
    const robotics_hw1::MotorSpeedConstPtr &message_front_left,
    const robotics_hw1::MotorSpeedConstPtr &message_front_right,
    const robotics_hw1::MotorSpeedConstPtr &message_rear_left,
    const robotics_hw1::MotorSpeedConstPtr &message_rear_right) {

  geometry_msgs::TwistStamped velocity_message;

  // take the Header from one of the messages, no matter which one, they are all
  // the same because of ExactTimePolicy
  velocity_message.header = message_front_left->header;

  //TODO calculate velocity

  publisher.publish(velocity_message);
}

double VelocityEstimator::get_double_parameter(const std::string &parameter_key) {

  // we take the parameter from the node_handle
  double parameter_value;
  if (!node_handle.getParam(parameter_key, parameter_value) ) {
    ROS_ERROR("Parameter %s not provided! Defaulting to 0",
              parameter_key.c_str());
    return 0;
  }
  return parameter_value;
}

// the constructor will just take the parameters and set them to the const
// variables
VelocityEstimator::VelocityEstimator()
    : initial_pose_x(get_double_parameter("initial_pose_x")),
      initial_pose_y(get_double_parameter("initial_pose_y")),
      initial_pose_theta(get_double_parameter("initial_pose_theta")) {

  publisher = node_handle.advertise<geometry_msgs::TwistStamped>(
      "/robot_twisted_stamped", 10);

  // initialize the time syncronizer
  time_syncronizer_ptr.reset(new ExactTimeSynchronizer(
      ExactTimePolicy(10), subscriber_front_left, subscriber_front_right,
      subscriber_rear_left, subscriber_rear_rigt));

  time_syncronizer_ptr->registerCallback(
      boost::bind(&VelocityEstimator::sync_and_publish_velocity_message, this,
                  _1, _2, _3, _4));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "tf_publisher");

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
