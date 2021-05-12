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

#include "std_srvs/Empty.h"
#include "robotics_hw1/SetOdometry.h"

#include "tf/transform_broadcaster.h"

#include "dynamic_reconfigure/server.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/time_synchronizer.h"

#include "robotics_hw1/IntegratedOdometry.h"
#include "robotics_hw1/MotorSpeed.h"
#include "robotics_hw1/OdometryCalculatorConfig.h"

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

class OdometryCalculator {
private:
  const ros::NodeHandle node_handle;

  ros::Publisher twist_stamped_publisher;
  ros::Publisher odometry_publisher;
  ros::Publisher odometry_custom_message_publisher;

  ros::ServiceServer reset_odometry_service_server;
  ros::ServiceServer set_odometry_service_server;

  tf2_ros::TransformBroadcaster transform_broadcaster;

  boost::shared_ptr<ExactTimeSynchronizer> time_syncronizer_ptr;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_front_right;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_left;
  message_filters::Subscriber<robotics_hw1::MotorSpeed> subscriber_rear_rigt;

  boost::shared_ptr<
      dynamic_reconfigure::Server<robotics_hw1::OdometryCalculatorConfig>>
      dynamic_reconfigure_server_ptr;

  geometry_msgs::PoseStamped last_pose_stamped;
  bool is_first_measurement = true;

  // parameters initialized at class instantiation.
  const geometry_msgs::Pose initial_pose;
  const double real_baseline;
  const double apparent_baseline;
  const double wheel_radius;
  const double gear_ratio;

  // by default set euler as odometry integration method
  OdometryIntegrationMethod odometry_integration_method = euler;

  nav_msgs::Odometry
  calculateOdometry(const geometry_msgs::TwistStamped &current_twist);

public:
  OdometryCalculator(ros::NodeHandle &node_handle, const double &initial_pose_x,
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

  void
  dynamicReconfigureCallback(robotics_hw1::OdometryCalculatorConfig &config,
                             uint32_t level);

  // We use the empty request and response because we already know the value the
  // odometry is to be reset to
  bool
  resetOdometryServiceCallback(const std_srvs::EmptyRequestConstPtr &request,
                               const std_srvs::EmptyResponseConstPtr &response);

  bool setOdometryServiceCallback(const robotics_hw1::SetOdometryRequestConstPtr &request,
                                  robotics_hw1::SetOdometryResponse response
                                  );
};
