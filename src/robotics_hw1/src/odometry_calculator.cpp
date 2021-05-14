#include "include/odometry_calculator.hpp"

void OdometryCalculator::dynamicReconfigureCallback(
    robotics_hw1::OdometryCalculatorConfig &config, uint32_t level) {

  odometry_integration_method =
      OdometryIntegrationMethod(config.odometry_integration_method);

  ROS_INFO(
      "Integration method changed to %s",
      odometry_integration_method_name[config.odometry_integration_method]);
}

nav_msgs::Odometry OdometryCalculator::calculateOdometry(
    const geometry_msgs::TwistStamped &current_twist) {

  nav_msgs::Odometry calculated_odometry;
  calculated_odometry.header = current_twist.header;

  calculated_odometry.child_frame_id = pose_or_odom + "_link";

  // the twist (i.e. the velocities)
  calculated_odometry.twist.twist = current_twist.twist;

  const double &delta_time =
      (current_twist.header.stamp - last_pose_stamped.header.stamp).toSec();

  const double &linear_speed = current_twist.twist.linear.x;
  const double &angular_speed = current_twist.twist.angular.z;

  // Let's get theta angle from the orientation quaternion
  tf::Quaternion quaternion(last_pose_stamped.pose.orientation.x,
                            last_pose_stamped.pose.orientation.y,
                            last_pose_stamped.pose.orientation.z,
                            last_pose_stamped.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  const double &theta = yaw;

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

  return calculated_odometry;
}

void OdometryCalculator::motorsSyncCallback(
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
  std_msgs::Header header = motor_speed_front_left->header;
  header.frame_id = "odom";

  velocity_message.header = header;

  if (is_first_measurement) {
    last_pose_stamped.header = velocity_message.header;
    last_pose_stamped.pose = initial_pose;
    is_first_measurement = false;
  }

  // Since often on same timestamps the rpm of the front and back wheels on the
  // same side differ, maybe because of measurement errors or due the fact that
  // it is not an ideal robot, we take the mean of the two measurements

  // the left rotation is registered as backwards, so we invert it
  const double rpm_left =
      -(motor_speed_front_left->rpm + motor_speed_rear_left->rpm) / 2;
  const double rpm_right =
      (motor_speed_front_right->rpm + motor_speed_rear_right->rpm) / 2;

  const double omega_l = (2 * pi) * ((rpm_left * gear_ratio) / 60);
  const double omega_r = (2 * pi) * ((rpm_right * gear_ratio) / 60);

  // from the rotational speeds and the wheel radius we can easily estimate the
  // linear speed
  velocity_message.twist.linear.x =
      ((omega_l * wheel_radius) + (omega_r * wheel_radius)) / 2;

  // we use the formula with the apparent baseline to get omega_z, i.e. the
  // angular speed of the robot when it turns
  velocity_message.twist.angular.z =
      (-(omega_l * wheel_radius) + (omega_r * wheel_radius)) /
      apparent_baseline;

  twist_stamped_publisher.publish(velocity_message);

  const nav_msgs::Odometry odometry_message =
      calculateOdometry(velocity_message);

  odometry_publisher.publish(odometry_message);

  // build the custom message
  robotics_hw1::IntegratedOdometry integrated_odometry_custom_message;
  integrated_odometry_custom_message.odom = odometry_message;
  integrated_odometry_custom_message.method.data =
      odometry_integration_method_name[odometry_integration_method];

  odometry_custom_message_publisher.publish(integrated_odometry_custom_message);

  last_pose_stamped.pose = odometry_message.pose.pose;
  last_pose_stamped.header = header;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = pose_or_odom + "_link";

  transform_stamped.transform.rotation = odometry_message.pose.pose.orientation;
  transform_stamped.transform.translation.x =
      odometry_message.pose.pose.position.x;
  transform_stamped.transform.translation.y =
      odometry_message.pose.pose.position.y;

  transform_broadcaster.sendTransform(transform_stamped);
}

geometry_msgs::Pose createPoseMsgFromXYTheta(const double &x, const double &y,
                                             const double &theta) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  return pose;
}

OdometryCalculator::OdometryCalculator(
    ros::NodeHandle &node_handle, const double &initial_pose_x,
    const double &initial_pose_y, const double &initial_pose_theta,
    const double &wheel_radius, const double &real_baseline,
    const double &gear_ratio, const double &apparent_baseline,
    const std::string &pose_or_odom)
    : node_handle(node_handle),
      initial_pose(createPoseMsgFromXYTheta(initial_pose_x, initial_pose_y,
                                            initial_pose_theta)),
      wheel_radius(wheel_radius), real_baseline(real_baseline),
      gear_ratio(gear_ratio), apparent_baseline(apparent_baseline),
      pose_or_odom(pose_or_odom) {

  subscriber_front_right.subscribe(node_handle, "/motor_speed_fr", 100);
  subscriber_front_left.subscribe(node_handle, "/motor_speed_fl", 100);
  subscriber_rear_rigt.subscribe(node_handle, "/motor_speed_rr", 100);
  subscriber_rear_left.subscribe(node_handle, "/motor_speed_rl", 100);

  // I put the node name in the topic names to avoid conflicts in case more
  // instances are run at the same time
  const std::string node_name = ros::this_node::getName();

  // publish the topics.
  twist_stamped_publisher = node_handle.advertise<geometry_msgs::TwistStamped>(
      node_name + "/twist_stamped", 10);

  // the odometry
  odometry_publisher = node_handle.advertise<nav_msgs::Odometry>(
      node_name + "/integrated_odom", 10);

  // the custom message, contains odometry and method used as a string
  odometry_custom_message_publisher =
      node_handle.advertise<robotics_hw1::IntegratedOdometry>(
          node_name + "/integrated_odom_and_method", 10);

  // initialize the time syncronizer to get the speed of each wheel
  time_syncronizer_ptr.reset(new ExactTimeSynchronizer(
      ExactTimePolicy(100), subscriber_front_left, subscriber_front_right,
      subscriber_rear_left, subscriber_rear_rigt));

  // register the syncronizer callback
  time_syncronizer_ptr->registerCallback(boost::bind(
      &OdometryCalculator::motorsSyncCallback, this, _1, _2, _3, _4));

  // set up the dynamic reconfigure server
  dynamic_reconfigure_server_ptr.reset(
      new dynamic_reconfigure::Server<robotics_hw1::OdometryCalculatorConfig>);
  dynamic_reconfigure_server_ptr->setCallback(boost::bind(
      &OdometryCalculator::dynamicReconfigureCallback, this, _1, _2));

  // register the services. I put the name of the node in case more instances
  // are running at once
  reset_odometry_service_server = node_handle.advertiseService(
      node_name + "/reset_odometry",
      &OdometryCalculator::resetOdometryServiceCallback, this);
  set_odometry_service_server = node_handle.advertiseService(
      node_name + "/set_odometry",
      &OdometryCalculator::setOdometryServiceCallback, this);
}

bool OdometryCalculator::resetOdometryServiceCallback(
    robotics_hw1::ResetOdometry::Request &request,
    robotics_hw1::ResetOdometry::Response &response) {

  // set everything to zero

  last_pose_stamped.pose = createPoseMsgFromXYTheta(0, 0, 0);

  response.outcome = "Odometry reset succeeded. The pose and orientation have "
                     "been set to zero";

  return true;
}

bool OdometryCalculator::setOdometryServiceCallback(
    robotics_hw1::SetOdometry::Request &request,
    robotics_hw1::SetOdometry::Response &response) {

  // calculate the pose
  const geometry_msgs::Pose &new_pose =
      createPoseMsgFromXYTheta(request.x, request.y, request.theta);

  // set it to the last_pose_stamped variable, so that it will be used in the
  // next computations
  last_pose_stamped.pose = new_pose;

  // respond with the new pose as a confirmation to the client
  response.new_pose = new_pose;
  response.outcome = "The new Odometry has been set successfully";

  return true;
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

  ros::init(argc, argv, "odometry_calculator");

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

  const std::string &pose_or_odom = args[1];

  ros::NodeHandle node_handle;
  OdometryCalculator odometry_calculator(
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
