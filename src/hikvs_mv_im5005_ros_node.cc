/**
 * Copyright (c) 2020 CoTEK Inc. All rights reserved.
 */
#include <ros/ros.h>

#include "hikvs_mv_im5005_ros/hikvs_mv_im5005.h"
#include "hikvs_mv_im5005_ros/hikvs_mv_im5005_driver.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hikvs_mv_im5005");

  // Set ros log level:
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string sensor_ip;
  int sensor_port = 0;
  double frequency = 0.;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("sensor_ip", sensor_ip, std::string("192.168.0.20"));
  private_nh.param("sensor_port", sensor_port, 7777);
  private_nh.param("frequency", frequency, 50.0);

  ros::Publisher sensor_publisher =
      nh.advertise<hikvs_mv_im5005_ros::hikvs_mv_im5005>("hikvs_qr_feedback",
                                                         2);

  hikvs_mv::HikvsMvIm5005Driver driver;
  if (driver.Connect(sensor_ip, sensor_port)) {
    driver.Run();
  }

  ros::Rate rate(frequency);  // 默认50hz
  while (ros::ok()) {
    hikvs_mv_im5005_ros::hikvs_mv_im5005 driver_msg;
    driver_msg.time_stamp = driver.time_stamp();
    driver_msg.tag_number = driver.tag_number();
    driver_msg.x_deviation = driver.x_deviation();
    driver_msg.y_deviation = driver.y_deviation();
    driver_msg.theta = driver.theta();
    driver_msg.error_code = driver.GetErrorCode();
    
    sensor_publisher.publish(driver_msg);

    rate.sleep();
  }

  ros::spin();
  return 0;
}
