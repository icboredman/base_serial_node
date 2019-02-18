// =============================================================================
// This program enables serial communication with slave base microcontroller and
// publishes odometry and tf messages over ROS.
// In addition, it subscribes to cmd_vel messages and retransmitts them to base.
//
// Copyright (c) 2019 boredman <http://BoredomProjects.net>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <sys/time.h>
#include <time.h>

#include "serial/serial.h"

#include <pthread.h>
pthread_t subscriber_thread;

serial::Serial *uart_;

typedef struct Power {
  uint16_t battery_miV;
  int16_t  battery_miA;
  uint16_t battery_mOhm;
  uint8_t  charger_state;
  uint8_t  bat_percentage;
} tPower;

typedef struct {
  float theta;
  float dx;
  float dth;
  uint16_t dt_ms;
} tOdom;

typedef struct {
  tPower power;
  tOdom  odom;
} tRxPacket;

typedef struct {
  int16_t speed_mm_s;
  int16_t turn_mrad_s;
} tDrive;



/**************************************************************
 * Find CDC device with a given USB HWID
 * and return serial port allocated to this device.
 **************************************************************/
std::string FindSerialDevice(std::string hwid)
{
  // enumerate available devices
  std::vector<serial::PortInfo> devices_found = serial::list_ports();

  // find one with matching ID
  hwid.insert(0, "USB VID:PID=");
  ROS_DEBUG_STREAM("Looking for device " << hwid);

  std::vector<serial::PortInfo>::iterator iter = devices_found.begin();
  while( iter != devices_found.end() )
  {
    serial::PortInfo device = *iter++;
    if (device.hardware_id.compare(0,hwid.length(),hwid) == 0)
    {
      ROS_DEBUG_STREAM("Found device " << hwid << " on port " << device.port);
      return device.port;
    }
  }

  ROS_WARN_STREAM("Device " << hwid << " not found!");
  return "";
}


/**************************************************************
 * Ros callback for 'geometry_msgs::Twist' messages
 **************************************************************/
void callback_cmd_vel(const geometry_msgs::Twist& cmd_vel)
{
  tDrive drive;

  drive.speed_mm_s = (int16_t)(cmd_vel.linear.x * 1000.0);
  drive.turn_mrad_s = (int16_t)(cmd_vel.angular.z * 1000.0);

  if (uart_->isOpen())
    uart_->write((uint8_t*)&drive, sizeof(drive));
  else
    ROS_ERROR_STREAM("Serial port error");
}



/**************************************************************
 * Thread to spin incoming messages loop
 **************************************************************/
void* SubscriberWorker(void* param)
{
  ROS_DEBUG_STREAM("Subscriber Thread started");
  ros::NodeHandle *nh = (ros::NodeHandle*)param;

  ros::Subscriber sub_vel = nh->subscribe("cmd_vel", 10, callback_cmd_vel);

  ros::Duration duration(0.050);
  while(ros::ok())
  {
    ros::spinOnce();
    duration.sleep();
  }

  ROS_DEBUG_STREAM("Subscriber Thread stopped");
}



/**************************************************************
 * Main
 **************************************************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_serial");

  // get parameters from private namespace:
  ros::NodeHandle _nh("~");
  std::string usb_hwid;
  _nh.param("usb_hwid", usb_hwid, (std::string)"16c0:0483");

  std::string portname = FindSerialDevice(usb_hwid);
  if ("" == portname)
  {
    ROS_ERROR_STREAM("No connection to Base");
    exit(1);
  }

  // configure hardware serial port
  // setting both inter_byte_timeout and read_timeout_constant
  // enables thread suspension between bursts of read data
  uart_ = new serial::Serial(portname, 115200, serial::Timeout(10,100,0,250,0));

  if( ! uart_->isOpen() )
  {
    ROS_ERROR("fail init serial device");
    exit(2);
  }

  // publish and subscribe under this namespace:
  ros::NodeHandle nh;

  if (pthread_create(&subscriber_thread, NULL, &SubscriberWorker, (void*)&nh) != 0)
  {
    ROS_ERROR_STREAM("Couldn't create Subscriber THREAD");
    exit(3);
  }

  sensor_msgs::BatteryState bat_msg;
  ros::Publisher pub_bat = nh.advertise<sensor_msgs::BatteryState>("battery", 5);

  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  // populate some constant values
  bat_msg.design_capacity = 2850.0;
  bat_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
  bat_msg.present = true;

  ros::Time packet_recv_time = ros::Time::now();
  ros::Time battery_send_time = ros::Time::now();
  ros::Duration conn_lost_duration(5);	//sec
  bool base_connected = false;

  tRxPacket rxPacket;

  while( ros::ok() )
  {
    // read will suspend execution until packet arrives
    if (uart_->read((uint8_t*)&rxPacket, sizeof(tRxPacket)) == sizeof(tRxPacket))
    {
      packet_recv_time = ros::Time::now();

      double theta = rxPacket.odom.theta;
      double dx = rxPacket.odom.dx;
      double dth = rxPacket.odom.dth;
      double dt = rxPacket.odom.dt_ms / 1000.0;

      double vx = dx / dt;
      double vth = dth / dt;

      // calculate position while translating to "odom" frame
      static double x, y;
      x += dx * cos(theta);
      y += dx * sin(theta);

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header.stamp = packet_recv_time;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";

      odom_tf.transform.translation.x = x;
      odom_tf.transform.translation.y = y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_tf);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = packet_recv_time;
      odom_msg.header.frame_id = "odom";

      //set the position
      odom_msg.pose.pose.position.x = x;
      odom_msg.pose.pose.position.y = y;
      odom_msg.pose.pose.position.z = 0.0;
      odom_msg.pose.pose.orientation = odom_quat;

      //set the velocity
      odom_msg.child_frame_id = "base_link";
      odom_msg.twist.twist.linear.x = vx;
      odom_msg.twist.twist.linear.y = 0.0;
      odom_msg.twist.twist.angular.z = vth;

      //publish the message
      pub_odom.publish(odom_msg);

      if ((ros::Time::now() - battery_send_time) > ros::Duration(0.25))
      { //process battery status
        bat_msg.current = rxPacket.power.battery_miA / 1023.0;
        bat_msg.voltage = rxPacket.power.battery_miV / 1023.0;
        bat_msg.charge = rxPacket.power.battery_mOhm / 1000.0; // used for internal resistance
        bat_msg.power_supply_status = rxPacket.power.charger_state;
        bat_msg.percentage = rxPacket.power.bat_percentage / 100.0;

        pub_bat.publish(bat_msg);
        battery_send_time = ros::Time::now();
      }

      if (!base_connected)
      {
        ROS_INFO("Base sending tf, odom and battery_state");
        base_connected = true;
      }
    }

    if (base_connected && (ros::Time::now() - packet_recv_time) > conn_lost_duration)
    {
      ROS_ERROR("Lost communication with Base");
      base_connected = false;
    }
  }

}

