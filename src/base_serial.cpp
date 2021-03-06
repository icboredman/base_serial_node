// =============================================================================
// This program enables serial communication with slave base microcontroller and
// publishes odometry and tf messages over ROS.
// In addition, it subscribes to cmd_vel messages and retransmitts them to slave.
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

#include <fstream>      // std::ifstream
#include <sensor_msgs/Temperature.h>

#include "serial/serial.h"
#define  MSG_SERIAL_SKIP_CRC
#include "MessageSerial/MessageSerial.h"

#include <pthread.h>
pthread_t subscriber_thread;


typedef struct {
  int16_t speed_mm_s;
  int16_t turn_mrad_s;
} tDrive;
Message<tDrive,7> *drive_;



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
  drive_->data.speed_mm_s = (int16_t)(cmd_vel.linear.x * 1000.0);
  drive_->data.turn_mrad_s = (int16_t)(cmd_vel.angular.z * 1000.0);
  if (!drive_->send())
    ROS_ERROR("Drive send failed");
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

  // configure connection to Base controller (Teensy)
  std::string usb_hwid;
  _nh.param("base_usb_hwid", usb_hwid, (std::string)"16c0:0483");

  std::string portname = FindSerialDevice(usb_hwid);
  if ("" == portname)
  {
    ROS_ERROR_STREAM("No connection to Base");
    exit(1);
  }

  // configure hardware serial port
  // setting both inter_byte_timeout and read_timeout_constant
  // enables thread suspension between bursts of read data
  serial::Serial base_uart(portname, 115200, serial::Timeout(20,20,0,250,0));

  if (!base_uart.isOpen())
  {
    ROS_ERROR("Unable to open Base port: %s", portname.c_str());
    exit(2);
  }
  else
    ROS_INFO("Base port found and opened: %s", portname.c_str());

  // MessageSerial object will use the above port
  MessageSerial serial(base_uart);

  // define actual messages and create corresponding Message objects
  typedef struct Power {
    uint16_t battery_miV;
    int16_t  battery_miA;
    uint16_t battery_mOhm;
    uint8_t  charger_state;
    uint8_t  bat_percentage;
  } tPower;
  // Message objects should reference the above MessageSerial object
  Message<tPower,2> power(serial);

  typedef struct {
    float theta;
    float dx;
    float dth;
    uint16_t dt_ms;
  } tOdom;
  Message<tOdom,5> odom(serial);

  drive_ = new Message<tDrive,7>(serial);

  typedef struct {
    char str[100];
  } tStr;
  Message<tStr,1> text(serial); // message id 1 is reserved for character string messages

  // configure connection to Fan Controller (Leonardo on LattePanda)
  _nh.param("fan_usb_hwid", usb_hwid, (std::string)"2341:8036");

  portname = FindSerialDevice(usb_hwid);
  if ("" == portname)
  {
    ROS_ERROR_STREAM("No connection to Fan Controller");
    exit(3);
  }

  // configure hardware serial port
  // we don't care about timeouts here
  serial::Serial fan_uart(portname, 115200);

  if (!fan_uart.isOpen())
  {
    ROS_ERROR("Unable to open Fan port: %s", portname.c_str());
    exit(4);
  }
  else
    ROS_INFO("Fan port found and opened: %s", portname.c_str());

  // seems to be necessary for Leonardo on LattePanda
  fan_uart.setDTR();
  fan_uart.setRTS();

  // publish and subscribe under this namespace:
  ros::NodeHandle nh;

  if (pthread_create(&subscriber_thread, NULL, &SubscriberWorker, (void*)&nh) != 0)
  {
    ROS_ERROR_STREAM("Couldn't create Subscriber THREAD");
    exit(5);
  }

  sensor_msgs::BatteryState bat_msg;
  ros::Publisher pub_bat = nh.advertise<sensor_msgs::BatteryState>("battery", 5);

  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  // populate some constant values
  bat_msg.design_capacity = 2850.0;
  bat_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
  bat_msg.present = true;

  ros::Time last_recv_time = ros::Time::now();
  bool conn_lost_timeout = false;
  bool bat_connected = false;
  bool odom_connected = false;

  ros::Time last_temp_reading = ros::Time::now();
  sensor_msgs::Temperature temp_msg;
  ros::Publisher pub_temp = nh.advertise<sensor_msgs::Temperature>("temperature", 5);

  while( ros::ok() )
  {
    // message processing function
    serial.update();

    if( power.available() )
    {
      last_recv_time = ros::Time::now();

      bat_msg.header.stamp = last_recv_time;
      bat_msg.current = power.data.battery_miA / 1023.0;
      bat_msg.voltage = power.data.battery_miV / 1023.0;
      bat_msg.charge = power.data.battery_mOhm / 1000.0; // use for internal resistance
      bat_msg.power_supply_status = power.data.charger_state;
      bat_msg.percentage = power.data.bat_percentage / 100.0;
      power.ready();
      pub_bat.publish(bat_msg);

      if( ! bat_connected )
      {
        ROS_INFO("Base sending BatteryState");
        bat_connected = true;
      }
    }

    if( odom.available() )
    {
      last_recv_time = ros::Time::now();

      double theta = odom.data.theta;
      double dx = odom.data.dx;
      double dth = odom.data.dth;
      double dt = odom.data.dt_ms / 1000.0;
      odom.ready();

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
      odom_tf.header.stamp = last_recv_time;
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
      odom_msg.header.stamp = last_recv_time;
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

      if( ! odom_connected )
      {
        ROS_INFO("Base sending tf and odom");
        odom_connected = true;
      }
    }

    if( text.available() )
    {
      last_recv_time = ros::Time::now();
      ROS_INFO_STREAM(text.data.str);
      text.ready();
    }

    if( !conn_lost_timeout && (ros::Time::now() - last_recv_time) > ros::Duration(5) )
    {
      ROS_ERROR("Lost communication with Base");
      bat_connected = false;
      odom_connected = false;
      conn_lost_timeout = true;
    }
    if( bat_connected || odom_connected )
      conn_lost_timeout = false;

    if ((ros::Time::now() - last_temp_reading) > ros::Duration(2))
    {
      last_temp_reading = ros::Time::now();

      // read temperature
      int cpu_temp_millideg = 0;
      std::ifstream ifthermal("/sys/class/thermal/thermal_zone3/temp", std::ifstream::in);
      if (!ifthermal.good())
      {
        ROS_ERROR("Unable to read CPU temperature");
        exit(6);
      }
      else
      {
        ifthermal >> cpu_temp_millideg;
        ifthermal.close();
      }

      // send to Arduino (Leonardo)
      uint8_t cpu_temp_deg = cpu_temp_millideg / 1000;
      fan_uart.write(&cpu_temp_deg, 1);

      // publish as ROS message
      temp_msg.header.stamp = last_temp_reading;
      temp_msg.temperature = cpu_temp_deg;
      temp_msg.variance = 0.0;
      pub_temp.publish(temp_msg);
    }
  }

}
