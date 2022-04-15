#ifndef TF_MANAGER_ROS_HPP
#define TF_MANAGER_ROS_HPP

#include <ros/ros.h>

// ROS TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS TF Msg
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>

// ROS link between TF and TF2
#include <tf/transform_datatypes.h>

// ROS TF
#include <tf/transform_broadcaster.h>

class RosTFManager
{
private:
public:
  tf2_ros::Buffer mTF2Buffer;
  tf2_ros::TransformListener mTF2Listener;
  tf2_ros::TransformBroadcaster mTF2BroadCaster;
  tf::TransformBroadcaster mTFBroadCaster;

  geometry_msgs::Transform MapToBase;
  geometry_msgs::Transform BaseToMap;
  geometry_msgs::Transform OdomToBase;
  geometry_msgs::Transform BaseToOdom;
  geometry_msgs::Transform SensorToBase;

  geometry_msgs::Transform refBaseToMap;


public:
  RosTFManager();
  ~RosTFManager() = default;

  bool lookupTransform(const std::string &targetFrame,
                       const std::string &sourceFrame,
                       geometry_msgs::Transform &transform,
                       ros::Time time = ros::Time(0));

  void sendTransform(const std::string &frame,
                     const std::string &childFrame, geometry_msgs::Transform &transform,
                     ros::Time time);
  void sendTransform(const std::string &frame,
                     const std::string &childFrame, tf::Transform &transform,
                     ros::Time time);

  // TF getter
  bool getTF(const ros::Time &time);
  bool getStaticTF(const std::string &sensorFrame);

  static void getRPYfromMsg(const geometry_msgs::Transform &tfMsg, double &roll, double &pitch, double &yaw);

  static geometry_msgs::Transform fuseTransform(const geometry_msgs::Transform &AtoB,
                                                const geometry_msgs::Transform &BtoC);
};

// Definition
RosTFManager::RosTFManager()
    : mTF2Listener(mTF2Buffer)
{
}

bool RosTFManager::lookupTransform(const std::string &targetFrame,
                                   const std::string &sourceFrame, geometry_msgs::Transform &transform,
                                   ros::Time time)
{
  geometry_msgs::TransformStamped tfStamped;
  try
  {
    tfStamped = mTF2Buffer.lookupTransform(targetFrame, sourceFrame, time, ros::Duration(1.0));
    transform = tfStamped.transform;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  return true;
}

void RosTFManager::sendTransform(const std::string &frame,
                                 const std::string &childFrame, geometry_msgs::Transform &transform,
                                 ros::Time time)
{
  geometry_msgs::TransformStamped tfMsg;
  tfMsg.header.stamp = time;
  tfMsg.header.frame_id = frame;
  tfMsg.child_frame_id = childFrame;
  tfMsg.transform = transform;

  mTF2BroadCaster.sendTransform(tfMsg);
}
void RosTFManager::sendTransform(const std::string &frame,
                                 const std::string &childFrame, tf::Transform &transform,
                                 ros::Time time)
{
  mTFBroadCaster.sendTransform(tf::StampedTransform(transform, time, frame, childFrame));
}

bool RosTFManager::getTF(const ros::Time &time)
{
  if (!lookupTransform("map", "base_link", BaseToMap, time))
    return false;
  if (!lookupTransform("base_link", "map", MapToBase, time))
    return false;
  if (!lookupTransform("odom", "base_link", BaseToOdom, time))
    return false;
  if (!lookupTransform("base_link", "odom", OdomToBase, time))
    return false;

  return true;
}

bool RosTFManager::getStaticTF(const std::string &sensorFrame)
{
  if (!lookupTransform("base_link", sensorFrame, SensorToBase))
    return false;

  return true;
}

geometry_msgs::Transform RosTFManager::fuseTransform(const geometry_msgs::Transform &AtoB,
                                                     const geometry_msgs::Transform &BtoC)
{
  geometry_msgs::Transform AtoC;
  tf2::Transform tf1, tf2, tfOut;

  tf2::fromMsg(AtoB, tf1);
  tf2::fromMsg(BtoC, tf2);
  tfOut = tf2 * tf1;
  AtoC = tf2::toMsg(tfOut);

  return AtoC;
}

void RosTFManager::getRPYfromMsg(const geometry_msgs::Transform &tfMsg, double &roll, double &pitch, double &yaw)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(tfMsg.rotation, q);
  tf::Matrix3x3 m(q);

  m.getRPY(roll, pitch, yaw);
}

#endif