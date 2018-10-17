// TODO: Add by name feature, latency tracking, occlusion warning, different message formats
#include <map>
#include <utility>
#include <iterator>
#include <stdlib.h>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <libmotioncapture/vicon.h>

static ros::NodeHandle *nh = NULL;

static void object_publish(const libmotioncapture::Object &object, const size_t &frame_num)
{
  static std::map<std::string, ros::Publisher> vicon_publishers;
  std::map<std::string, ros::Publisher>::iterator it;

  it = vicon_publishers.find(object.name());
  if(it == vicon_publishers.end())
  {
    ROS_INFO("Publishing new object: %s", object.name().c_str());
    ros::Publisher pub = nh->advertise<geometry_msgs::PoseStamped>(object.name(), 10);
    it = vicon_publishers.insert(std::make_pair(object.name(), pub)).first;
  }

  if (object.occluded() == false) {
    geometry_msgs::PoseStamped msg;
    Eigen::Vector3f position = object.position();
    Eigen::Quaternionf rotation = object.rotation();
    msg.header.seq = frame_num;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/vicon";
    msg.pose.position.x = position(0);
    msg.pose.position.y = position(1);
    msg.pose.position.z = position(2);
    msg.pose.orientation.x = rotation.vec()(0);
    msg.pose.orientation.y = rotation.vec()(1);
    msg.pose.orientation.z = rotation.vec()(2);
    msg.pose.orientation.w = rotation.w();
    it->second.publish(msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon");

  nh = new ros::NodeHandle("~");

  libmotioncapture::MotionCapture* vicon = nullptr;

  std::string host_name;
  bool enable_objects;
  bool enable_point_cloud;

  nh->getParam("vicon_host_name", host_name);
  nh->param("enable_objects", enable_objects, true);
  nh->param("enable_point_cloud", enable_point_cloud, false);

  vicon = new libmotioncapture::MotionCaptureVicon(host_name,
      enable_objects, enable_point_cloud);

  std::vector<libmotioncapture::Object> objects;

  for (size_t frame_num = 0; ros::ok(); ++frame_num) {
    vicon->waitForNextFrame();
    vicon->getObjects(objects);

    for (auto const& object: objects) {
      object_publish(object, frame_num);
    }

    ros::spinOnce();
  }

  delete nh;

  return 0;
}
