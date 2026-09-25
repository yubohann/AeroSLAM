// Converts the point cloud published by the simulated Livox LiDAR
// (sensor_msgs/PointCloud) into livox_ros_driver/CustomMsg, which FAST-LIO
// consumes. Simulated scans carry no per-point timing, so offset_time is zero
// (no motion compensation; sufficient for simulation).

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <livox_ros_driver/CustomMsg.h>

namespace aeroslam {

class LivoxSimBridge
{
public:
  LivoxSimBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    std::string input_topic;
    std::string output_topic;
    pnh.param<std::string>("input_topic", input_topic, "/livox/lidar_sim");
    pnh.param<std::string>("output_topic", output_topic, "/livox/lidar");
    pnh.param<std::string>("frame_id", frame_id_, "livox_frame");
    pnh.param<int>("reflectivity", reflectivity_, 100);
    pnh.param<double>("scan_period", scan_period_, 0.0);

    publisher_ = nh.advertise<livox_ros_driver::CustomMsg>(output_topic, 5);
    subscriber_ = nh.subscribe(input_topic, 5, &LivoxSimBridge::cloudCallback, this);

    ROS_INFO_STREAM("livox_sim_bridge: " << input_topic << " (sensor_msgs/PointCloud) -> "
                                         << output_topic << " (livox_ros_driver/CustomMsg)");
  }

private:
  void cloudCallback(const sensor_msgs::PointCloudConstPtr& cloud)
  {
    livox_ros_driver::CustomMsg out;
    out.header = cloud->header;
    if (!frame_id_.empty())
    {
      out.header.frame_id = frame_id_;
    }
    out.timebase = static_cast<uint64_t>(cloud->header.stamp.toNSec());
    out.lidar_id = 0;
    out.point_num = static_cast<uint32_t>(cloud->points.size());
    out.points.resize(cloud->points.size());

    // The simulator emits points in acquisition order but provides no timing.
    // Spread them linearly over one scan period so FAST-LIO can de-skew.
    const double point_count = std::max<size_t>(cloud->points.size(), 1u);
    const double period_ns = scan_period_ * 1e9;
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      const geometry_msgs::Point32& in = cloud->points[i];
      livox_ros_driver::CustomPoint& point = out.points[i];
      point.offset_time = static_cast<uint32_t>(period_ns * static_cast<double>(i) / point_count);
      point.x = in.x;
      point.y = in.y;
      point.z = in.z;
      point.reflectivity = static_cast<uint8_t>(reflectivity_);
      point.tag = 0;
      point.line = 0;
    }

    publisher_.publish(out);
  }

  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  std::string frame_id_;
  int reflectivity_{ 100 };
  double scan_period_{ 0.1 };
};

}  // namespace aeroslam

int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_sim_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  aeroslam::LivoxSimBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
