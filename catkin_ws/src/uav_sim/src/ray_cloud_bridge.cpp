// Gazebo Classic 11 native multi-layer Ray -> ROS PointCloud2.
// Noise is applied only by RaySensor; this node does not sample random values.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

class RayCloudBridge {
 public:
  RayCloudBridge() : nh_("~") {
    nh_.param<std::string>("frame_id", frame_, "radar_link");
    std::string input, output;
    nh_.param<std::string>("gazebo_topic", input,
        "/gazebo/base_scene/radar_3d/radar_link/ray_3d/scan");
    nh_.param<std::string>("cloud_topic", output, "/radar/points");
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output, 2);
    node_.reset(new gazebo::transport::Node());
    node_->Init();
    subscriber_ = node_->Subscribe(input, &RayCloudBridge::OnScan, this);
    ROS_INFO_STREAM("3D Ray bridge " << input << " -> " << output
                    << "; frame=" << frame_ << "; no additional noise");
  }

 private:
  template<typename T>
  void Put(sensor_msgs::PointCloud2 &cloud, size_t offset, const T value) {
    std::memcpy(cloud.data.data() + offset, &value, sizeof(T));
  }
  void OnScan(ConstLaserScanStampedPtr &message) {
    const auto &scan = message->scan();
    const uint32_t width = scan.count(), height = scan.vertical_count();
    if (width < 2 || height < 2 || width > 65535 || height > 65535 ||
        !(scan.angle_max() > scan.angle_min()) ||
        !(scan.vertical_angle_max() > scan.vertical_angle_min()) ||
        static_cast<uint64_t>(width) * height !=
        static_cast<uint64_t>(scan.ranges_size())) {
      ROS_ERROR_THROTTLE(5, "Rejected non-3D / malformed Gazebo Ray scan");
      return;
    }
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = ros::Time(message->time().sec(), message->time().nsec());
    cloud.header.frame_id = frame_;
    cloud.width = width;
    cloud.height = height;
    const std::string names[] = {"x", "y", "z", "range", "ring", "column", "status"};
    const uint32_t offsets[] = {0, 4, 8, 12, 16, 18, 20};
    for (size_t f = 0; f < 7; ++f) {
      sensor_msgs::PointField field;
      field.name = names[f]; field.offset = offsets[f]; field.count = 1;
      field.datatype = f < 4 ? sensor_msgs::PointField::FLOAT32 :
          f < 6 ? sensor_msgs::PointField::UINT16 : sensor_msgs::PointField::UINT8;
      cloud.fields.push_back(field);
    }
    // Declare native byte order because memcpy below writes native values.
    const uint16_t endian_test = 0x0102;
    cloud.is_bigendian = *reinterpret_cast<const uint8_t *>(&endian_test) == 1;
    cloud.is_dense = false; cloud.point_step = 24;
    cloud.row_step = cloud.point_step * width;
    cloud.data.resize(static_cast<size_t>(cloud.row_step) * height, 0);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (uint32_t v = 0; v < height; ++v) {
      const double elevation = scan.vertical_angle_min() +
          v * (scan.vertical_angle_max() - scan.vertical_angle_min()) / (height - 1);
      for (uint32_t h = 0; h < width; ++h) {
        const double azimuth = scan.angle_min() +
            h * (scan.angle_max() - scan.angle_min()) / (width - 1);
        const size_t index = static_cast<size_t>(v) * width + h;
        const double range = scan.ranges(index);
        const bool valid = std::isfinite(range) && range >= scan.range_min() &&
                           range <= scan.range_max();
        // 0 no echo/invalid, 1 interior, 2 below min, 3 min boundary, 4 max boundary.
        uint8_t status = 0;
        if (valid) status = range <= scan.range_min() ? 3 :
                            range >= scan.range_max() ? 4 : 1;
        else if (range < scan.range_min()) status = 2;
        const size_t at = index * cloud.point_step;
        Put<float>(cloud, at, valid ? range * std::cos(elevation) * std::cos(azimuth) : nan);
        Put<float>(cloud, at + 4, valid ? range * std::cos(elevation) * std::sin(azimuth) : nan);
        Put<float>(cloud, at + 8, valid ? range * std::sin(elevation) : nan);
        Put<float>(cloud, at + 12, static_cast<float>(range));
        Put<uint16_t>(cloud, at + 16, v); Put<uint16_t>(cloud, at + 18, h);
        Put<uint8_t>(cloud, at + 20, status);
      }
    }
    publisher_.publish(cloud);
    ROS_INFO_ONCE("Received native %u x %u rays, publishing organized PointCloud2", width, height);
  }
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  gazebo::transport::NodePtr node_;
  gazebo::transport::SubscriberPtr subscriber_;
  std::string frame_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ray_cloud_bridge");
  gazebo::client::setup(argc, argv);
  { RayCloudBridge bridge; ros::spin(); }
  gazebo::client::shutdown();
  return 0;
}
