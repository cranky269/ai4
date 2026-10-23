// Independent Gazebo transport source audit: records native sensor measurement stamps.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <fstream>
#include <mutex>
class Audit {
  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr camera, radar;
  std::ofstream file;
  std::mutex lock;
  int camera_sec=-1,camera_nsec=-1;
  void write(const char* kind, const gazebo::msgs::Time& t) {
    std::lock_guard<std::mutex> guard(lock);
    file << kind << ',' << t.sec() << ',' << t.nsec() << '\n'; file.flush();
    if(!file) ROS_FATAL("Native sensor audit disk write failed");
  }
 public:
  Audit() {
    ros::NodeHandle nh("~"); std::string path; nh.getParam("output", path);
    file.open(path); if(!file) throw std::runtime_error("cannot open source audit");
    file << "sensor,stamp_sec,stamp_nsec\n";
    node.reset(new gazebo::transport::Node()); node->Init();
    camera=node->Subscribe("/gazebo/base_scene/static_uav/camera_link/front_camera/image",&Audit::image,this);
    radar=node->Subscribe("/gazebo/base_scene/static_uav/base_link/ray_3d/scan",&Audit::scan,this);
    nh.setParam("ready",true);
  }
  void image(ConstImageStampedPtr& m) {
    if(m->time().sec()==camera_sec && m->time().nsec()==camera_nsec) return;
    camera_sec=m->time().sec();camera_nsec=m->time().nsec();write("camera",m->time());
  }
  void scan(ConstLaserScanStampedPtr& m) { write("radar",m->time()); }
};
int main(int argc,char** argv) {
 ros::init(argc,argv,"sensor_source_audit"); gazebo::client::setup(argc,argv);
 {Audit audit; ros::spin();} gazebo::client::shutdown();return 0;
}
