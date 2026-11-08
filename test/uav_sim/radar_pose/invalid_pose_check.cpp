// Fault injection against the real bridge through Gazebo transport, not a mock.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <set>
#include <limits>
#include <cmath>
#include <fstream>
#include <cstring>
int main(int argc, char **argv) {
  ros::init(argc,argv,"radar_pose_fault_check"); gazebo::client::setup(argc,argv);
  ros::NodeHandle nh;
  std::map<unsigned,sensor_msgs::PointCloud2> clouds;std::map<unsigned,geometry_msgs::PoseStamped> poses;
  auto c=nh.subscribe<sensor_msgs::PointCloud2>("/fixture/points",20,[&](const sensor_msgs::PointCloud2::ConstPtr &m){clouds[m->header.stamp.sec]=*m;});
  auto p=nh.subscribe<geometry_msgs::PoseStamped>("/fixture/pose",20,[&](const geometry_msgs::PoseStamped::ConstPtr &m){poses[m->header.stamp.sec]=*m;});
  gazebo::transport::NodePtr node(new gazebo::transport::Node());node->Init("base_scene");
  auto pub=node->Advertise<gazebo::msgs::LaserScanStamped>("~/pose_fixture/scan");
  for(int i=0;i<100 && !pub->HasConnections();++i){ros::spinOnce();ros::WallDuration(.1).sleep();}
  if(!pub->HasConnections())return 2;
  ros::WallDuration(1).sleep();
  for(int i=0;i<5;++i){
    gazebo::msgs::LaserScanStamped msg;
    msg.mutable_time()->set_sec(i==4?1:100+i);msg.mutable_time()->set_nsec(123);
    auto s=msg.mutable_scan();s->set_frame(i==3?"target_uav::base_link":"static_uav::base_link");
    gazebo::msgs::Set(s->mutable_world_pose(),ignition::math::Pose3d(10,20,30,0,0,1.5707963267948966));
    if(i==1)s->mutable_world_pose()->mutable_orientation()->set_w(0);
    if(i==2)s->mutable_world_pose()->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
    s->set_angle_min(-.1);s->set_angle_max(.1);s->set_angle_step(.2);s->set_count(2);
    s->set_vertical_angle_min(-.1);s->set_vertical_angle_max(.1);s->set_vertical_angle_step(.2);s->set_vertical_count(2);
    s->set_range_min(.1);s->set_range_max(10);
    for(int j=0;j<4;++j){s->add_ranges(j==3?std::numeric_limits<double>::infinity():2);s->add_intensities(0);}
    pub->Publish(msg,true);
    for(int j=0;j<15;++j){ros::spinOnce();ros::WallDuration(.02).sleep();}
  }
  bool ok=clouds.size()==2 && poses.size()==2 && poses.count(100) && poses.count(1);
  for(const auto &item:poses){const auto &m=item.second;
    ok=ok && m.header.stamp.nsec==123 && m.header.frame_id=="world" &&
      std::abs(m.pose.position.x-10)<1e-9 && std::abs(m.pose.position.y-19.7)<1e-9 &&
      std::abs(m.pose.position.z-29.88)<1e-9 && std::abs(m.pose.orientation.w-std::sqrt(.5))<1e-9;
  }
  double max_error=0;
  for (const auto &item:clouds) {
    const auto &m=item.second;
    ok=ok && m.header.frame_id=="world" && m.header.stamp.nsec==123 &&
        m.point_step==24 && m.width==2 && m.height==2 && !m.is_dense;
    for(int j=0;j<4;++j) {
      float xyz[3],range; uint16_t ring,column; uint8_t status;
      std::memcpy(xyz,m.data.data()+24*j,12);
      std::memcpy(&range,m.data.data()+24*j+12,4);
      std::memcpy(&ring,m.data.data()+24*j+16,2);
      std::memcpy(&column,m.data.data()+24*j+18,2);
      status=m.data[24*j+20];
      ok=ok && ring==j/2 && column==j%2;
      if(j==3) {ok=ok && std::isnan(xyz[0]) && std::isnan(xyz[1]) &&
          std::isnan(xyz[2]) && std::isinf(range) && status==0;continue;}
      const double el=j/2? .1:-.1, az=j%2? .1:-.1;
      // Independent analytic 90-degree world rotation + nonzero translation.
      const double expected[]={10-2*std::cos(el)*std::sin(az),
          20+2*std::cos(el)*std::cos(az),30+2*std::sin(el)};
      for(int k=0;k<3;++k)max_error=std::max(max_error,std::abs(xyz[k]-expected[k]));
      ok=ok && range==2 && status==1;
    }
  }
  ok=ok && max_error<3e-6;
  ros::NodeHandle priv("~");std::string output;priv.getParam("output",output);
  std::ofstream out(output);out << "{\"passed\":" << (ok?"true":"false") << ",\"clouds\":" << clouds.size() << ",\"poses\":" << poses.size() << ",\"max_xyz_error_m\":" << max_error << "}\n";
  gazebo::client::shutdown();return ok?0:1;
}
