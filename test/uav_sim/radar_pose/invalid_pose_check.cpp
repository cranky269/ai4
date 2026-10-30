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
  bool numeric_ok = true;
  std::set<unsigned> clouds;std::map<unsigned,geometry_msgs::PoseStamped> poses;
  auto c=nh.subscribe<sensor_msgs::PointCloud2>("/fixture/points",20,[&](const sensor_msgs::PointCloud2::ConstPtr &m){clouds.insert(m->header.stamp.sec);
    numeric_ok = numeric_ok && m->header.frame_id=="world" && m->point_step==24;
    const ignition::math::Pose3d expected(10,20,30,.2,-.3,.7);
    for (unsigned j=0;j<4;++j) {
      float x,y,z,r;
      std::memcpy(&x,m->data.data()+24*j,4);std::memcpy(&y,m->data.data()+24*j+4,4);
      std::memcpy(&z,m->data.data()+24*j+8,4);std::memcpy(&r,m->data.data()+24*j+12,4);
      if(j==3) { numeric_ok = numeric_ok && std::isnan(x) && std::isnan(y) && std::isnan(z) && std::isinf(r) && m->data[24*j+20]==0;continue; }
      double el=j/2? .1:-.1, az=j%2? .1:-.1;
      auto point=expected.Rot().RotateVector(ignition::math::Vector3d(2*cos(el)*cos(az),2*cos(el)*sin(az),2*sin(el)))+expected.Pos();
      numeric_ok = numeric_ok && (point-ignition::math::Vector3d(x,y,z)).Length()<3e-6 && r==2;
    }});
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
    gazebo::msgs::Set(s->mutable_world_pose(),ignition::math::Pose3d(10,20,30,.2,-.3,.7));
    if(i==1)s->mutable_world_pose()->mutable_orientation()->set_w(0);
    if(i==2)s->mutable_world_pose()->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
    s->set_angle_min(-.1);s->set_angle_max(.1);s->set_angle_step(.2);s->set_count(2);
    s->set_vertical_angle_min(-.1);s->set_vertical_angle_max(.1);s->set_vertical_angle_step(.2);s->set_vertical_count(2);
    s->set_range_min(.1);s->set_range_max(10);
    for(int j=0;j<4;++j){s->add_ranges(j==3?std::numeric_limits<double>::infinity():2);s->add_intensities(0);}
    pub->Publish(msg,true);
    for(int j=0;j<15;++j){ros::spinOnce();ros::WallDuration(.02).sleep();}
  }
  bool ok=numeric_ok && clouds.size()==2 && poses.size()==2 && poses.count(100) && poses.count(1);
  for(const auto &item:poses){const auto &m=item.second;
    ok=ok && m.header.stamp.nsec==123 && m.header.frame_id=="world" &&
      (ignition::math::Vector3d(m.pose.position.x,m.pose.position.y,m.pose.position.z)-
       (ignition::math::Pose3d(10,20,30,.2,-.3,.7)*ignition::math::Pose3d(.3,0,.12,0,0,0).Inverse()).Pos()).Length()<1e-9;
  }
  ros::NodeHandle priv("~");std::string output;priv.getParam("output",output);
  std::ofstream out(output);out << "{\"passed\":" << (ok?"true":"false") << ",\"clouds\":" << clouds.size() << ",\"poses\":" << poses.size() << "}\n";
  gazebo::client::shutdown();return ok?0:1;
}
