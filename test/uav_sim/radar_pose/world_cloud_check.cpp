// Exercise the production bridge math without requiring a ROS/Gazebo master.
#define main bridge_process_main
#include "../../../catkin_ws/src/uav_sim/src/ray_cloud_bridge.cpp"
#undef main
#include <iostream>
int main() {
  double maximum=0; int samples=0;
  for (const auto &body : std::vector<ignition::math::Pose3d>{
      {0,0,1.3,0,0,0}, {1.2,-.7,2.1,.18,-.12,1.1}, {-.6,.8,1.7,-.15,.2,-.8}}) {
    const ignition::math::Pose3d mount(.3,.02,.12,.1,-.2,.3);
    const auto sensor=body*mount;
    gazebo::msgs::LaserScan scan;
    gazebo::msgs::Set(scan.mutable_world_pose(),sensor);
    ignition::math::Pose3d parsed;
    if (!radar_world::ReadPose(scan,parsed))return 1;
    const auto recovered=parsed*mount.Inverse();
    maximum=std::max(maximum,(recovered.Pos()-body.Pos()).Length());
    if (!recovered.Rot().Equal(body.Rot(),1e-10))return 2;
    // Fixed world objects: inverse-project to native spherical scan, then use
    // the production point function. Translation/mount must be applied once.
    for (auto target : std::vector<ignition::math::Vector3d>{{4,1,2},{3,-2,.4},{8,.5,1}}) {
      const auto local=parsed.Rot().RotateVectorReverse(target-parsed.Pos());
      const double r=local.Length();
      const auto result=radar_world::Point(parsed,r,std::asin(local.Z()/r),
                                            std::atan2(local.Y(),local.X()));
      maximum=std::max(maximum,(result-target).Length());++samples;
      if(std::abs((result-parsed.Pos()).Length()-r)>1e-10)return 3;
    }
    scan.mutable_world_pose()->mutable_orientation()->set_w(0);
    scan.mutable_world_pose()->mutable_orientation()->set_x(0);
    scan.mutable_world_pose()->mutable_orientation()->set_y(0);
    scan.mutable_world_pose()->mutable_orientation()->set_z(0);
    if(radar_world::ReadPose(scan,parsed))return 4;
    gazebo::msgs::Set(scan.mutable_world_pose(),sensor);
    scan.mutable_world_pose()->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
    if(radar_world::ReadPose(scan,parsed))return 5;
    scan.clear_world_pose();if(radar_world::ReadPose(scan,parsed))return 6;
  }
  std::cout << "{\"fixed_world_samples\":" << samples
            << ",\"max_error_m\":" << maximum << "}\n";
  return maximum<1e-10?0:7;
}
