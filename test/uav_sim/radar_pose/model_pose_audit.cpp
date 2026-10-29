// Test-only world plugin: independent model pose at each physics step.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <fstream>
namespace gazebo {
class ModelPoseAudit : public WorldPlugin {
  physics::WorldPtr world_;
  event::ConnectionPtr update_;
  std::ofstream out_;
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    out_.open(sdf->Get<std::string>("output"));
    if (!out_) throw std::runtime_error("Cannot open model pose audit output");
    out_ << "sec,nsec,x,y,z,qx,qy,qz,qw\n";
    out_.precision(17);
    update_ = event::Events::ConnectWorldUpdateEnd([this]() {
      auto model = world_->ModelByName("static_uav");
      if (!model) return;
      const auto t = world_->SimTime();
      const auto p = model->WorldPose();
      out_ << t.sec << ',' << t.nsec << ',' << p.Pos().X() << ','
           << p.Pos().Y() << ',' << p.Pos().Z() << ',' << p.Rot().X() << ','
           << p.Rot().Y() << ',' << p.Rot().Z() << ',' << p.Rot().W() << '\n';
      out_.flush();
    });
  }
};
GZ_REGISTER_WORLD_PLUGIN(ModelPoseAudit)
}
