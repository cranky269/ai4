// Set only the spectator camera, once the actual model visuals have arrived.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <ignition/math/Pose3.hh>
#include <fstream>
#include <cstdlib>

namespace gazebo {
class AI4SceneView : public GUIPlugin {
 public:
  AI4SceneView() {
    // GUIPlugin is a QWidget; constrain its otherwise 640x480 empty overlay.
    this->setMaximumSize(1, 1);
    this->resize(1, 1);
    this->hide();
  }
  void Load(sdf::ElementPtr) override {
    connection_ = event::Events::ConnectPreRender([this]() { this->Apply(); });
  }
 private:
  void Apply() {
    if (applied_) return;
    auto camera = gui::get_active_camera();
    if (!camera || !camera->GetScene()) return;
    auto scene = camera->GetScene();
    if (!scene->GetVisual("static_uav") || !scene->GetVisual("target_uav")) return;
    camera->SetViewController("orbit");
    const ignition::math::Pose3d pose(2, -6, 4.2, 0, .43, 1.5707963267948966);
    camera->SetUseSDFPose(true);
    camera->SetInitialPose(pose);
    camera->SetWorldPose(pose);
    applied_ = true;
    const char *path = std::getenv("AI4_VIEW_STATUS");
    if (path) std::ofstream(path) << "spectator view applied after both model visuals arrived\n";
  }
  event::ConnectionPtr connection_;
  bool applied_ = false;
};
GZ_REGISTER_GUI_PLUGIN(AI4SceneView)
}
