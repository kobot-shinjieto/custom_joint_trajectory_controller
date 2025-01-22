#include <memory>
#include <pluginlib/class_loader.hpp>
#include "controller_interface/controller_interface.hpp"

int main(int argc, char ** argv)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader("controller_interface", "controller_interface::ControllerInterface");

  try
  {
    std::shared_ptr<controller_interface::ControllerInterface> plugin = loader.createSharedInstance("ros2_nyokkey4_controller/Ros2Nyokkey4Controller");
    // 必要に応じて、configure や他のメソッドを呼び出して動作確認
    std::cout << "Plugin loaded successfully!" << std::endl;
  }
  catch (const pluginlib::PluginlibException & ex)
  {
    std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
  }

  return 0;
}
