#include <canbus_test/master_can.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "master_can");
  ros::NodeHandle nh("");
  MasterCAN master(nh);

  master.start();
  master.reset_node();
  master.setup_node();

  while (!master.connected()) {
    std::cout << "Master not conneceted to device. " << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  master.communicate();
  return 0;
}